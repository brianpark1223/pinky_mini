#!/usr/bin/env python3
"""ROS2 Jazzy single-robot delivery controller (v2).

Default dry_run:=true: virtual arrival after 2 seconds, no navigation goals.
Real mode: edit POINTS, start/localize Nav2, then pass -p dry_run:=false.

Services (std_srvs/srv/Trigger):
  /delivery/request_a, /delivery/request_b : enqueue delivery
  /delivery/loaded                         : confirm loading at STORE
  /delivery/received                       : confirm unloading at workstation
  /delivery/retry                          : ERROR -> resend the failed goal
  /delivery/reset                          : ERROR/any -> drop current task, IDLE
Status (std_msgs/msg/String): /delivery/status (JSON, every 0.2 s)

All points are fixed in POINTS below (map frame). For two robots the GUI expects the
status format in README "팀 인터페이스 · 2대" (robots dict, per-robot services);
this file still drives one robot.

Parameters:
  dry_run (bool, true), map_frame (str, 'map'),
  nav_timeout (float sec, 90.0), dry_run_travel (float sec, 2.0),
  robot_name (str, 'pinky_1'), home_point (str, 'HOME')

Do not send competing goals from RViz while this controller is running.
Stopping this process does not stop the robot: use the robot's own e-stop.
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Fixed map poses (m, m, rad) in the map frame of sesac_gangnam_2 (the driving corridor).
# Example values placed inside the corridor: replace with the measured ones.
POINTS = {
    'STORE': (1.15, 3.55, 1.57),     # 픽업대 (ArUco marker in front)
    'A': (0.85, 2.40, 3.14),         # 작업대1
    'B': (0.15, 0.45, 3.14),         # 작업대2
    'HOME': (0.60, 0.20, 1.57),      # 수납대 (one, shared by both robots)
}

MOVING_STATES = ('TO_STORE', 'TO_STATION', 'TO_HOME')


class PartsDelivery(Node):
    def __init__(self):
        super().__init__('parts_delivery')
        self.dry_run = self.declare_parameter('dry_run', True).value
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.nav_timeout = float(self.declare_parameter('nav_timeout', 90.0).value)
        self.dry_travel = float(self.declare_parameter('dry_run_travel', 2.0).value)
        self.robot_name = self.declare_parameter('robot_name', 'pinky_1').value
        self.home = self.declare_parameter('home_point', 'HOME').value
        self.points = {k: tuple(v) for k, v in POINTS.items()}

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.queue = deque()
        self.current = None
        self.state = 'IDLE'
        self.next_id = 1
        self.error = None

        # Active navigation bookkeeping.
        self.target = None          # location name of the active/failed goal
        self.move_state = None      # state used for that goal (for retry)
        self.goal_seq = 0           # bumps on every new goal; stale callbacks ignored
        self.goal_handle = None
        self.nav_deadline = None    # timeout (real) or virtual arrival (dry_run)
        self.last_wait_log = 0.0
        self.distance_remaining = None  # from Nav2 feedback (m)

        self.status_pub = self.create_publisher(String, 'delivery/status', 10)
        # NOTE: do not name this self.services - rclpy Node already has a
        # read-only `services` property, and assigning it raises AttributeError.
        self.srvs = [
            self.create_service(Trigger, 'delivery/request_a',
                                lambda req, res: self.enqueue('A', res)),
            self.create_service(Trigger, 'delivery/request_b',
                                lambda req, res: self.enqueue('B', res)),
            self.create_service(Trigger, 'delivery/loaded', self.loaded),
            self.create_service(Trigger, 'delivery/received', self.received),
            self.create_service(Trigger, 'delivery/retry', self.retry),
            self.create_service(Trigger, 'delivery/reset', self.reset),
        ]

        self.tick_timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'Ready: dry_run={self.dry_run}, nav_timeout={self.nav_timeout}s')

    # ------------------------------------------------------------------ helpers
    def set_state(self, state):
        self.state = state
        self.get_logger().info(f'{state} | task={self.current}')

    def nav_ready(self):
        return self.dry_run or self.nav.server_is_ready()

    def publish_status(self):
        remaining = None
        if self.nav_deadline is not None and self.state in MOVING_STATES:
            remaining = round(max(0.0, self.nav_deadline - time.monotonic()), 1)
        status = {
            'state': self.state,
            'current': self.current,
            'target': self.target if self.state in MOVING_STATES + ('ERROR',) else None,
            'queue': list(self.queue),
            'error': self.error,
            'nav_ready': self.nav_ready(),
            'timeout_left': None if self.dry_run else remaining,
            'distance_remaining': self.distance_remaining
            if self.state in MOVING_STATES else None,
            'dry_run': self.dry_run,
            'robot': self.robot_name,
            'home': self.home,
            'points': {k: [round(c, 3) for c in v] for k, v in self.points.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # -------------------------------------------------------------------- loop
    def tick(self):
        now = time.monotonic()

        # Start (or preempt the home trip for) the next task.
        if self.queue and self.state in ('IDLE', 'TO_HOME'):
            if self.nav_ready():
                if self.state == 'TO_HOME':
                    self.get_logger().info(f'New request: abandoning return to {self.home}')
                self.current = self.queue.popleft()
                self.move('STORE', 'TO_STORE')
            elif now - self.last_wait_log > 5.0:
                self.last_wait_log = now
                self.get_logger().warn('Waiting for Nav2 action server...')

        # Virtual arrival (dry_run) or navigation timeout (real).
        if self.nav_deadline is not None and now >= self.nav_deadline \
                and self.state in MOVING_STATES:
            self.nav_deadline = None
            if self.dry_run:
                self.arrived()
            else:
                self.cancel_active_goal()
                self.fail(f'Navigation timeout ({self.nav_timeout:.0f}s) to {self.target}')

        self.publish_status()

    # -------------------------------------------------------------- navigation
    def move(self, location, state):
        self.goal_seq += 1
        seq = self.goal_seq
        self.target = location
        self.move_state = state
        self.goal_handle = None
        self.error = None
        self.distance_remaining = None
        self.set_state(state)
        self.get_logger().info(f'Goal #{seq}: {location}')

        if self.dry_run:
            self.nav_deadline = time.monotonic() + self.dry_travel
            return
        if not self.nav.server_is_ready():
            self.fail('Nav2 action server not ready')
            return

        x, y, yaw = self.points[location]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_deadline = time.monotonic() + self.nav_timeout
        self.nav.send_goal_async(
            goal, feedback_callback=lambda fb: self.nav_feedback(fb, seq)
        ).add_done_callback(lambda fut: self.goal_response(fut, seq))

    def nav_feedback(self, msg, seq):
        if seq == self.goal_seq:
            self.distance_remaining = round(float(msg.feedback.distance_remaining), 2)

    def goal_response(self, future, seq):
        if seq != self.goal_seq:
            return  # superseded goal
        try:
            handle = future.result()
        except Exception as exc:
            self.fail(f'send_goal failed: {exc}')
            return
        if not handle.accepted:
            self.fail(f'Nav2 rejected goal to {self.target}')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda fut: self.nav_result(fut, seq))

    def nav_result(self, future, seq):
        if seq != self.goal_seq or self.state not in MOVING_STATES:
            return  # superseded, timed out, or reset
        try:
            result = future.result()
        except Exception as exc:
            self.fail(f'get_result failed: {exc}')
            return
        self.goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.arrived()
        else:
            self.fail(f'Navigation to {self.target} ended: status={result.status}')

    def cancel_active_goal(self):
        self.goal_seq += 1  # make any pending callbacks stale
        self.nav_deadline = None
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling active Nav2 goal')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def arrived(self):
        self.nav_deadline = None
        if self.state == 'TO_STORE':
            self.set_state('WAIT_LOAD')
        elif self.state == 'TO_STATION':
            self.set_state('WAIT_RECEIVE')
        elif self.state == 'TO_HOME':
            self.target = None
            self.set_state('IDLE')

    # ---------------------------------------------------------------- services
    def enqueue(self, station, response):
        task = {'id': self.next_id, 'station': station}
        self.next_id += 1
        self.queue.append(task)
        response.success = True
        response.message = f'Queued: {task} (position {len(self.queue)})'
        if self.state == 'ERROR':
            response.message += ' - controller in ERROR, call retry or reset'
        return response

    def loaded(self, request, response):
        response.success = self.state == 'WAIT_LOAD'
        response.message = 'Loading confirmed' if response.success \
            else f'Not waiting for loading (state={self.state})'
        if response.success:
            self.move(self.current['station'], 'TO_STATION')
        return response

    def received(self, request, response):
        response.success = self.state == 'WAIT_RECEIVE'
        response.message = 'Delivery completed' if response.success \
            else f'Not waiting for receipt (state={self.state})'
        if response.success:
            self.get_logger().info(f'COMPLETED: {self.current}')
            self.current = None
            if self.queue:
                self.target = None
                self.set_state('IDLE')  # next tick starts next delivery
            else:
                self.move(self.home, 'TO_HOME')
        return response

    def retry(self, request, response):
        if self.state != 'ERROR' or self.move_state is None:
            response.success = False
            response.message = f'Nothing to retry (state={self.state})'
            return response
        if not self.nav_ready():
            response.success = False
            response.message = 'Nav2 action server still not ready'
            return response
        response.success = True
        response.message = f'Retrying {self.move_state} -> {self.target}'
        self.move(self.target, self.move_state)
        return response

    def reset(self, request, response):
        dropped = self.current
        self.cancel_active_goal()
        self.current = None
        self.target = None
        self.move_state = None
        self.error = None
        self.set_state('IDLE')
        response.success = True
        response.message = (f'Reset. Dropped task: {dropped}. '
                            f'{len(self.queue)} task(s) still queued.')
        self.get_logger().warn(response.message)
        return response

    def fail(self, message):
        self.nav_deadline = None
        self.error = message
        self.get_logger().error(message)
        self.set_state('ERROR')


def main():
    rclpy.init()
    node = PartsDelivery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
