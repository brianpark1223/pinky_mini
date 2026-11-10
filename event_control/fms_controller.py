"""ROS2 Jazzy FMS controller for two (or more) Pinky robots.

One node manages every robot: one shared request queue, one pickup (STORE) used by
one robot at a time, one shared HOME (수납대) with parking slots, and a state
machine per robot. delivery_gui.py and robot_notifier.py read its status.

Default dry_run:=true: virtual arrival after 2 seconds, no navigation goals.
Real mode: fill POINTS, start each robot's Nav2 in its own namespace (/pinky_1, /pinky_2),
localize both, then pass -p dry_run:=false.

Per-robot state machine
  IDLE -> TO_STORE -> (DOCKING) -> WAIT_LOAD -> TO_STATION -> WAIT_RECEIVE -> TO_HOME -> IDLE
  WAIT_PICKUP : the pickup is used by the other robot; stopped where it is until it is free
  ERROR       : only this robot stops; retry resends the failed step, reset drops its task
                (and sends it back to the 수납대 when there is nothing else to do)

Services (std_srvs/srv/Trigger)
  /delivery/request_a, /delivery/request_b        shared queue
  /delivery/<robot>/loaded                         loading done at the pickup
  /delivery/<robot>/received                       parts taken out at the workstation
  /delivery/<robot>/retry, /delivery/<robot>/reset ERROR recovery
Status (std_msgs/msg/String JSON, /delivery/status, every 0.2 s)
  {"robots": {"pinky_1": {state, current, target, error, nav_ready, timeout_left,
                          distance_remaining, home}, ...},
   "queue": [...], "pickup": holder | null, "dry_run": bool, "points": {...}}

ArUco docking (docking:=true): after Nav2 reaches STORE the controller calls
  <ns>/delivery/dock_start (Trigger) and waits on <ns>/delivery/dock (String JSON):
  {"distance": m, "angle_deg": deg, "visible": bool, "state": "running" | "done" | "failed"}
  "done" -> WAIT_LOAD, "failed" or dock_timeout -> ERROR.

Who chooses which robot does what: task_allocation.py (assign, pickup_order).

Parameters
  robots 'pinky_1,pinky_2' | dry_run true | map_frame 'map'
  nav_action '/{robot}/navigate_to_pose' | pose_topic '/{robot}/amcl_pose'
  nav_timeout 90.0 s | dry_run_travel 2.0 s
  home_point 'HOME' | home_slot_offset 0.35 m (2nd robot parks this far to the left of HOME;
                      0 = both use HOME itself)
  docking false | dock_start '/{robot}/delivery/dock_start' | dock_topic '/{robot}/delivery/dock'
  dock_timeout 60.0 s | dry_run_dock 2.0 s

Do not send competing goals from RViz while this controller is running.
Stopping this process does not stop the robots: use each robot's own e-stop.
"""
import json
import math
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import task_allocation
except ImportError:  # run from another folder: load it next to this file
    import importlib.util
    import os
    _spec = importlib.util.spec_from_file_location(
        'task_allocation', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task_allocation.py'))
    task_allocation = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(task_allocation)

# Fixed map poses (m, m, rad) in the map frame of sesac_gangnam_2 (the driving corridor).
# Example values placed inside the corridor: replace with the measured ones.
POINTS = {
    'STORE': (1.15, 3.55, 1.57),     # 픽업대 (ArUco marker in front)
    'A': (0.85, 2.40, 3.14),         # 작업대1
    'B': (0.15, 0.45, 3.14),         # 작업대2
    'HOME': (0.60, 0.20, 1.57),      # 수납대 (one, shared by both robots)
}
STATIONS = ('A', 'B')
MOVING_STATES = ('TO_STORE', 'TO_STATION', 'TO_HOME')


class Robot:
    """Everything the controller tracks for one Pinky."""

    def __init__(self, name):
        self.name = name
        self.state = 'IDLE'
        self.current = None          # task being delivered
        self.target = None           # point name of the active/failed goal
        self.move_state = None       # state of the failed step (for retry)
        self.error = None
        self.nav = None              # ActionClient
        self.dock_cli = None         # Trigger client
        self.goal_seq = 0            # bumps on every goal; stale callbacks ignored
        self.goal_handle = None
        self.goal_pose = None        # (x, y, yaw) actually sent (HOME slot may differ)
        self.nav_deadline = None     # timeout (real) or virtual arrival (dry_run)
        self.dock_deadline = None
        self.distance_remaining = None
        self.pose = None             # (x, y) for allocation cost
        self.home_slot = None        # index of the HOME parking slot it uses
        self.wait_since = None       # entered WAIT_PICKUP
        self.go_home_when_free = False   # delivered, queue empty -> head home


class FmsController(Node):
    def __init__(self):
        super().__init__('fms_controller')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.dry_run = p('dry_run', True)
        self.frame = p('map_frame', 'map')
        self.nav_timeout = float(p('nav_timeout', 90.0))
        self.dry_travel = float(p('dry_run_travel', 2.0))
        self.home = p('home_point', 'HOME')
        self.home_offset = float(p('home_slot_offset', 0.35))
        self.docking = bool(p('docking', False))
        self.dock_timeout = float(p('dock_timeout', 60.0))
        self.dry_dock = float(p('dry_run_dock', 2.0))
        nav_fmt = p('nav_action', '/{robot}/navigate_to_pose')
        pose_fmt = p('pose_topic', '/{robot}/amcl_pose')
        dock_start_fmt = p('dock_start', '/{robot}/delivery/dock_start')
        dock_topic_fmt = p('dock_topic', '/{robot}/delivery/dock')
        names = [r.strip() for r in str(p('robots', 'pinky_1,pinky_2')).split(',') if r.strip()]
        self.points = {k: tuple(v) for k, v in POINTS.items()}

        self.robots = {}
        for name in names:
            r = Robot(name)
            r.nav = ActionClient(self, NavigateToPose, nav_fmt.format(robot=name))
            if self.docking:
                r.dock_cli = self.create_client(Trigger, dock_start_fmt.format(robot=name))
                self.create_subscription(String, dock_topic_fmt.format(robot=name),
                                         lambda m, r=r: self.on_dock(r, m), 10)
            if pose_fmt:
                self.create_subscription(PoseWithCovarianceStamped, pose_fmt.format(robot=name),
                                         lambda m, r=r: self.on_pose(r, m), 10)
            r.pose = self.home_slot_pose(len(self.robots))[:2] if self.dry_run else None
            r.home_slot = len(self.robots)       # everyone starts parked at the 수납대
            self.robots[name] = r

        self.queue = deque()
        self.next_id = 1
        self.pickup = None           # robot name holding the pickup
        self.last_wait_log = 0.0

        self.status_pub = self.create_publisher(String, 'delivery/status', 10)
        # NOTE: not self.services - rclpy Node already has a read-only `services` property.
        self.srvs = [
            self.create_service(Trigger, f'delivery/request_{st.lower()}',
                                lambda req, res, st=st: self.enqueue(st, res))
            for st in STATIONS]
        for name, r in self.robots.items():
            for srv, fn in (('loaded', self.loaded), ('received', self.received),
                            ('retry', self.retry), ('reset', self.reset)):
                self.srvs.append(self.create_service(
                    Trigger, f'delivery/{name}/{srv}', lambda req, res, r=r, fn=fn: fn(r, res)))

        self.tick_timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(f'Ready: robots={names}, dry_run={self.dry_run}, '
                               f'docking={self.docking}, nav_timeout={self.nav_timeout}s, '
                               f'allocator={getattr(task_allocation, "SOURCE", "default")}')

    # ------------------------------------------------------------------ helpers
    def log(self, r, text):
        self.get_logger().info(f'[{r.name}] {text}')

    def set_state(self, r, state):
        r.state = state
        self.log(r, f'{state} | task={r.current}')

    def nav_ready(self, r):
        return self.dry_run or r.nav.server_is_ready()

    def home_slot_pose(self, slot):
        """Parking slot i of the shared 수납대: slot 0 = HOME, slot 1 = home_slot_offset to its left."""
        x, y, yaw = self.points[self.home]
        if slot == 0 or self.home_offset == 0:
            return (x, y, yaw)
        d = self.home_offset * slot
        return (x - d * math.sin(yaw), y + d * math.cos(yaw), yaw)

    def on_pose(self, r, msg):
        p = msg.pose.pose.position
        r.pose = (p.x, p.y)

    def publish_status(self):
        now = time.monotonic()
        robots = {}
        for name, r in self.robots.items():
            left = None
            if r.nav_deadline is not None and r.state in MOVING_STATES:
                left = round(max(0.0, r.nav_deadline - now), 1)
            elif r.dock_deadline is not None and r.state == 'DOCKING':
                left = round(max(0.0, r.dock_deadline - now), 1)
            robots[name] = {
                'state': r.state,
                'current': r.current,
                'target': None if r.state in ('IDLE', 'WAIT_PICKUP') else r.target,
                'error': r.error,
                'nav_ready': self.nav_ready(r),
                'timeout_left': None if self.dry_run else left,
                'distance_remaining': r.distance_remaining if r.state in MOVING_STATES else None,
                'home': self.home,
            }
        status = {
            'robots': robots,
            'queue': list(self.queue),
            'pickup': self.pickup,
            'dry_run': self.dry_run,
            'points': {k: [round(c, 3) for c in v] for k, v in self.points.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # -------------------------------------------------------------------- loop
    def tick(self):
        now = time.monotonic()
        for r in self.robots.values():
            if r.nav_deadline is not None and now >= r.nav_deadline and r.state in MOVING_STATES:
                r.nav_deadline = None
                if self.dry_run:
                    self.arrived(r)
                else:
                    self.cancel_goal(r)
                    self.fail(r, f'Navigation timeout ({self.nav_timeout:.0f}s) to {r.target}')
            if r.dock_deadline is not None and now >= r.dock_deadline and r.state == 'DOCKING':
                r.dock_deadline = None
                if self.dry_run:
                    self.docked(r)
                else:
                    self.fail(r, f'Docking timeout ({self.dock_timeout:.0f}s)')

        # 1) hand new requests to free robots (task_allocation.assign decides who)
        free = {n: {'pose': r.pose, 'state': r.state} for n, r in self.robots.items()
                if r.state in ('IDLE', 'TO_HOME') and r.current is None and self.nav_ready(r)}
        if self.queue and free:
            for name, task in task_allocation.assign(free, list(self.queue), self.points):
                r = self.robots[name]
                self.queue.remove(task)
                task = dict(task, robot=name)
                r.current = task
                r.go_home_when_free = False
                r.home_slot = None
                if r.state == 'TO_HOME':
                    self.log(r, 'New request: abandoning return to HOME')
                self.log(r, f'Assigned {task}')
                self.go_pickup(r)
        elif self.queue and now - self.last_wait_log > 5.0 and not free:
            self.last_wait_log = now
            if not any(self.nav_ready(r) for r in self.robots.values()):
                self.get_logger().warn('Waiting for Nav2 action servers...')

        # 2) pickup free -> let the next waiting robot in (task_allocation.pickup_order)
        if self.pickup is None:
            waiting = {n: {'pose': r.pose, 'since': r.wait_since, 'task': r.current}
                       for n, r in self.robots.items() if r.state == 'WAIT_PICKUP'}
            name = task_allocation.pickup_order(waiting, self.points)
            if name is not None:
                self.go_pickup(self.robots[name])

        # 3) delivered and nothing left to do -> back to the 수납대
        for r in self.robots.values():
            if r.state == 'IDLE' and r.go_home_when_free and not self.queue:
                r.go_home_when_free = False
                self.go_home(r)

        self.publish_status()

    # ---------------------------------------------------------- shared places
    def go_pickup(self, r):
        """Drive to the pickup if it is free (and take it), else wait where it is."""
        if self.pickup in (None, r.name):
            self.pickup = r.name
            r.wait_since = None
            self.move(r, 'STORE', 'TO_STORE')
        else:
            self.cancel_goal(r)              # stop (e.g. abandon the trip home)
            r.target = None
            r.distance_remaining = None
            r.wait_since = time.monotonic()
            self.set_state(r, 'WAIT_PICKUP')

    def release_pickup(self, r):
        if self.pickup == r.name:
            self.pickup = None
            self.log(r, 'Pickup released')

    def go_home(self, r):
        used = {o.home_slot for o in self.robots.values() if o is not r and o.home_slot is not None}
        slot = next(i for i in range(len(self.robots) + 1) if i not in used)
        r.home_slot = slot
        self.move(r, self.home, 'TO_HOME', pose=self.home_slot_pose(slot))

    # -------------------------------------------------------------- navigation
    def move(self, r, location, state, pose=None):
        r.goal_seq += 1
        seq = r.goal_seq
        r.target = location
        r.move_state = state
        r.goal_handle = None
        r.error = None
        r.distance_remaining = None
        r.goal_pose = pose or self.points[location]
        self.set_state(r, state)
        self.log(r, f'Goal #{seq}: {location}')

        if self.dry_run:
            r.nav_deadline = time.monotonic() + self.dry_travel
            return
        if not r.nav.server_is_ready():
            self.fail(r, 'Nav2 action server not ready')
            return
        x, y, yaw = r.goal_pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        r.nav_deadline = time.monotonic() + self.nav_timeout
        r.nav.send_goal_async(
            goal, feedback_callback=lambda fb: self.nav_feedback(r, fb, seq)
        ).add_done_callback(lambda fut: self.goal_response(r, fut, seq))

    def nav_feedback(self, r, msg, seq):
        if seq == r.goal_seq:
            r.distance_remaining = round(float(msg.feedback.distance_remaining), 2)

    def goal_response(self, r, future, seq):
        if seq != r.goal_seq:
            return  # superseded goal
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail(r, f'send_goal failed: {exc}')
            return
        if not handle.accepted:
            self.fail(r, f'Nav2 rejected goal to {r.target}')
            return
        r.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut: self.nav_result(r, fut, seq))

    def nav_result(self, r, future, seq):
        if seq != r.goal_seq or r.state not in MOVING_STATES:
            return  # superseded, timed out, reset, or now waiting
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail(r, f'get_result failed: {exc}')
            return
        r.goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.arrived(r)
        else:
            self.fail(r, f'Navigation to {r.target} ended: status={result.status}')

    def cancel_goal(self, r):
        r.goal_seq += 1  # make pending callbacks stale
        r.nav_deadline = None
        if r.goal_handle is not None:
            self.log(r, 'Cancelling active Nav2 goal')
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None

    def arrived(self, r):
        r.nav_deadline = None
        if self.dry_run and r.goal_pose is not None:
            r.pose = (r.goal_pose[0], r.goal_pose[1])      # real runs: pose from amcl_pose
        if r.state == 'TO_STORE':
            if self.docking:
                self.start_docking(r)
            else:
                self.set_state(r, 'WAIT_LOAD')
        elif r.state == 'TO_STATION':
            self.set_state(r, 'WAIT_RECEIVE')
        elif r.state == 'TO_HOME':
            r.target = None
            self.set_state(r, 'IDLE')

    # ---------------------------------------------------------------- docking
    def start_docking(self, r):
        r.target = 'STORE'
        r.move_state = 'DOCKING'
        r.error = None
        self.set_state(r, 'DOCKING')
        if self.dry_run:
            r.dock_deadline = time.monotonic() + self.dry_dock
            return
        if r.dock_cli is None or not r.dock_cli.service_is_ready():
            self.fail(r, 'Docking service not ready')
            return
        r.dock_deadline = time.monotonic() + self.dock_timeout
        r.dock_cli.call_async(Trigger.Request())

    def on_dock(self, r, msg):
        if r.state != 'DOCKING':
            return
        try:
            fb = json.loads(msg.data)
        except ValueError:
            return
        if fb.get('state') == 'done':
            self.docked(r)
        elif fb.get('state') == 'failed':
            r.dock_deadline = None
            self.fail(r, f'Docking failed: {fb.get("reason", "")}'.rstrip(': '))

    def docked(self, r):
        r.dock_deadline = None
        self.set_state(r, 'WAIT_LOAD')

    # ---------------------------------------------------------------- services
    def enqueue(self, station, response):
        task = {'id': self.next_id, 'station': station}
        self.next_id += 1
        self.queue.append(task)
        response.success = True
        response.message = f'Queued: {task} (position {len(self.queue)})'
        return response

    def loaded(self, r, response):
        response.success = r.state == 'WAIT_LOAD'
        response.message = f'{r.name}: loading confirmed' if response.success \
            else f'{r.name} is not waiting for loading (state={r.state})'
        if response.success:
            self.release_pickup(r)
            self.move(r, r.current['station'], 'TO_STATION')
        return response

    def received(self, r, response):
        response.success = r.state == 'WAIT_RECEIVE'
        response.message = f'{r.name}: delivery completed' if response.success \
            else f'{r.name} is not waiting for receipt (state={r.state})'
        if response.success:
            self.log(r, f'COMPLETED: {r.current}')
            r.current = None
            r.target = None
            r.go_home_when_free = True      # next tick: new task, or back to the 수납대
            self.set_state(r, 'IDLE')
        return response

    def retry(self, r, response):
        if r.state != 'ERROR' or r.move_state is None:
            response.success = False
            response.message = f'{r.name}: nothing to retry (state={r.state})'
            return response
        if not self.nav_ready(r):
            response.success = False
            response.message = f'{r.name}: Nav2 action server still not ready'
            return response
        response.success = True
        response.message = f'{r.name}: retrying {r.move_state} -> {r.target}'
        step = r.move_state
        if step == 'TO_STORE':
            self.go_pickup(r)
        elif step == 'DOCKING':
            if self.pickup in (None, r.name):
                self.pickup = r.name
                self.start_docking(r)
            else:
                self.go_pickup(r)
        elif step == 'TO_HOME':
            self.go_home(r)
        else:
            self.move(r, r.target, step)
        return response

    def reset(self, r, response):
        dropped = r.current
        self.cancel_goal(r)
        self.release_pickup(r)
        r.current = None
        r.target = None
        r.move_state = None
        r.error = None
        r.dock_deadline = None
        r.wait_since = None
        r.go_home_when_free = True       # free robot: next task, or back to the 수납대
        self.set_state(r, 'IDLE')
        response.success = True
        response.message = (f'{r.name}: reset. Dropped task: {dropped}. '
                            f'{len(self.queue)} task(s) still queued.')
        self.get_logger().warn(response.message)
        return response

    def fail(self, r, message):
        r.nav_deadline = None
        r.dock_deadline = None
        r.error = message
        if r.state == 'TO_STORE':
            self.release_pickup(r)   # not there yet: let the other robot use it
        # DOCKING failed: it is physically at the pickup, keep it until retry/reset
        self.get_logger().error(f'[{r.name}] {message}')
        self.set_state(r, 'ERROR')


def main():
    rclpy.init()
    node = FmsController()
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
