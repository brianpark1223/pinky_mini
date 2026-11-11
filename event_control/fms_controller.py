"""ROS2 Jazzy FMS controller for two (or more) Pinky robots.

One node manages every robot: one shared request queue, one pickup (STORE) used by
one robot at a time, one shared HOME (수납대) with parking slots, and a state
machine per robot. delivery_gui.py and robot_notifier.py read its status.

Default dry_run:=false: real mode (measured configuration and /delivery/arm required).
For virtual arrivals without navigation goals, run fms_sim.py.
Real mode: provide measured points_json/file and home_slots_json/file,
start namespaced Nav2, localize both, then explicitly call /delivery/arm.

Per-robot state machine
  IDLE -> TO_STORE -> (DOCKING) -> WAIT_LOAD -> TO_STATION -> WAIT_RECEIVE -> TO_HOME -> IDLE
  WAIT_PICKUP : the pickup is used by the other robot; stopped where it is until it is free
  ERROR       : only this robot stops; retry resends the failed step, reset drops its task

Services (std_srvs/srv/Trigger)
  /delivery/request_a, /delivery/request_b        shared queue
  /delivery/arm, /delivery/disarm              enable dispatch / request Nav2 cancel
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

Who chooses which robot does what: task_allocation.py -> ../task_allocation/allocator.py.
  Allocator.dispatch() picks, among free robots, the one with the lowest cost for each
  request whose station is not locked; on 수령 완료 Allocator.next_for() hands the robot the
  next assignable request (straight to the pickup, no trip home). Every robot's state, pose
  and time in state are passed so the cost can include the other robot's pickup use.
  Missing / failing allocator -> default rules (nearest free robot, first come first served).

Parameters
  robots 'pinky_1,pinky_2' | dry_run false (fms_sim.py sets true) | map_frame 'map'
  nav_action '/{robot}/navigate_to_pose' | pose_topic '/{robot}/amcl_pose'
  nav_timeout 90.0 s | dry_run_travel 2.0 s
  pose_timeout 3.0 s | cancel_timeout 5.0 s
  pickup_clearance 0.8 m | station_clearance 0.6 m (real)
  points_file JSON | home_slots_file JSON (both required in real two-robot mode)
  home_point 'HOME' | home_slot_offset 0.35 m (2nd robot parks this far to the left of HOME;
                      0 = both use HOME itself)
  docking false | dock_start '/{robot}/delivery/dock_start' | dock_topic '/{robot}/delivery/dock'
  dock_timeout 60.0 s | dry_run_dock 2.0 s
  avg_speed 0.15 m/s | t_loading 20.0 s | t_unloading 15.0 s | detour_factor 1.25 (allocation cost)

Do not send competing goals from RViz while this controller is running.
The FMS cancellation service is not a hardware e-stop; keep the physical stop available.
"""
import json
import math
import signal
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fleet_metrics import battery_fraction, euclidean_distance, path_distance
from traffic_manager import TrafficManager

try:
    import task_allocation
except ImportError:  # run from another folder: load it next to this file
    import importlib.util
    import os
    _spec = importlib.util.spec_from_file_location(
        'task_allocation', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task_allocation.py'))
    task_allocation = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(task_allocation)

# Map poses (x, y, yaw) in the sesac_gangnam_2 map frame. XY were supplied from
# map-frame PointStamped measurements; yaw values remain the previous defaults.
POINTS = {
    'STORE': (0.093633, 0.990360, 1.57),  # 픽업대 (기존 수납대 위치)
    'A': (1.376683, 2.648840, 3.14),      # 작업대1
    'B': (1.363277, 1.327869, 3.14),      # 작업대2 (기존 픽업대 위치)
    'HOME': (1.060465, 3.829644, 1.57),   # 수납대 기준점 (기존 작업대2 위치)
}
STATIONS = ('A', 'B')
MOVING_STATES = ('TO_STORE', 'TO_STATION', 'TO_HOME')
WAITING_STATES = ('WAIT_PICKUP', 'WAIT_TRAFFIC', 'WAIT_LOAD', 'WAIT_RECEIVE')


class Robot:
    """Everything the controller tracks for one Pinky."""

    def __init__(self, name):
        self.name = name
        self.state = 'IDLE'
        self.state_since = time.monotonic()   # for allocation (remaining loading / unloading)
        self.current = None          # task being delivered
        self.target = None           # point name of the active/failed goal
        self.move_state = None       # state of the failed step (for retry)
        self.error = None
        self.nav = None              # ActionClient
        self.planner = None          # ComputePathToPose ActionClient
        self.dock_cli = None         # Trigger client
        self.goal_seq = 0            # bumps on every goal; stale callbacks ignored
        self.goal_handle = None
        self.goal_pending = False
        self.stop_reason = None
        self.stop_callback = None
        self.stop_started = None
        self.goal_pose = None        # (x, y, yaw) actually sent (HOME slot may differ)
        self.goal_started_at = None
        self.departure_started_at = None
        self.nav_deadline = None     # timeout (real) or virtual arrival (dry_run)
        self.dock_deadline = None
        self.distance_remaining = None
        self.pose = None             # (x, y) for allocation cost
        self.pose_seen = None
        self.battery_percent = None
        self.battery_seen = None
        self.path_distance = None
        self.distance_to_store = None
        self.path_distance_seen = None
        self.path_target = None
        self.path_origin = None
        self.path_source = None
        self.path_pending = False
        self.path_request_started = None
        self.path_seq = 0
        self.nav_missing_since = None
        self.pending_move = None
        self.traffic_direction = None
        self.traffic_waypoint_pending = False
        self.last_location = 'HOME'
        self.allocation_cost = None
        self.home_slot = None        # index of the HOME parking slot it uses
        self.wait_since = None       # entered WAIT_PICKUP
        self.go_home_when_free = False   # delivered, queue empty -> head home


class FmsController(Node):
    def __init__(self):
        super().__init__('fms_controller')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.dry_run = p('dry_run', False)
        self.frame = p('map_frame', 'map')
        self.nav_timeout = float(p('nav_timeout', 90.0))
        self.dry_travel = float(p('dry_run_travel', 2.0))
        self.home = p('home_point', 'HOME')
        self.home_offset = float(p('home_slot_offset', 0.35))
        self.home_min_separation = float(p('home_min_separation', 0.5))
        self.docking = bool(p('docking', False))
        self.pose_timeout = float(p('pose_timeout', 3.0))
        self.offline_timeout = float(p('offline_timeout', 5.0))
        self.battery_timeout = float(p('battery_timeout', 10.0))
        self.low_battery = float(p('low_battery_percent', 0.20))
        self.critical_battery = float(p('critical_battery_percent', 0.08))
        self.battery_required = bool(p('battery_required', not self.dry_run))
        self.path_cost_timeout = float(p('path_cost_timeout', 10.0))
        self.path_query_wait = float(p('path_query_wait', 0.8))
        self.cost_distance_weight = float(p('cost_distance_weight', 0.6))
        self.cost_battery_weight = float(p('cost_battery_weight', 0.2))
        self.cost_load_weight = float(p('cost_load_weight', 0.2))
        self.cancel_timeout = float(p('cancel_timeout', 5.0))
        self.pickup_clearance = float(p('pickup_clearance', 0.8))
        self.station_clearance = float(p('station_clearance', 0.2 if self.dry_run else 0.6))
        self.dock_timeout = float(p('dock_timeout', 60.0))
        self.dry_dock = float(p('dry_run_dock', 2.0))
        self.avg_speed = float(p('avg_speed', 0.15))
        self.t_loading = float(p('t_loading', 20.0))
        self.t_unloading = float(p('t_unloading', 15.0))
        self.detour = float(p('detour_factor', 1.25))
        nav_fmt = p('nav_action', '/{robot}/navigate_to_pose')
        path_fmt = p('path_action', '/{robot}/compute_path_to_pose')
        pose_fmt = p('pose_topic', '/{robot}/amcl_pose')
        battery_fmt = p('battery_topic', '/{robot}/battery_state')
        dock_start_fmt = p('dock_start', '/{robot}/delivery/dock_start')
        dock_topic_fmt = p('dock_topic', '/{robot}/delivery/dock')
        names = [r.strip() for r in str(p('robots', 'pinky_1,pinky_2')).split(',') if r.strip()]
        points_json = str(p('points_json', '')).strip()
        points_file = str(p('points_file', '')).strip()
        if points_json and points_file:
            raise ValueError('Specify either points_json or points_file')
        if not self.dry_run and not (points_json or points_file):
            raise ValueError('Real mode requires measured points_json or points_file')
        if points_file:
            with open(points_file, encoding='utf-8') as stream:
                supplied = json.load(stream)
        else:
            supplied = json.loads(points_json) if points_json else POINTS
        if not isinstance(supplied, dict) or set(supplied) != set(POINTS):
            raise ValueError('points_json must contain exactly STORE, A, B and HOME')
        self.points = {}
        for name, values in supplied.items():
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError(f'{name} must be [x, y, yaw]')
            try:
                pose = tuple(float(value) for value in values)
            except (TypeError, ValueError) as exc:
                raise ValueError(f'{name} requires measured numeric x, y and yaw') from exc
            if not all(math.isfinite(value) for value in pose):
                raise ValueError(f'{name} has a non-finite coordinate')
            self.points[name] = pose
        home_slots_json = str(p('home_slots_json', '')).strip()
        home_slots_file = str(p('home_slots_file', '')).strip()
        if home_slots_json and home_slots_file:
            raise ValueError('Specify either home_slots_json or home_slots_file')
        if not self.dry_run and len(names) > 1 and not (home_slots_json or home_slots_file):
            raise ValueError('Real two-robot mode requires measured home_slots_file')
        if home_slots_file:
            with open(home_slots_file, encoding='utf-8') as stream:
                raw_slots = json.load(stream)
        else:
            raw_slots = json.loads(home_slots_json) if home_slots_json else None
        self.measured_home_slots = None
        if raw_slots is not None:
            if not isinstance(raw_slots, list) or len(raw_slots) != len(names):
                raise ValueError('home slots must contain one [x, y, yaw] pose per robot')
            self.measured_home_slots = []
            for values in raw_slots:
                if not isinstance(values, list) or len(values) != 3:
                    raise ValueError('each home slot must be [x, y, yaw]')
                try:
                    pose = tuple(float(value) for value in values)
                except (TypeError, ValueError) as exc:
                    raise ValueError('home slots require measured numeric coordinates') from exc
                if not all(math.isfinite(value) for value in pose):
                    raise ValueError('home slot has a non-finite coordinate')
                self.measured_home_slots.append(pose)
            for i, first in enumerate(self.measured_home_slots):
                for second in self.measured_home_slots[i + 1:]:
                    if math.hypot(first[0] - second[0], first[1] - second[1]) \
                            < self.home_min_separation:
                        raise ValueError('home slots are closer than home_min_separation')
        self.traffic_enabled = bool(p('traffic_enabled', not self.dry_run))
        traffic_json = str(p('traffic_json', '')).strip()
        traffic_file = str(p('traffic_file', '')).strip()
        if traffic_json and traffic_file:
            raise ValueError('Specify either traffic_json or traffic_file')
        if self.traffic_enabled and not self.dry_run and not (traffic_json or traffic_file):
            raise ValueError('Real mode requires a measured traffic_file when traffic_enabled=true')
        if traffic_file:
            with open(traffic_file, encoding='utf-8') as stream:
                traffic_config = json.load(stream)
        elif traffic_json:
            traffic_config = json.loads(traffic_json)
        else:
            traffic_config = {'enabled': self.traffic_enabled, 'corridor_axis': [[0, 0], [1, 0]]}
        traffic_config['enabled'] = self.traffic_enabled
        self.traffic = TrafficManager(traffic_config)
        if self.traffic_enabled and not self.dry_run \
                and any(self.traffic.waypoint(direction) is None
                        for direction in self.traffic.DIRECTIONS):
            raise ValueError('Real traffic config requires measured entry_waypoint and exit_waypoint')
        if len(names) != len(set(names)) or not names or self.pose_timeout <= 0 \
                or self.pickup_clearance <= 0 or self.station_clearance <= 0 \
                or self.cancel_timeout <= 0 or self.nav_timeout <= 0 \
                or self.home_min_separation <= 0 or self.home not in self.points \
                or self.offline_timeout <= 0 or self.battery_timeout <= 0 \
                or not 0 <= self.critical_battery < self.low_battery <= 1 \
                or self.path_cost_timeout <= 0 or self.path_query_wait < 0 \
                or min(self.cost_distance_weight, self.cost_battery_weight,
                       self.cost_load_weight) < 0:
            raise ValueError('Invalid robot names or safety parameters')

        self.robots = {}
        for name in names:
            r = Robot(name)
            r.nav = ActionClient(self, NavigateToPose, nav_fmt.format(robot=name))
            r.planner = ActionClient(self, ComputePathToPose, path_fmt.format(robot=name))
            if self.docking:
                r.dock_cli = self.create_client(Trigger, dock_start_fmt.format(robot=name))
                self.create_subscription(String, dock_topic_fmt.format(robot=name),
                                         lambda m, r=r: self.on_dock(r, m), 10)
            if pose_fmt:
                self.create_subscription(PoseWithCovarianceStamped, pose_fmt.format(robot=name),
                                         lambda m, r=r: self.on_pose(r, m), 10)
            if battery_fmt:
                self.create_subscription(BatteryState, battery_fmt.format(robot=name),
                                         lambda m, r=r: self.on_battery(r, m), 10)
            r.pose = self.home_slot_pose(len(self.robots))[:2] if self.dry_run else None
            r.pose_seen = time.monotonic() if self.dry_run else None
            r.battery_percent = 1.0 if self.dry_run else None
            r.battery_seen = time.monotonic() if self.dry_run else None
            r.home_slot = len(self.robots)       # everyone starts parked at the 수납대
            self.robots[name] = r

        self.queue = deque()
        self.next_id = 1
        self.pickup = None           # robot name holding the pickup
        self.station_owners = {name: None for name in STATIONS}
        self.station_release_pending = set()
        self.armed = self.dry_run
        self.shutting_down = False
        self.last_wait_log = 0.0

        self.status_pub = self.create_publisher(String, 'delivery/status', 10)
        # NOTE: not self.services - rclpy Node already has a read-only `services` property.
        self.srvs = [
            self.create_service(Trigger, f'delivery/request_{st.lower()}',
                                lambda req, res, st=st: self.enqueue(st, res))
            for st in STATIONS]
        self.srvs.append(self.create_service(Trigger, 'delivery/arm', self.arm))
        self.srvs.append(self.create_service(Trigger, 'delivery/disarm', self.disarm))
        for name, r in self.robots.items():
            for srv, fn in (('loaded', self.loaded), ('received', self.received),
                            ('retry', self.retry), ('reset', self.reset),
                            ('traffic_clear', self.traffic_clear)):
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
        if state != r.state:
            r.state_since = time.monotonic()
        r.state = state
        self.log(r, f'{state} | task={r.current}')

    def nav_ready(self, r):
        return self.dry_run or r.nav.server_is_ready()

    def battery_ready(self, r):
        return r.battery_percent is not None and r.battery_seen is not None and (
            self.dry_run or time.monotonic() - r.battery_seen <= self.battery_timeout)

    def robot_online(self, r):
        return self.pose_ready(r) and self.nav_ready(r) \
            and (self.battery_ready(r) or not self.battery_required)

    def robot_eligible(self, r):
        return self.robot_online(r) and (
            not self.battery_required or r.battery_percent >= self.low_battery)

    def pose_ready(self, r):
        return r.pose is not None and r.pose_seen is not None and (
            self.dry_run or time.monotonic() - r.pose_seen <= self.pose_timeout)

    def clear_of_pickup(self, r):
        return self.pose_ready(r) and math.hypot(
            r.pose[0] - self.points['STORE'][0], r.pose[1] - self.points['STORE'][1]
        ) >= self.pickup_clearance

    def pickup_area_clear(self, requesting):
        return all(other is requesting or self.clear_of_pickup(other)
                   for other in self.robots.values())

    def clear_of_station(self, r, station):
        pose = self.points[station]
        return self.pose_ready(r) and math.hypot(r.pose[0] - pose[0], r.pose[1] - pose[1]) \
            >= self.station_clearance

    def station_area_clear(self, station):
        return all(self.clear_of_station(r, station) for r in self.robots.values())

    def arm(self, request, response):
        response.success = not self.shutting_down and all(
            self.robot_eligible(r)
            and r.state not in MOVING_STATES and r.state != 'STOPPING'
            and not r.goal_pending and r.goal_handle is None and r.stop_reason is None
            for r in self.robots.values()) \
            and self.pickup_area_clear(self.robots.get(self.pickup)) \
            and all(self.station_area_clear(station) if owner is None
                    else all(other.name == owner or self.clear_of_station(other, station)
                             for other in self.robots.values())
                    for station, owner in self.station_owners.items())
        if self.traffic.stale_occupants():
            response.success = False
        if response.success:
            self.armed = True
        response.message = 'FMS armed' if response.success else 'Nav2, localization, goals or shared areas not clear'
        return response

    def disarm(self, request, response):
        self.armed = False
        for r in self.robots.values():
            if r.state in MOVING_STATES:
                self.stop_goal(r, 'FMS disarmed',
                               lambda r=r: self.fail(r, 'FMS disarmed; inspect robot and shared areas'))
        response.success = True
        response.message = 'FMS disarmed; cancel requested for active Nav2 goals'
        return response

    def home_slot_pose(self, slot):
        """Parking slot i of the shared 수납대: slot 0 = HOME, slot 1 = home_slot_offset to its left."""
        if self.measured_home_slots is not None:
            return self.measured_home_slots[slot]
        x, y, yaw = self.points[self.home]
        if slot == 0 or self.home_offset == 0:
            return (x, y, yaw)
        d = self.home_offset * slot
        return (x - d * math.sin(yaw), y + d * math.cos(yaw), yaw)

    def on_pose(self, r, msg):
        p = msg.pose.pose.position
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in (p.x, p.y)):
            return
        r.pose = (p.x, p.y)
        r.pose_seen = time.monotonic()
        self.traffic.update(r.name, r.pose, r.pose_seen)

    def on_battery(self, r, msg):
        value = battery_fraction(msg)
        if value is None:
            self.get_logger().warn(f'[{r.name}] invalid BatteryState.percentage; data ignored')
            return
        r.battery_percent = value
        r.battery_seen = time.monotonic()

    def euclidean_cost(self, r, location='STORE'):
        return euclidean_distance(r.pose, self.points.get(location))

    def distance_cost(self, r, location='STORE'):
        now = time.monotonic()
        if r.path_target == location and r.path_distance is not None \
                and r.path_distance_seen is not None \
                and now - r.path_distance_seen <= self.path_cost_timeout \
                and euclidean_distance(r.pose, r.path_origin) is not None \
                and euclidean_distance(r.pose, r.path_origin) <= 0.25:
            return r.path_distance, 'nav2_path'
        fallback = self.euclidean_cost(r, location)
        if self.dry_run or not r.planner.server_is_ready():
            return fallback, 'euclidean'
        if not r.path_pending:
            self.request_path_cost(r, location)
        if self.path_query_wait > 0 and r.path_pending and r.path_request_started is not None \
                and now - r.path_request_started < self.path_query_wait:
            return None, 'nav2_pending'
        return fallback, 'euclidean'

    def request_path_cost(self, r, location):
        r.path_seq += 1
        seq = r.path_seq
        r.path_pending = True
        r.path_request_started = time.monotonic()
        r.path_target = location
        r.path_origin = tuple(r.pose) if r.pose is not None else None
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self.frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.points[location]
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.z = math.sin(yaw / 2)
        goal.goal.pose.orientation.w = math.cos(yaw / 2)
        try:
            r.planner.send_goal_async(goal).add_done_callback(
                lambda fut, r=r, seq=seq: self.path_goal_response(r, fut, seq))
        except Exception as exc:  # noqa: BLE001
            r.path_pending = False
            self.get_logger().warn(f'[{r.name}] path query failed; Euclidean fallback: {exc}')

    def path_goal_response(self, r, future, seq):
        if seq != r.path_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            r.path_pending = False
            self.get_logger().warn(f'[{r.name}] path goal unavailable; Euclidean fallback: {exc}')
            return
        if not handle.accepted:
            r.path_pending = False
            return
        handle.get_result_async().add_done_callback(
            lambda fut, r=r, seq=seq: self.path_result(r, fut, seq))

    def path_result(self, r, future, seq):
        if seq != r.path_seq:
            return
        r.path_pending = False
        try:
            wrapped = future.result()
            result = getattr(wrapped, 'result', wrapped)
            distance = path_distance(result.path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[{r.name}] invalid Nav2 path; Euclidean fallback: {exc}')
            return
        if distance is not None:
            r.path_distance = distance
            r.path_distance_seen = time.monotonic()
            r.path_source = 'nav2_path'

    # ---------------------------------------------------- allocation inputs
    def alloc_ctx(self):
        return {'speed': self.avg_speed, 't_loading': self.t_loading, 't_unloading': self.t_unloading,
                'detour': self.detour, 'pickup': self.pickup, 'home': self.home,
                'docking': self.docking, 'w_dist': self.cost_distance_weight,
                'w_wait': self.cost_load_weight, 'w_battery': self.cost_battery_weight}

    def can_take_task(self, r):
        return (r.state == 'IDLE' and r.current is None and self.robot_eligible(r)
                and not r.goal_pending and r.goal_handle is None
                and r.stop_reason is None)

    def robot_info(self, r, now):
        """What the allocator sees for one robot (every robot, busy ones too)."""
        free = self.can_take_task(r)
        distance, source = (None, None)
        if free:
            distance, source = self.distance_cost(r, 'STORE')
            free = distance is not None
        if distance is not None:
            r.distance_to_store, r.path_source = distance, source
        battery = r.battery_percent * 100.0 if self.battery_ready(r) else None
        return {'state': r.state, 'pose': r.pose, 'battery': battery,
                'battery_percent': r.battery_percent if self.battery_ready(r) else None,
                'battery_ready': self.battery_ready(r), 'current': r.current,
                'target': r.target, 'free': free, 'eligible': self.robot_eligible(r),
                'route_m': distance, 'distance_to_store': distance,
                'distance_source': source, 'elapsed': round(now - r.state_since, 2)}

    def assignable(self):
        """Queued requests whose station is not locked and is clear of robots."""
        return [task for task in self.queue if self.station_owners[task['station']] is None
                and self.station_area_clear(task['station'])]

    def take_task(self, r, task):
        """Give a queued request to r: lock its station and head for the pickup."""
        self.queue.remove(task)
        task = dict(task, robot=r.name)
        self.station_owners[task['station']] = r.name
        self.station_release_pending.discard(task['station'])
        r.current = task
        r.go_home_when_free = False
        r.home_slot = None
        self.log(r, f'Assigned {task}')
        self.go_pickup(r)

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
                'move_state': r.move_state,
                'error': r.error,
                'nav_ready': self.nav_ready(r),
                'pose_ready': self.pose_ready(r),
                'online': self.robot_online(r),
                'battery_ready': self.battery_ready(r),
                'battery_percent': None if r.battery_percent is None else round(r.battery_percent, 3),
                'low_battery': self.battery_ready(r) and r.battery_percent < self.low_battery,
                'distance_to_store': r.distance_to_store,
                'distance_source': r.path_source,
                'allocation_cost': r.allocation_cost,
                'traffic_direction': r.traffic_direction,
                'goal_pending': r.goal_pending,
                'timeout_left': None if self.dry_run else left,
                'distance_remaining': r.distance_remaining if r.state in MOVING_STATES else None,
                'home': self.home,
            }
        status = {
            'robots': robots,
            'queue': list(self.queue),
            'pickup': self.pickup,
            'pickup_clear': self.pickup is None and self.pickup_area_clear(None),
            'stations': self.station_owners,
            'traffic': self.traffic.status(),
            'armed': self.armed,
            'dry_run': self.dry_run,
            'points': {k: [round(c, 3) for c in v] for k, v in self.points.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # -------------------------------------------------------------------- loop
    def tick(self):
        now = time.monotonic()
        for r in self.robots.values():
            self.traffic.update(r.name, r.pose, r.pose_seen)
            if r.path_pending and r.path_request_started is not None \
                    and now - r.path_request_started > self.path_cost_timeout:
                r.path_pending = False
            if r.state in MOVING_STATES and not self.nav_ready(r):
                r.nav_missing_since = r.nav_missing_since or now
                if now - r.nav_missing_since > self.offline_timeout:
                    self.stop_goal(r, 'Nav2 server offline during navigation',
                                   lambda r=r: self.fail(r, 'Nav2 server offline during navigation'))
            else:
                r.nav_missing_since = None
            if r.state in MOVING_STATES and self.battery_ready(r) \
                    and r.battery_percent < self.critical_battery:
                self.stop_goal(r, 'Critical battery during navigation',
                               lambda r=r: self.fail(r, 'Critical battery; operator recovery required'))
            if r.state in MOVING_STATES and self.battery_required and not self.battery_ready(r):
                self.stop_goal(r, 'Battery telemetry stale during navigation',
                               lambda r=r: self.fail(
                                   r, 'Battery telemetry stale; operator recovery required'))
            if r.stop_reason is not None and r.stop_started is not None \
                    and now - r.stop_started > self.cancel_timeout and r.state == 'STOPPING':
                r.state = 'ERROR'
                r.error = 'Nav2 cancellation unconfirmed; keep robot and pickup locked'
                self.get_logger().error(f'[{r.name}] {r.error}')
            if r.state in MOVING_STATES and not self.dry_run and not self.pose_ready(r):
                self.stop_goal(r, 'Localization stale during navigation',
                               lambda r=r: self.fail(r, 'Localization stale during navigation'))
            if r.nav_deadline is not None and now >= r.nav_deadline and r.state in MOVING_STATES:
                r.nav_deadline = None
                if self.dry_run:
                    self.arrived(r)
                else:
                    reason = f'Navigation timeout ({self.nav_timeout:.0f}s) to {r.target}'
                    self.stop_goal(r, reason, lambda r=r, reason=reason: self.fail(r, reason))
            if r.dock_deadline is not None and now >= r.dock_deadline and r.state == 'DOCKING':
                r.dock_deadline = None
                if self.dry_run:
                    self.docked(r)
                else:
                    self.fail(r, f'Docking timeout ({self.dock_timeout:.0f}s)')

        for station, owner in self.station_owners.items():
            if owner is not None and station in self.station_release_pending \
                    and self.clear_of_station(self.robots[owner], station) \
                    and self.robots[owner].stop_reason is None \
                    and self.robots[owner].departure_started_at is not None \
                    and self.robots[owner].pose_seen > self.robots[owner].departure_started_at:
                self.station_owners[station] = None
                self.station_release_pending.discard(station)

        if not self.armed or self.shutting_down:
            self.publish_status()
            return

        # A robot waiting at a safe endpoint retries its corridor reservation.
        for r in self.robots.values():
            if r.state != 'WAIT_TRAFFIC' or r.pending_move is None:
                continue
            if self.traffic.waiting_timed_out(r.name):
                self.traffic.cancel_wait(r.name)
                self.fail(r, 'Traffic wait timeout; inspect corridor and reservations')
                continue
            location, state, pose = r.pending_move
            self.schedule_move(r, location, state, pose)

        # 1) hand new requests to free robots. The existing allocator keeps its
        # wait-time/continuous-task model; route_m and battery now come from live
        # Nav2/BatteryState data instead of placeholders.
        infos = {n: self.robot_info(r, now) for n, r in self.robots.items()}
        free = {n for n, i in infos.items() if i['free']}
        available = self.assignable()
        costs = task_allocation.allocation_costs(infos, self.points, self.alloc_ctx())
        for name, value in costs.items():
            if name in self.robots:
                self.robots[name].allocation_cost = value
        if available and free:
            for name, task in task_allocation.assign(infos, available, self.points, self.alloc_ctx()):
                if name not in free or task not in self.queue \
                        or self.station_owners[task['station']] is not None:
                    continue
                free.discard(name)
                self.take_task(self.robots[name], task)
        elif self.queue and now - self.last_wait_log > 5.0 and not free:
            self.last_wait_log = now
            if not any(self.nav_ready(r) for r in self.robots.values()):
                self.get_logger().warn('Waiting for Nav2 action servers...')

        # 2) pickup free -> let the next waiting robot in (task_allocation.pickup_order)
        if self.pickup is None:
            waiting = {n: {'pose': r.pose, 'since': r.wait_since, 'task': r.current}
                       for n, r in self.robots.items() if r.state == 'WAIT_PICKUP'
                       and self.robot_eligible(r)}
            name = task_allocation.pickup_order(waiting, self.points)
            if name is not None:
                self.go_pickup(self.robots[name])

        # 3) Leave a completed station even when another request to that same
        # station is queued; waiting there would deadlock its occupancy lock.
        for r in self.robots.values():
            assignable = any(self.station_owners[task['station']] is None
                             and self.station_area_clear(task['station']) for task in self.queue)
            if r.state == 'IDLE' and r.go_home_when_free and not assignable \
                    and self.robot_eligible(r):
                r.go_home_when_free = False
                self.go_home(r)

        holder = self.robots.get(self.pickup)
        clearance_since = None if holder is None else (
            holder.departure_started_at if holder.departure_started_at is not None
            else holder.goal_started_at if holder.state == 'ERROR' else None)
        if holder is not None and holder.state in ('TO_STATION', 'WAIT_RECEIVE', 'TO_HOME', 'IDLE', 'ERROR') \
                and holder.stop_reason is None and not holder.goal_pending \
                and self.clear_of_pickup(holder) and clearance_since is not None \
                and holder.pose_seen > clearance_since:
            self.release_pickup(holder)

        self.publish_status()

    # ---------------------------------------------------------- shared places
    def go_pickup(self, r):
        """Drive to the pickup if it is free (and take it), else wait where it is."""
        if self.pickup in (None, r.name) and self.pose_ready(r) \
                and self.pickup_area_clear(r) and self.nav_ready(r):
            self.pickup = r.name
            r.wait_since = None
            self.schedule_move(r, 'STORE', 'TO_STORE')
        else:
            r.target = None
            r.distance_remaining = None
            if r.wait_since is None:
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
        self.schedule_move(r, self.home, 'TO_HOME', pose=self.home_slot_pose(slot))

    def schedule_move(self, r, location, state, pose=None):
        direction = 'entry' if state == 'TO_STORE' else 'exit'
        allowed, reason = self.traffic.request(r.name, direction, r.pose, r.pose_seen)
        if allowed:
            r.pending_move = None
            r.traffic_direction = direction if self.traffic.enabled else None
            # Enter/leave the shared gate through a measured waypoint. Other
            # trips keep the conservative reservation without a needless detour.
            waypoint = self.traffic.waypoint(direction) if (
                state == 'TO_STORE' or r.last_location == 'STORE') else None
            r.traffic_waypoint_pending = waypoint is not None
            self.move(r, location, state, waypoint if waypoint is not None else pose)
            return True
        r.pending_move = (location, state, pose)
        r.target = location
        r.move_state = state
        r.wait_since = r.wait_since or time.monotonic()
        r.error = f'Traffic waiting: {reason}'
        self.set_state(r, 'WAIT_TRAFFIC')
        return False

    # -------------------------------------------------------------- navigation
    def move(self, r, location, state, pose=None):
        if r.goal_pending or r.goal_handle is not None or r.stop_reason is not None:
            raise RuntimeError(f'{r.name}: previous Nav2 goal is not resolved')
        r.goal_seq += 1
        seq = r.goal_seq
        r.target = location
        r.move_state = state
        r.goal_handle = None
        r.error = None
        r.distance_remaining = None
        r.goal_pose = pose or self.points[location]
        r.goal_started_at = time.monotonic()
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
        r.goal_pending = True
        try:
            r.nav.send_goal_async(
                goal, feedback_callback=lambda fb: self.nav_feedback(r, fb, seq)
            ).add_done_callback(lambda fut: self.goal_response(r, fut, seq))
        except Exception as exc:  # noqa: BLE001
            r.goal_pending = False
            self.mark_uncertain(r, f'Nav2 send_goal failed: {exc}')

    def nav_feedback(self, r, msg, seq):
        if seq == r.goal_seq and r.stop_reason is None:
            r.distance_remaining = round(float(msg.feedback.distance_remaining), 2)

    def goal_response(self, r, future, seq):
        if seq != r.goal_seq:
            return  # superseded goal
        r.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.mark_uncertain(r, f'Nav2 goal acceptance unconfirmed: {exc}')
            return
        if not handle.accepted:
            if r.stop_reason is not None:
                self.finish_stop(r)
            else:
                self.fail(r, f'Nav2 rejected goal to {r.target}')
            return
        r.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut: self.nav_result(r, fut, seq))
        if r.stop_reason is not None:
            self.send_cancel(r)

    def nav_result(self, r, future, seq):
        if seq != r.goal_seq:
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.mark_uncertain(r, f'Nav2 result unconfirmed: {exc}')
            return
        r.goal_handle = None
        if r.stop_reason is not None:
            self.finish_stop(r)
            return
        if r.state not in MOVING_STATES:
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.arrived(r)
        else:
            self.fail(r, f'Navigation to {r.target} ended: status={result.status}')

    def mark_uncertain(self, r, message):
        r.error = f'{message}; operator must verify stop'
        r.state = 'ERROR'
        r.nav_deadline = None
        r.stop_reason = 'Nav2 motion unconfirmed'
        self.get_logger().error(f'[{r.name}] {r.error}')

    def stop_goal(self, r, reason, on_stopped):
        """Do not send another goal until Nav2 resolves this one."""
        if r.stop_reason is not None:
            return
        r.nav_deadline = None
        if self.dry_run or (not r.goal_pending and r.goal_handle is None):
            on_stopped()
            return
        r.stop_reason = reason
        r.stop_callback = on_stopped
        r.stop_started = time.monotonic()
        self.set_state(r, 'STOPPING')
        self.send_cancel(r)

    def send_cancel(self, r):
        if r.goal_handle is not None:
            self.log(r, 'Cancelling active Nav2 goal')
            try:
                r.goal_handle.cancel_goal_async().add_done_callback(
                    lambda fut, r=r: self.cancel_response(r, fut))
            except Exception as exc:  # noqa: BLE001
                r.error = f'Nav2 cancel request failed: {exc}; stop unconfirmed'
                self.get_logger().error(f'[{r.name}] {r.error}')

    def cancel_response(self, r, future):
        if r.stop_reason is None:
            return
        try:
            response = future.result()
            if not response.goals_canceling:
                r.error = 'Nav2 rejected cancellation; waiting for goal result'
        except Exception as exc:  # noqa: BLE001
            r.error = f'Nav2 cancellation response failed: {exc}'

    def finish_stop(self, r):
        callback = r.stop_callback
        r.stop_reason = None
        r.stop_callback = None
        r.stop_started = None
        r.goal_handle = None
        r.goal_pending = False
        if callback is not None:
            callback()

    def arrived(self, r):
        r.nav_deadline = None
        if self.dry_run and r.goal_pose is not None:
            r.pose = (r.goal_pose[0], r.goal_pose[1])      # real runs: pose from amcl_pose
            r.pose_seen = time.monotonic()
        if r.traffic_waypoint_pending:
            # Keep the reservation between the gate waypoint and destination.
            r.traffic_waypoint_pending = False
            self.move(r, r.target, r.move_state)
            return
        r.last_location = r.target
        self.traffic.release(r.name)
        r.traffic_direction = None
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
        response.success = self.armed and r.state == 'WAIT_LOAD' \
            and self.robot_eligible(r)
        response.message = f'{r.name}: loading confirmed' if response.success \
            else f'{r.name}: FMS disarmed, Nav2/localization unavailable, or not waiting for loading (state={r.state})'
        if response.success:
            r.departure_started_at = time.monotonic()
            self.schedule_move(r, r.current['station'], 'TO_STATION')
        return response

    def received(self, r, response):
        response.success = r.state == 'WAIT_RECEIVE'
        response.message = f'{r.name}: delivery completed' if response.success \
            else f'{r.name} is not waiting for receipt (state={r.state})'
        if response.success:
            self.log(r, f'COMPLETED: {r.current}')
            self.station_release_pending.add(r.current['station'])
            r.departure_started_at = time.monotonic()
            r.current = None
            r.target = None
            # Allocator.next_for: take the next assignable request now, straight to the pickup
            nxt = None
            available = self.assignable() if self.armed and not self.shutting_down \
                and self.nav_ready(r) and self.pose_ready(r) and not r.goal_pending \
                and r.goal_handle is None and r.stop_reason is None else []
            if available:
                nxt = task_allocation.next_for(r.name, available, self.points, self.alloc_ctx())
            if nxt is not None and nxt in available:
                self.log(r, 'Next request right after delivery (no trip home)')
                self.take_task(r, nxt)
            else:
                r.go_home_when_free = True      # next tick: new task, or back to the 수납대
                self.set_state(r, 'IDLE')
        return response

    def retry(self, r, response):
        if not self.armed:
            response.success = False
            response.message = f'{r.name}: FMS disarmed'
            return response
        if r.state != 'ERROR' or r.move_state is None:
            response.success = False
            response.message = f'{r.name}: nothing to retry (state={r.state})'
            return response
        if r.stop_reason is not None or r.goal_pending or r.goal_handle is not None:
            response.success = False
            response.message = f'{r.name}: previous Nav2 goal has not stopped'
            return response
        if not self.robot_eligible(r):
            response.success = False
            response.message = f'{r.name}: Nav2, localization, or battery not ready'
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
            self.schedule_move(r, r.target, step)
        return response

    def reset(self, r, response):
        if r.stop_reason is not None:
            response.success = False
            response.message = f'{r.name}: cancellation already in progress'
            return response
        dropped = r.current
        self.stop_goal(r, 'Operator reset', lambda r=r, dropped=dropped:
                       self.finish_reset(r, dropped))
        response.success = True
        response.message = f'{r.name}: reset requested; Nav2 stop must be confirmed'
        return response

    def finish_reset(self, r, dropped):
        if self.pickup == r.name and self.clear_of_pickup(r) \
                and r.goal_started_at is not None and r.pose_seen > r.goal_started_at:
            self.release_pickup(r)
        if dropped is not None and self.station_owners.get(dropped['station']) == r.name:
            self.station_release_pending.add(dropped['station'])
            r.departure_started_at = time.monotonic()
        r.current = None
        r.target = None
        r.move_state = None
        r.error = None
        r.dock_deadline = None
        r.wait_since = None
        r.go_home_when_free = False      # do not initiate another trip after a reset
        r.pending_move = None
        r.traffic_waypoint_pending = False
        self.traffic.cancel_wait(r.name)
        if r.name in self.traffic.occupants:
            r.error = 'Traffic lock retained after cancellation; disarm and clear after physical inspection'
            self.set_state(r, 'ERROR')
        else:
            self.set_state(r, 'IDLE')
        self.get_logger().warn(f'[{r.name}] reset completed. Dropped task: {dropped}')

    def traffic_clear(self, r, response):
        safe = not self.armed and r.goal_handle is None and not r.goal_pending \
            and r.state not in MOVING_STATES and r.state != 'STOPPING'
        response.success = safe
        if safe:
            self.traffic.release(r.name)
            r.traffic_direction = None
            if r.error and 'Traffic lock retained' in r.error:
                r.error = None
                r.stop_reason = None
                self.set_state(r, 'IDLE')
            response.message = f'{r.name}: traffic reservation cleared after operator confirmation'
        else:
            response.message = f'{r.name}: disarm and confirm Nav2 stopped before clearing traffic'
        return response

    def fail(self, r, message):
        r.nav_deadline = None
        r.dock_deadline = None
        r.error = message
        if self.pickup == r.name and self.clear_of_pickup(r) and r.stop_reason is None \
                and r.goal_started_at is not None and r.pose_seen > r.goal_started_at:
            self.release_pickup(r)
        self.get_logger().error(f'[{r.name}] {message}')
        self.set_state(r, 'ERROR')

    def safe_shutdown(self):
        """Cancel tracked goals before ending the process; report unresolved stops."""
        self.armed = False
        self.shutting_down = True
        for r in self.robots.values():
            if r.state in MOVING_STATES:
                self.stop_goal(r, 'Controller shutdown',
                               lambda r=r: self.fail(r, 'Controller shutdown'))
        deadline = time.monotonic() + self.cancel_timeout
        while time.monotonic() < deadline and any(
                r.goal_pending or r.goal_handle is not None for r in self.robots.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
        for r in self.robots.values():
            if r.goal_pending or r.goal_handle is not None or r.stop_reason is not None:
                self.get_logger().error(
                    f'[{r.name}] Nav2 stop unconfirmed at shutdown; use robot stop control')


def main(args=None):
    # Keep the ROS context alive after SIGINT/SIGTERM so Nav2 cancellation can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    node = FmsController()
    if not node.dry_run:
        node.get_logger().warn('REAL MODE: measured configuration and /delivery/arm are required. '
                               'For simulation without motion: python3 fms_sim.py')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
