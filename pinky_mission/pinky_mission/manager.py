"""ROS bridge for RViz clicks, mission commands, Nav2 and status."""
import json
import math
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray

from .logic import DELIVERY_ROUTES, route, heading, clear_in_grid, point_name


class MissionManager(Node):
    def __init__(self):
        super().__init__('pinky_mission_manager')
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('max_points', 4)
        self.declare_parameter('footprint_radius', 0.14)
        self.declare_parameter('costmap_max_age', 3.0)
        self.declare_parameter('scan_max_age', 3.0)
        self.declare_parameter('pose_max_age', 2.0)
        self.declare_parameter('goal_timeout', 180.0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('photo_dir', str(Path.home() / 'Pictures' / 'pinky_mission'))
        self.declare_parameter('mission_file', str(Path.home() / '.config' / 'pinky_mission' / 'mission.json'))
        self.points = []
        self.kind = 'delivery_1'
        self.cycles = 1
        self.continuous = False
        self.wait_seconds = 0.0
        self.state = 'idle'
        self.error = ''
        self.sequence = []
        self.step = 0
        self.goal_handle = None
        self.goal_pending = False
        self.cancel_pending = False
        self.cancel_started = 0.0
        self.goal_started = 0.0
        self.wait_until = 0.0
        self.photo_until = 0.0
        self.camera_seen = False
        self.pose = None
        self.pose_seen = 0.0
        self.scan_seen = 0.0
        self.costmap = None
        self.costmap_seen = 0.0
        self.map_seen = False
        self.nav_feedback = ''
        self.last_status = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/pinky_mission/markers', latched)
        self.status_pub = self.create_publisher(String, '/pinky_mission/status', latched)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        self.create_subscription(String, '/pinky_mission/command', self.on_command, 10)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self.on_costmap, 10)
        self.create_subscription(Costmap, '/global_costmap/costmap', self.on_costmap, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image, self.get_parameter('camera_topic').value,
                                 self.on_image, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.2, self.tick)
        self.load(silent=True)
        self.publish_markers()
        self.get_logger().warn('Motion gate is %s' % ('OPEN' if self.get_parameter('allow_motion').value else 'CLOSED'))

    def on_map(self, msg):
        self.map_seen = True

    def on_scan(self, msg):
        self.scan_seen = time.monotonic()

    def on_costmap(self, msg):
        if not msg.data:
            return
        m = msg.metadata
        q = m.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.costmap = dict(width=m.size_x, height=m.size_y, resolution=m.resolution,
                            origin_x=m.origin.position.x, origin_y=m.origin.position.y,
                            origin_yaw=yaw, data=msg.data)
        self.costmap_seen = time.monotonic()

    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p = transform.transform.translation
            q = transform.transform.rotation
            self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z)))
            self.pose_seen = time.monotonic()
        except TransformException:
            pass

    def on_click(self, msg):
        if self.state in ('running', 'waiting', 'photo', 'paused', 'canceling') or self.goal_pending:
            self.error = '任务运行期间不能修改目标点'
            return
        if len(self.points) >= self.get_parameter('max_points').value:
            self.error = '目标点已满，请删除或修改现有点'
            return
        try:
            if msg.header.frame_id != 'map':
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
                msg = do_transform_point(msg, transform)
            if not all(math.isfinite(v) for v in (msg.point.x, msg.point.y)):
                raise ValueError('non-finite point')
        except (TransformException, ValueError) as exc:
            self.error = '坐标转换失败: ' + str(exc)
            return
        self.points.append(dict(x=msg.point.x, y=msg.point.y, yaw=None))
        self.error = ''
        self.publish_markers()
        self.publish_status()

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
            op = cmd['op']
            if op in ('undo', 'clear_points', 'set_waypoint', 'load', 'clear_task'):
                if self.state in ('running', 'waiting', 'photo', 'paused', 'canceling') or self.goal_pending:
                    raise ValueError('运行中不能修改目标点或任务')
                if op == 'undo' and self.points:
                    self.points.pop()
                elif op == 'clear_points':
                    self.points.clear()
                elif op == 'set_waypoint':
                    index = int(cmd['index'])
                    if index < 0 or index >= len(self.points):
                        raise ValueError('无效目标点编号')
                    p = self.points[index]
                    for key in ('x', 'y', 'yaw'):
                        if key in cmd:
                            value = cmd[key]
                            if value is not None and not math.isfinite(float(value)):
                                raise ValueError('目标点数值无效')
                            p[key] = None if value is None else float(value)
                elif op == 'load':
                    self.load()
                elif op == 'clear_task':
                    self.sequence = []
                    self.step = 0
                    self.state = 'idle'
                self.publish_markers()
            elif op == 'configure':
                if self.state not in ('idle', 'completed', 'failed', 'canceled'):
                    raise ValueError('任务运行中不能改参数')
                kind = cmd.get('kind', self.kind)
                cycles = int(cmd.get('cycles', self.cycles))
                wait = float(cmd.get('wait_seconds', self.wait_seconds))
                valid_kinds = tuple(DELIVERY_ROUTES) + ('sequence', 'patrol', 'roundtrip', 'wait', 'photo')
                if kind not in valid_kinds or cycles < 1 or wait < 0 or not math.isfinite(wait):
                    raise ValueError('任务参数无效')
                self.kind, self.cycles, self.wait_seconds = kind, cycles, wait
                self.continuous = bool(cmd.get('continuous', self.continuous))
            elif op == 'save':
                self.save()
            elif op == 'start':
                self.start()
            elif op == 'pause':
                self.pause()
            elif op == 'resume':
                self.resume()
            elif op in ('cancel', 'emergency_stop'):
                self.cancel()
            else:
                raise ValueError('未知命令: ' + str(op))
            if op not in ('start', 'pause', 'resume', 'cancel', 'emergency_stop'):
                self.error = ''
        except (ValueError, KeyError, TypeError, OSError, json.JSONDecodeError) as exc:
            self.error = str(exc)
            self.get_logger().warn(self.error)
        self.publish_status()

    def preflight(self):
        now = time.monotonic()
        problems = []
        if not self.get_parameter('allow_motion').value:
            problems.append('运动闸门关闭 (allow_motion=false)')
        if not self.map_seen or self.count_publishers('/map') == 0:
            problems.append('没有 /map')
        if now - self.pose_seen > self.get_parameter('pose_max_age').value:
            problems.append('map→base_footprint TF 不可用')
        if now - self.scan_seen > self.get_parameter('scan_max_age').value:
            problems.append('/scan 超时')
        if not self.costmap or now - self.costmap_seen > self.get_parameter('costmap_max_age').value:
            problems.append('全局 Costmap 不可用或超时')
        if not self.nav.server_is_ready():
            problems.append('Nav2 /navigate_to_pose 不可用')
        if len(self.points) != self.get_parameter('max_points').value:
            problems.append('目标点数量应为 %d' % self.get_parameter('max_points').value)
        if self.costmap and not problems:
            radius = self.get_parameter('footprint_radius').value
            for i, p in enumerate(self.points):
                if not clear_in_grid(p['x'], p['y'], self.costmap, radius):
                    problems.append('%s 不在已知可通行 Costmap 区域' % point_name(i))
        return problems

    def start(self):
        if self.state in ('running', 'waiting', 'photo', 'paused', 'canceling') or self.goal_pending:
            raise ValueError('已有任务运行')
        problems = self.preflight()
        if problems:
            raise ValueError('预检查失败: ' + '; '.join(problems))
        self.sequence = route(self.kind, len(self.points), self.cycles)
        self.step = 0
        self.state = 'running'
        self.error = ''
        self.send_next()

    def send_next(self):
        if self.state != 'running' or self.goal_pending or self.goal_handle:
            return
        if self.step >= len(self.sequence):
            if self.kind == 'patrol' and self.continuous:
                self.step = 0
            else:
                self.state = 'completed'
                self.publish_status()
                return
        problems = self.preflight()
        if problems:
            self.fail('继续导航预检查失败: ' + '; '.join(problems))
            return
        index = self.sequence[self.step]
        p = self.points[index]
        next_index = self.sequence[self.step + 1] if self.step + 1 < len(self.sequence) else None
        yaw = heading(self.points, index, self.pose[:2] if self.pose else None, next_index)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = p['x'], p['y']
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_pending = True
        self.goal_started = time.monotonic()
        future = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        self.publish_status()

    def on_goal_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.fail('Nav2 拒绝目标')
                return
            self.goal_handle = handle
            result = handle.get_result_async()
            result.add_done_callback(self.on_result)
            if self.state in ('canceling', 'paused'):
                self.request_cancel()
        except Exception as exc:
            self.fail('发送目标失败: ' + str(exc))

    def on_feedback(self, msg):
        feedback = msg.feedback
        self.nav_feedback = '剩余距离 %.2f m' % feedback.distance_remaining

    def on_result(self, future):
        self.goal_handle = None
        self.cancel_pending = False
        try:
            status = future.result().status
        except Exception as exc:
            self.fail('获取导航结果失败: ' + str(exc))
            return
        if self.state == 'canceling':
            self.state = 'canceled'
        elif self.state in ('paused', 'failed'):
            pass
        elif status == GoalStatus.STATUS_SUCCEEDED:
            if self.kind == 'photo':
                self.state = 'photo'
                self.camera_seen = False
                self.photo_until = time.monotonic() + 5.0
            elif self.kind == 'wait' and self.wait_seconds:
                self.state = 'waiting'
                self.wait_until = time.monotonic() + self.wait_seconds
            else:
                self.step += 1
                self.send_next()
        else:
            self.fail('Nav2 导航失败，状态码 %d' % status)
        self.publish_status()

    def request_cancel(self):
        if self.goal_handle and not self.cancel_pending:
            self.cancel_pending = True
            self.cancel_started = time.monotonic()
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.on_cancel_response)

    def on_cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self.error = 'Nav2 未确认取消请求；请使用实体急停'
        except Exception as exc:
            self.error = '取消请求失败: %s；请使用实体急停' % exc
        self.publish_status()

    def cancel(self):
        if self.state not in ('running', 'waiting', 'photo', 'paused', 'canceling'):
            return
        self.state = 'canceling'
        if self.goal_handle:
            self.request_cancel()
        elif not self.goal_pending:
            self.state = 'canceled'
        self.publish_status()

    def pause(self):
        if self.state != 'running':
            raise ValueError('只有导航中可暂停')
        self.state = 'paused'
        if self.goal_handle:
            self.request_cancel()
        self.publish_status()

    def resume(self):
        if self.state != 'paused' or self.goal_pending or self.goal_handle:
            raise ValueError('等待 Nav2 取消完成后才能继续')
        problems = self.preflight()
        if problems:
            raise ValueError('继续前预检查失败: ' + '; '.join(problems))
        self.state = 'running'
        self.send_next()

    def fail(self, reason):
        self.error = reason
        self.state = 'failed'
        self.get_logger().error(reason)
        if self.goal_handle:
            self.request_cancel()
        self.publish_status()

    def on_image(self, msg):
        if self.state != 'photo':
            return
        try:
            import cv2
            from cv_bridge import CvBridge
            folder = Path(self.get_parameter('photo_dir').value).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            label = chr(65 + self.sequence[self.step])
            name = folder / ('%s_%s.jpg' % (datetime.now().strftime('%Y%m%d_%H%M%S_%f'), label))
            image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not cv2.imwrite(str(name), image):
                raise OSError('cv2.imwrite returned false')
            self.camera_seen = True
            self.get_logger().info('照片保存: ' + str(name))
            self.step += 1
            self.state = 'running'
            self.send_next()
        except Exception as exc:
            self.error = '拍照失败: ' + str(exc)
            self.state = 'failed'
        self.publish_status()

    def tick(self):
        self.update_pose()
        now = time.monotonic()
        if self.state == 'running' and self.goal_handle and now - self.goal_started > self.get_parameter('goal_timeout').value:
            self.fail('导航超时，已请求取消')
        if self.state == 'running' and self.goal_handle:
            if (now - self.pose_seen > self.get_parameter('pose_max_age').value or
                    now - self.scan_seen > self.get_parameter('scan_max_age').value or
                    now - self.costmap_seen > self.get_parameter('costmap_max_age').value or
                    not self.nav.server_is_ready()):
                self.fail('导航状态、定位、雷达或 Costmap 丢失，已请求取消；请确认底盘停止')
        if self.cancel_pending and now - self.cancel_started > 3.0 and self.goal_handle:
            self.error = '3 秒内未确认 Nav2 取消结果；请使用实体急停并确认底盘停止'
        if self.state == 'waiting' and now >= self.wait_until:
            self.step += 1
            self.state = 'running'
            self.send_next()
        if self.state == 'photo' and now >= self.photo_until:
            self.fail('摄像头不可用：5 秒内未收到可保存图像')
        self.publish_status()

    def publish_status(self):
        payload = dict(state=self.state, error=self.error, points=self.points,
                       kind=self.kind, cycles=self.cycles, continuous=self.continuous,
                       wait_seconds=self.wait_seconds, step=self.step,
                       total=len(self.sequence),
                       target=(point_name(self.sequence[self.step]) if self.step < len(self.sequence) else ''),
                       pose=self.pose, feedback=self.nav_feedback,
                       map=(self.map_seen and self.count_publishers('/map') > 0),
                       scan=(time.monotonic() - self.scan_seen < 3),
                       costmap=(time.monotonic() - self.costmap_seen < 3),
                       nav2=self.nav.server_is_ready(),
                       motion_enabled=self.get_parameter('allow_motion').value)
        data = json.dumps(payload, ensure_ascii=False)
        if data != self.last_status:
            self.status_pub.publish(String(data=data))
            self.last_status = data

    def publish_markers(self):
        markers = MarkerArray()
        delete = Marker()
        delete.header.frame_id = 'map'
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        for i, p in enumerate(self.points):
            for part in ('point', 'label'):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns, m.id = part, i
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y = p['x'], p['y']
                m.pose.orientation.w = 1.0
                m.color.a = 1.0
                if part == 'point':
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 0.14
                    m.color.r, m.color.g, m.color.b = 1.0, 0.35, 0.1
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.text = point_name(i)
                    m.pose.position.z = 0.28
                    m.scale.z = 0.22
                    m.color.r = m.color.g = m.color.b = 1.0
                markers.markers.append(m)
        self.marker_pub.publish(markers)

    def save(self):
        path = Path(self.get_parameter('mission_file').value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = dict(version=1, points=self.points, kind=self.kind, cycles=self.cycles,
                    continuous=self.continuous, wait_seconds=self.wait_seconds)
        tmp = path.with_suffix(path.suffix + '.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(tmp, path)

    def load(self, silent=False):
        path = Path(self.get_parameter('mission_file').value).expanduser()
        if not path.exists():
            if silent:
                return
            raise ValueError('任务文件不存在: ' + str(path))
        data = json.loads(path.read_text(encoding='utf-8'))
        points = data['points']
        if not isinstance(points, list) or len(points) > self.get_parameter('max_points').value:
            raise ValueError('任务文件目标点数量无效')
        checked = []
        for p in points:
            x, y = float(p['x']), float(p['y'])
            yaw = None if p.get('yaw') is None else float(p['yaw'])
            if not all(math.isfinite(v) for v in (x, y)) or (yaw is not None and not math.isfinite(yaw)):
                raise ValueError('任务文件含非法坐标')
            checked.append(dict(x=x, y=y, yaw=yaw))
        kind = data.get('kind', 'sequence')
        cycles = int(data.get('cycles', 1))
        wait = float(data.get('wait_seconds', 0.0))
        valid_kinds = tuple(DELIVERY_ROUTES) + ('sequence', 'patrol', 'roundtrip', 'wait', 'photo')
        if kind not in valid_kinds or cycles < 1 or wait < 0 or not math.isfinite(wait):
            raise ValueError('任务文件参数无效')
        self.points, self.kind, self.cycles, self.wait_seconds = checked, kind, cycles, wait
        self.continuous = bool(data.get('continuous', False))
        self.publish_markers()


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.goal_handle or node.goal_pending:
            node.cancel()
            end = time.monotonic() + 3
            while node.goal_handle and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
