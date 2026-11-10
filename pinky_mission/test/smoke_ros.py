"""Isolated DDS smoke test; deliberately keeps allow_motion=false."""
import json
import os
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    with tempfile.TemporaryDirectory() as temp:
        proc = subprocess.Popen(['ros2', 'run', 'pinky_mission', 'mission_manager', '--ros-args',
                                 '-p', 'allow_motion:=false', '-p', 'mission_file:=' + temp + '/mission.json'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        rclpy.init()
        node = rclpy.create_node('pinky_mission_smoke')
        state = {}
        markers = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/pinky_mission/status',
                                 lambda msg: state.update(json.loads(msg.data)), qos)
        node.create_subscription(MarkerArray, '/pinky_mission/markers',
                                 lambda msg: markers.append(msg), qos)
        click = node.create_publisher(PointStamped, '/clicked_point', 10)
        command = node.create_publisher(String, '/pinky_mission/command', 10)

        def until(predicate, seconds=5):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if predicate():
                    return
            raise AssertionError('timeout: ' + repr(state))

        try:
            until(lambda: state.get('state') == 'idle')
            for i in range(4):
                p = PointStamped()
                p.header.frame_id = 'map'
                p.point.x = float(i)
                p.point.y = 1.0
                click.publish(p)
                until(lambda i=i: len(state.get('points', [])) == i + 1)
            until(lambda: markers and len(markers[-1].markers) >= 9)
            assert all(marker.ns != 'route' for marker in markers[-1].markers)
            command.publish(String(data=json.dumps({'op': 'start'})))
            until(lambda: '运动闸门关闭' in state.get('error', ''))
            assert state['state'] == 'idle'
            command.publish(String(data=json.dumps({'op': 'save'})))
            until(lambda: os.path.isfile(temp + '/mission.json'))
            command.publish(String(data=json.dumps({'op': 'undo'})))
            until(lambda: len(state.get('points', [])) == 3)
            command.publish(String(data=json.dumps({'op': 'load'})))
            until(lambda: len(state.get('points', [])) == 4)
            print('PASS: clicks, markers, motion gate, save, undo, load')
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
