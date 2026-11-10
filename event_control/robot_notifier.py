#!/usr/bin/env python3
"""Robot-side sound notifier (memo items 3 and 5). Run this ON THE ROBOT.

3) Song: when the controller enters a "waiting for a person" state
   (WAIT_LOAD at the pickup point, WAIT_RECEIVE at a workstation), play a
   song so people know to come. Repeats every `song_repeat` seconds while
   still waiting; stops as soon as the state changes.
5) Beep: while the robot is driving (TO_*), if the LiDAR sees something
   closer than `beep_distance` in the front sector, beep every
   `beep_interval` seconds. Muted in the last `near_goal_mute` metres of a
   trip (workstations/walls are close there) when Nav2 distance is known.
   LiDAR cannot tell people from objects: tune distance/sector on site.

Inputs : /delivery/status (std_msgs/String JSON), /scan (sensor_msgs/LaserScan)
         With two robots the status has a "robots" dict: this node follows robots[robot_name]
         (robot_name param, default = the ROS namespace without '/', else 'pinky_1').
Output : sound via an external player command (default `aplay -q`).
         Set song_file/beep_file to '' to only log (no sound).

Parameters (defaults):
  player 'aplay -q' | song_file <dir>/sounds/song.wav | beep_file <dir>/sounds/beep.wav
  song_repeat 15.0 s (0 = play once) | wait_states ['WAIT_LOAD','WAIT_RECEIVE']
  scan_topic '/scan' | beep_distance 0.45 m | sector_deg 50 (total width)
  front_offset_deg 0 (if LiDAR 0deg is not robot front) | min_valid 0.08 m
  beep_interval 1.0 s | near_goal_mute 0.5 m (0 = never mute)
  robot_name '' (auto: namespace, e.g. /pinky_2 -> pinky_2; else pinky_1)
  status_topic '/delivery/status' (absolute: one controller for all robots)
"""
import json
import math
import os
import shlex
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
MOVING = ('TO_STORE', 'TO_STATION', 'TO_HOME')


def front_min_distance(scan, sector_deg, offset_deg, min_valid):
    """Smallest valid range inside the front sector, or None."""
    half = math.radians(sector_deg) / 2.0
    offset = math.radians(offset_deg)
    lo = max(scan.range_min, min_valid)
    best = None
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < lo or (scan.range_max > 0 and r > scan.range_max):
            continue
        a = scan.angle_min + i * scan.angle_increment - offset
        a = math.atan2(math.sin(a), math.cos(a))  # wrap to [-pi, pi]
        if abs(a) <= half and (best is None or r < best):
            best = r
    return best


class Player:
    """Non-blocking sound player around an external command."""

    def __init__(self, node, player_cmd):
        self.node = node
        self.cmd = shlex.split(player_cmd) if player_cmd else []
        self.procs = {}
        self.broken = False

    def play(self, key, path):
        if not path:
            self.node.get_logger().info(f'[sound:{key}] (no file, log only)')
            return
        if self.broken:
            return
        proc = self.procs.get(key)
        if proc is not None and proc.poll() is None:
            return  # still playing
        try:
            self.procs[key] = subprocess.Popen(
                self.cmd + [path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, ValueError) as exc:
            self.broken = True
            self.node.get_logger().error(
                f'Cannot run player "{" ".join(self.cmd)}": {exc}. Sound disabled.')

    def stop(self, key):
        proc = self.procs.pop(key, None)
        if proc is not None and proc.poll() is None:
            proc.terminate()


class RobotNotifier(Node):
    def __init__(self):
        super().__init__('robot_notifier')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.song_file = p('song_file', os.path.join(HERE, 'sounds', 'song.wav'))
        self.beep_file = p('beep_file', os.path.join(HERE, 'sounds', 'beep.wav'))
        self.song_repeat = float(p('song_repeat', 15.0))
        self.wait_states = list(p('wait_states', ['WAIT_LOAD', 'WAIT_RECEIVE']))
        self.beep_distance = float(p('beep_distance', 0.45))
        self.sector_deg = float(p('sector_deg', 50.0))
        self.front_offset = float(p('front_offset_deg', 0.0))
        self.min_valid = float(p('min_valid', 0.08))
        self.beep_interval = float(p('beep_interval', 1.0))
        self.near_goal_mute = float(p('near_goal_mute', 0.5))
        scan_topic = p('scan_topic', '/scan')
        ns = (self.get_namespace() if hasattr(self, 'get_namespace') else '/').strip('/')
        self.robot_name = p('robot_name', '') or ns or 'pinky_1'
        status_topic = p('status_topic', '/delivery/status')
        self.player = Player(self, p('player', 'aplay -q'))

        for f in (self.song_file, self.beep_file):
            if f and not os.path.isfile(f):
                self.get_logger().warn(f'Sound file not found: {f}')

        self.state = None
        self.dist_remaining = None
        self.last_song = 0.0
        self.last_beep = 0.0
        self.front = None

        self.create_subscription(String, status_topic, self.on_status, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Ready: beep<{self.beep_distance}m in front {self.sector_deg}deg, '
            f'song on {self.wait_states}, robot {self.robot_name}')

    # 3) song ------------------------------------------------------------
    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        robots = st.get('robots')
        if isinstance(robots, dict):          # 2-robot status: follow this robot only
            st = robots.get(self.robot_name)
            if not isinstance(st, dict):
                return
        new = st.get('state')
        self.dist_remaining = st.get('distance_remaining')
        now = time.monotonic()
        if new != self.state:
            self.get_logger().info(f'state {self.state} -> {new}')
            if self.state in self.wait_states:
                self.player.stop('song')
            self.state = new
            if new in self.wait_states:
                self.last_song = now
                self.player.play('song', self.song_file)
        elif new in self.wait_states and self.song_repeat > 0 \
                and now - self.last_song >= self.song_repeat:
            self.last_song = now
            self.player.play('song', self.song_file)

    # 5) beep ------------------------------------------------------------
    def on_scan(self, scan):
        self.front = front_min_distance(scan, self.sector_deg, self.front_offset, self.min_valid)
        if self.state not in MOVING or self.front is None or self.front >= self.beep_distance:
            return
        if self.near_goal_mute > 0 and self.dist_remaining is not None \
                and self.dist_remaining < self.near_goal_mute:
            return
        now = time.monotonic()
        if now - self.last_beep >= self.beep_interval:
            self.last_beep = now
            self.get_logger().info(f'Obstacle/person {self.front:.2f} m ahead: beep')
            self.player.play('beep', self.beep_file)


def main():
    rclpy.init()
    node = RobotNotifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.player.stop('song')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
