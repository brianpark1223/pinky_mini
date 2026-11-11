#!/usr/bin/env python3
"""Delivery control GUI (memo item 4). Run on the operator laptop.

Panels
  작업 요청   : one panel per workstation: 부품 요청 (click again = another request; 2 s lock per click)
               and 수령 완료 (enabled while a Pinky waits at that workstation); replies go to the log
  적재 확인   : 적재 완료 per robot (enabled in WAIT_LOAD); receiving is shown only
  복구       : retry / reset (enabled in ERROR), RViz launcher
  로봇 상태   : state, target, distance left, timeout, pose (amcl), Nav2 ready
  작업 상태   : current task, batch (if the controller publishes one), queue, error,
               and the raw status JSON (shows any extra field the task logic adds)
  카메라      : sensor_msgs/CompressedImage or Image
  로그       : every service reply with a timestamp

The GUI only talks to the controller through these ROS interfaces, so the
task logic can change freely as long as they stay the same:
  services  std_srvs/Trigger: delivery/request_<station lower>, and
            1 robot : delivery/loaded, delivery/received, delivery/retry, delivery/reset
            2 robots: delivery/<robot>/loaded, .../received, .../retry, .../reset
  topic     delivery/status (std_msgs/String JSON)
            1 robot : {"state", "current", "target", "queue", ...}
            2 robots: {"robots": {"pinky_1": {"state", "current", "target", ...}, ...}, "queue": [...]}
            extra robot state WAIT_PICKUP = waiting (where the controller parked it) while the other robot uses the pickup

Parameters
  stations 'A,B' | station_labels '작업대1,작업대2'
  robots 'pinky_1,pinky_2' (robots to show; with the 1-robot controller only the first is used)
  robot_ns '/{robot}' (per-robot topics <ns>/amcl_pose, <ns>/plan, <ns>/delivery/dock, <ns><camera_topic>;
            the first robot also listens to the plain topics)
  robot_name = first of robots (name for a 1-robot status without a 'robot' field)
  camera_topic '/camera/image_raw/compressed' ('' = off)
  arrive_tolerance 0.3 (m; Nav2 goal tolerance, ETA counts each leg as ending this early)
  ArUco docking (ETA only; the docking itself is the camera node's job)
    dock_topic 'delivery/dock' (std_msgs/String JSON {"distance": m, "angle_deg": deg, "visible": bool})
    dock_points 'STORE' (points that end with ArUco docking, comma list; '' = none)
    dock_speed 0.05 (m/s, refined from real dockings) | turn_speed 30 (deg/s)
    dock_approach 0.4 (m, Nav2 goal -> marker stop point, used before the marker is seen)
    dock_turn_after 0 (deg, extra turn after docking, e.g. 180 to face away from the marker)
  pose_source 'auto' (auto = TF, falls back to amcl_pose (+ odometry) | tf | amcl)
  odom_topic '/odom' (per robot <ns>/odom; fills gaps between amcl_pose updates, '' = off)
  pose_smooth 0.3 s (map display smoothing; 2 cm / 1 deg jitter ignored, jumps > 0.5 m snap; 0 = off)
  map_frame 'map' | base_frame 'auto' (<robot>/base_footprint, then base_footprint / base_link)
  pose_topic '/amcl_pose' | rviz_config '' (path to .rviz file)
  map_yaml '' (saved map, e.g. ~/maps/pinky.yaml: shown at once, replaced by /map when it arrives)
  map_style 'clean' (tidy display map: no laser streaks, solid walls) | 'raw' (cells as RViz)
  map_rotate 'auto' (turn the view so walls are level) | degrees, e.g. '0' = same as RViz
  floorplan 'auto' (<map_yaml name>.floorplan.json next to the yaml) | path | 'none'
            hand-drawn floor plan (walls, glass, obstacles, tables) in map metres, display only

Requires PyQt5 or PyQt6 (either works):  sudo apt install python3-pyqt5  /  pip install PyQt6
"""
import json
import math
import os
import re
from collections import deque
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Qt binding: PyQt5 if installed, else PyQt6 (set PINKY_QT=6 to force PyQt6).
if os.environ.get('PINKY_QT') != '6':
    try:
        from PyQt5.QtCore import QPointF, QRectF, Qt, QProcess, QSize, QTimer
        from PyQt5.QtGui import (QColor, QFont, QIcon, QImage, QPainter, QPainterPath, QPen, QPixmap,
                                 QTransform)
        from PyQt5.QtWidgets import (
            QApplication, QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QListWidget,
            QMessageBox, QPlainTextEdit, QPushButton, QSizePolicy, QTabWidget, QVBoxLayout, QWidget)
        QT_API = 'PyQt5'
    except ImportError:
        QT_API = None
else:
    QT_API = None
if QT_API is None:
    import enum
    from PyQt6.QtCore import QPointF, QRectF, Qt, QProcess, QSize, QTimer
    from PyQt6.QtGui import (QColor, QFont, QIcon, QImage, QPainter, QPainterPath, QPen, QPixmap,
                             QTransform)
    from PyQt6.QtWidgets import (
        QApplication, QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QListWidget,
        QMessageBox, QPlainTextEdit, QPushButton, QSizePolicy, QTabWidget, QVBoxLayout, QWidget)
    QT_API = 'PyQt6'

    def _flatten_enums(*classes):
        """PyQt6 scopes enums (Qt.AlignmentFlag.AlignCenter); also expose them as Qt.AlignCenter."""
        for cls in classes:
            for name in dir(cls):
                obj = getattr(cls, name, None)
                if isinstance(obj, type) and issubclass(obj, enum.Enum):
                    for mname, member in obj.__members__.items():   # includes combined flags
                        if not hasattr(cls, mname):
                            setattr(cls, mname, member)
    _flatten_enums(Qt, QImage, QPainter, QSizePolicy, QMessageBox, QProcess, QFont, QListWidget, QFrame)


def qimage_bytes(img, n):
    """Raw bytes of a QImage (PyQt5 and PyQt6)."""
    ptr = img.constBits()
    try:
        return ptr.asstring(n)
    except (AttributeError, TypeError):
        ptr.setsize(n)
        return bytes(ptr)

# state -> (Korean label, accent, soft background, dark text)
TEAL = ('#1D9E75', '#E1F5EE', '#085041')
AMBER = ('#EF9F27', '#FAEEDA', '#633806')
GRAY = ('#888780', '#F1EFE8', '#444441')
RED = ('#E24B4A', '#FCEBEB', '#791F1F')
STATE_STYLE = {
    'IDLE': ('대기 중', *GRAY),
    'TO_STORE': ('픽업 장소로 이동 중', *TEAL),
    'WAIT_PICKUP': ('픽업 차례 대기 · 다른 Pinky 적재 중', '#7F77DD', '#EEEDFE', '#3C3489'),
    'WAIT_TRAFFIC': ('공용 통로 진입 대기', '#7F77DD', '#EEEDFE', '#3C3489'),
    'DOCKING': ('픽업대 마커에 맞추는 중', *TEAL),   # only if the controller adds this state
    'TO_STATION': ('작업대로 배달 중', *TEAL),
    'TO_HOME': ('수납 위치로 복귀 중', *TEAL),
    'WAIT_LOAD': ('적재 대기 · 부품을 올려 주세요', *AMBER),
    'WAIT_RECEIVE': ('수령 대기 · 부품을 꺼내 주세요', *AMBER),
    'ERROR': ('오류 · 확인이 필요합니다', *RED),
}
STEPS = [('TO_STORE', '픽업 이동'), ('WAIT_LOAD', '적재'), ('TO_STATION', '배달'),
         ('WAIT_RECEIVE', '수령'), ('TO_HOME', '복귀')]
STALE_SEC = 2.0
MOVING_STATES = ('TO_STORE', 'TO_STATION', 'TO_HOME')  # 'DOCKING' handled separately


def josa(word, with_final='이', without_final='가'):
    """Korean particle after a name: 'pinky_1' + 이, 'pinky_2' + 가, '작업대' + 가."""
    ch = str(word)[-1:] if word else ''
    if '가' <= ch <= '힣':
        has_final = (ord(ch) - 0xAC00) % 28 != 0
    elif ch.isdigit():
        has_final = ch in '013678'          # 영/일/삼/육/칠/팔
    else:
        has_final = ch.lower() in 'lmn'
    return with_final if has_final else without_final


def fmt_left(sec):
    """'2m 05s left' style countdown text."""
    if sec is None:
        return '-'
    m, s = divmod(max(0, int(round(sec))), 60)
    return f'{m}m {s:02d}s left' if m else f'{s}s left'


def dock_seconds(dock, live=None):
    """Time for the ArUco docking phase.

    live = latest camera feedback {'distance', 'angle_deg'} while docking is under way;
    None = docking not started yet -> the approach distance, angle assumed already aligned.
      t = distance / dock_speed + |angle| / turn_speed + turn_after / turn_speed
    """
    if not dock:
        return 0.0
    dist, ang = dock['approach'], 0.0
    if live:
        if isinstance(live.get('distance'), (int, float)):
            dist = live['distance']
        if isinstance(live.get('angle_deg'), (int, float)):
            ang = live['angle_deg']
    turn = max(1e-3, dock['turn_speed'])
    return max(0.0, dist) / max(1e-3, dock['speed']) + (abs(ang) + abs(dock['turn_after'])) / turn


def estimate_fleet(robots, queue, poses, points, speed, detour=1.25, arrive_tol=0.3, dock=None,
                   dock_lives=None):
    """Travel-time estimate (s) for one or more Pinkys sharing one pickup.

    robots : {name: robot status (state, current, target, distance_remaining)}, in priority order
    poses / dock_lives : {name: (x, y, yaw) | None} / {name: camera feedback | None}
    Returns (cur, etas, who, pick):
      cur   {name: seconds until that robot's current parts arrive | None}
      etas  [seconds until each queued task arrives | None]
      who   [robot expected to take each queued task | None]  (earliest at the pickup)
      pick  [seconds until that robot is at the pickup, ready to load | None]
    Rules:
    - a leg = Nav2 driving (+ ArUco docking if it ends at a dock point); the leg being driven
      uses Nav2's distance_remaining or the camera feedback, other legs straight line x detour;
      Nav2 reports arrival arrive_tol early;
    - one pickup: a robot that gets there while another is loading waits where it is (WAIT_PICKUP);
    - loading/unloading time is unknown, so it is not counted (waiting = clock stops);
    - a queued task goes to the robot that can be at the pickup first.
    """
    dock_lives = dock_lives or {}
    names = list(robots)
    if not speed or speed <= 0 or 'STORE' not in points:
        return {n: None for n in names}, [None] * len(queue), [None] * len(queue), [None] * len(queue)
    dock_pts = set((dock or {}).get('points') or ())

    def xy(key):
        pt = points.get(key)
        return None if pt is None else (pt[0], pt[1])

    def docking(key):
        return dock_seconds(dock) if key in dock_pts else 0.0

    def leg(a, b, key=None):
        if a is None or b is None:
            return None
        return max(0.0, math.hypot(a[0] - b[0], a[1] - b[1]) * detour - arrive_tol) / speed + docking(key)

    def add(*xs):
        return None if any(x is None for x in xs) else sum(xs)

    def later(a, b):
        return None if a is None or b is None else max(a, b)

    store = xy('STORE')
    cur, free, holders, waiters = {}, {}, [], []
    for name in names:
        rs = robots[name] or {}
        state, target, c = rs.get('state'), rs.get('target'), rs.get('current')
        pose = (poses or {}).get(name)
        here = (pose[0], pose[1]) if pose else None
        live = dock_lives.get(name)

        def leg_now(key):
            if key in dock_pts and live is not None and target == key:
                return dock_seconds(dock, live)          # Nav2 part done, aligning to the marker
            d = rs.get('distance_remaining')
            if target == key and isinstance(d, (int, float)):
                return max(0.0, d - arrive_tol) / speed + docking(key)
            return leg(here, xy(key), key)

        sc = c.get('station') if isinstance(c, dict) else None
        cur[name], free[name] = None, None
        if state == 'ERROR':
            continue
        if sc is None:                                   # IDLE / going home: free now
            free[name] = (0.0, here)
            continue
        if state == 'WAIT_PICKUP':
            waiters.append((0.0, name, sc, here))
        elif state in ('TO_STORE', 'DOCKING'):
            holders.append((leg_now('STORE'), name, sc))
        elif state == 'WAIT_LOAD':
            holders.append((0.0, name, sc))
        elif state == 'TO_STATION':
            cur[name] = leg_now(sc)
            free[name] = (cur[name], xy(sc))
        elif state == 'WAIT_RECEIVE':
            cur[name] = 0.0
            free[name] = (0.0, xy(sc))

    pickup_free = 0.0            # when the pickup is next empty (None = unknown)
    for t_arr, name, sc in sorted(holders, key=lambda h: (h[0] is None, h[0] or 0.0)):
        start = later(t_arr, pickup_free)
        pickup_free = start
        cur[name] = add(start, leg(store, xy(sc), sc))
        free[name] = (cur[name], xy(sc))
    for t_w, name, sc, here in sorted(waiters, key=lambda h: (h[0] is None, h[0] or 0.0)):
        start = add(later(t_w, pickup_free), leg(here, store, 'STORE'))   # waits where it is, then drives in
        pickup_free = start
        cur[name] = add(start, leg(store, xy(sc), sc))
        free[name] = (cur[name], xy(sc))

    etas, who, pick = [], [], []
    for task in queue:
        s = task.get('station') if isinstance(task, dict) else None
        best = None
        for name in names:
            if free[name] is None:
                continue
            arr = add(free[name][0], leg(free[name][1], store, 'STORE'))
            if arr is not None and (best is None or arr < best[0]):
                best = (arr, name)
        if best is None:
            etas.append(None), who.append(None), pick.append(None)
            continue
        start = later(best[0], pickup_free)
        pickup_free = start
        eta = add(start, leg(store, xy(s), s))
        etas.append(eta), who.append(best[1]), pick.append(start)
        free[best[1]] = (eta, xy(s))
    return cur, etas, who, pick


def estimate_etas(status, pose, points, speed, detour=1.25, arrive_tol=0.3, dock=None, dock_live=None):
    """One robot: (current task eta, [eta per queued task]). See estimate_fleet."""
    st = status or {}
    queue = list(st.get('queue') or [])
    if st.get('state') == 'ERROR':
        return None, [None] * len(queue)
    cur, etas, _, _ = estimate_fleet({'r': st}, queue, {'r': pose}, points, speed, detour, arrive_tol,
                                     dock, {'r': dock_live})
    return cur['r'], etas


def fmt_dur(sec):
    """'1m 05s' (no 'left')."""
    return fmt_left(sec).replace(' left', '')

QSS = """
QWidget#root { background: #F6F5F1; }
QWidget { color: #2C2C2A; font-size: 13px; }
QFrame[card="true"] { background: #FFFFFF; border: 1px solid #E5E3DC; border-radius: 14px; }
QLabel[role="cardTitle"] { color: #5F5E5A; font-size: 12px; font-weight: 600; letter-spacing: 0.5px; }
QLabel[role="muted"] { color: #888780; font-size: 12px; }
QLabel[role="tileLabel"] { color: #888780; font-size: 11px; }
QLabel[role="tileValue"] { color: #2C2C2A; font-size: 15px; font-weight: 600; }
QLabel[role="title"] { font-size: 20px; font-weight: 700; color: #2C2C2A; }
QFrame[tile="true"] { background: #F6F5F1; border-radius: 10px; }

QPushButton { background: #FFFFFF; border: 1px solid #D3D1C7; border-radius: 10px;
              padding: 9px 14px; font-size: 13px; }
QPushButton:hover { background: #F1EFE8; }
QPushButton:pressed { background: #E5E3DC; }
QPushButton:disabled { color: #B4B2A9; border-color: #ECEAE4; background: #FAFAF8; }
QPushButton[kind="primary"] { background: #1D9E75; color: white; border: none;
              font-size: 15px; font-weight: 600; padding: 16px; text-align: left; padding-left: 18px; }
QPushButton[kind="primary"]:hover { background: #0F6E56; }
QPushButton[kind="primary"]:pressed { background: #085041; }
QPushButton[kind="primary"]:disabled { background: #D3D1C7; color: #6E6C66; }
QPushButton[kind="confirm"] { background: #EF9F27; color: #412402; border: none;
              font-size: 15px; font-weight: 600; padding: 16px; }
QPushButton[kind="confirm"]:hover { background: #DE8E18; }
QPushButton[kind="confirm"]:disabled { background: #F1EFE8; color: #B4B2A9; }
QPushButton[kind="danger"] { color: #A32D2D; border-color: #F09595; }
QPushButton[kind="danger"]:hover { background: #FCEBEB; }
QPushButton[kind="link"] { border: none; background: transparent; color: #185FA5;
              padding: 2px 0; text-align: left; font-size: 12px; }

QTabWidget::pane { background: #FFFFFF; border: 1px solid #E5E3DC; border-radius: 14px;
              border-top-left-radius: 0; top: -1px; }
QTabBar::tab { background: #E9E7E0; color: #5F5E5A; padding: 11px 20px; margin-right: 4px;
              border-top-left-radius: 12px; border-top-right-radius: 12px; min-width: 170px;
              font-size: 14px; border: 1px solid transparent; border-bottom: none; }
QTabBar::tab:selected { background: #FFFFFF; color: #2C2C2A; font-weight: 700;
              border: 1px solid #E5E3DC; border-bottom: 1px solid #FFFFFF; }
QTabBar::tab:hover:!selected { background: #F1EFE8; }
QListWidget { border: none; background: transparent; outline: none; }
QListWidget::item { background: #F6F5F1; border-radius: 8px; padding: 8px 10px; margin: 2px 0; }
QPlainTextEdit { background: #FAFAF8; border: 1px solid #ECEAE4; border-radius: 10px;
              padding: 6px; font-family: 'DejaVu Sans Mono', monospace; font-size: 12px; }
QScrollBar:vertical { width: 8px; background: transparent; }
QScrollBar::handle:vertical { background: #D3D1C7; border-radius: 4px; min-height: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
"""


def fmt_task(t):
    if t is None:
        return '-'
    if isinstance(t, dict):
        extra = ', '.join(f'{k}={v}' for k, v in t.items() if k not in ('id', 'station', 'robot'))
        s = f"#{t.get('id', '?')} → {t.get('station', '?')}"
        return f'{s} ({extra})' if extra else s
    return str(t)


# ---------------------------------------------------------------- display map
def _grow(mask, k):
    """Binary dilation by a disc of radius k cells (numpy only, no scipy needed)."""
    if k <= 0:
        return mask.copy()
    h, w = mask.shape
    pad = np.pad(mask, k)
    out = np.zeros_like(mask)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy <= k * k:
                out |= pad[k + dy:k + dy + h, k + dx:k + dx + w]
    return out


def _shrink(mask, k):
    return ~_grow(~mask, k)


def _flood(seeds, allowed):
    """Cells of `allowed` 4-connected to any seed cell."""
    h, w = allowed.shape
    seen = seeds & allowed
    q = deque(zip(*np.nonzero(seen)))
    while q:
        r, c = q.popleft()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and allowed[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                q.append((rr, cc))
    return seen


def _big_parts(mask):
    """Keep connected parts larger than 5 % of the largest one."""
    left, parts = mask.copy(), []
    while left.any():
        r, c = next(zip(*np.nonzero(left)))
        seed = np.zeros_like(mask)
        seed[r, c] = True
        part = _flood(seed, left)
        parts.append(part)
        left &= ~part
    if not parts:
        return mask
    biggest = max(p.sum() for p in parts)
    keep = np.zeros_like(mask)
    for part in parts:
        if part.sum() > 0.05 * biggest:
            keep |= part
    return keep


def build_display_map(cells, res):
    """Tidy picture of a SLAM map for people (image layout, row 0 = top).

    cells: -1 unknown / 0 free / 100 occupied. Returns (rgb, room_mask, wall_mask) or None.
    Laser streaks (thin free strips into unknown space) are cut off, unknown pockets
    inside the room are filled, walls are drawn solid and furniture as light dots.
    Only the picture changes: coordinates stay exactly those of the map.
    """
    free, occ = cells == 0, cells == 100
    k = max(1, int(round(0.25 / res)))
    core = _big_parts(_shrink(free | occ, k))     # thin streaks vanish under erosion
    if not core.any():
        return None
    room = _grow(core, k + 1) & (free | occ)
    blocked = room | (occ & _grow(room, 2))
    border = np.zeros_like(room)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    outside = _flood(border & ~blocked, ~blocked)
    filled = ~outside                              # room incl. unknown pockets under tables
    edge = filled & _grow(outside, 3)
    walls = _shrink(_grow(occ & edge, 2), 1) & _grow(filled, 1)
    furniture = occ & filled & ~edge
    rgb = np.empty(cells.shape + (3,), np.uint8)
    rgb[:] = (241, 239, 232)
    rgb[filled & ~walls] = (255, 255, 255)
    rgb[furniture] = (226, 224, 217)
    rgb[walls] = (95, 94, 90)
    return rgb, filled, walls


def dominant_angle(mask):
    """Angle (deg, -45..45) that makes most wall pixels line up with the screen axes."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 50:
        return 0.0
    if len(xs) > 20000:
        idx = np.linspace(0, len(xs) - 1, 20000).astype(int)
        xs, ys = xs[idx], ys[idx]
    best = (0, 0.0)
    for a2 in range(0, 180):
        a = math.radians(a2 / 2)
        u = xs * math.cos(a) + ys * math.sin(a)
        v = -xs * math.sin(a) + ys * math.cos(a)
        score = (np.bincount((u - u.min()).astype(int)) ** 2).sum() + \
            (np.bincount((v - v.min()).astype(int)) ** 2).sum()
        if score > best[0]:
            best = (score, a2 / 2)
    a = best[1]
    return a if a < 45 else a - 90


class Track:
    """What the GUI knows about one Pinky (pose, plan, camera, docking, speed bookkeeping)."""

    def __init__(self, name, base_frame):
        self.name = name
        self.base_frame = base_frame
        self.pose = None           # (x, y, yaw) shown on screen
        self.pose_source = None    # 'TF' | 'AMCL' | '끊김'
        self.amcl_pose, self.amcl_time = None, 0.0
        self.tf_pose, self.tf_stamp, self.tf_fresh, self.tf_error_logged = None, None, 0.0, False
        self.plan = []             # [(x, y), ...] in map frame
        self.odom, self.odom_time = None, 0.0      # wheel odometry (odom frame)
        self.amcl_ref_odom = None  # odometry when the last amcl_pose arrived
        self.view_pose = None      # smoothed pose drawn on the map
        self.frame, self.frame_time = None, 0.0
        self.dock_fb, self.dock_fb_time, self.dock_run = None, 0.0, None
        self.leg_state, self.leg_start, self.leg_last, self.leg_dist = None, None, None, 0.0
        self.leg_dock_at = None


TRACK_ATTRS = {'pose', 'pose_source', 'odom', 'odom_time', 'amcl_ref_odom', 'view_pose', 'amcl_pose', 'amcl_time', 'tf_pose', 'tf_stamp', 'tf_fresh',
               'tf_error_logged', 'plan', 'frame', 'frame_time', 'dock_fb', 'dock_fb_time', 'dock_run',
               'leg_state', 'leg_start', 'leg_last', 'leg_dist', 'leg_dock_at', 'base_frame'}


class GuiNode(Node):
    """ROS side: subscriptions cache data, services are called asynchronously.

    One controller, one or more robots. node.pose / node.plan / ... are the first robot's
    (kept for single-robot code and tests); per robot: node.tracks[name].
    """

    def __getattr__(self, name):                 # only for attributes not found normally
        if name in TRACK_ATTRS and 'tracks' in self.__dict__:
            return getattr(self.main, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name in TRACK_ATTRS and 'tracks' in self.__dict__:
            setattr(self.main, name, value)
        else:
            super().__setattr__(name, value)

    def __init__(self):
        super().__init__('delivery_gui')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.stations = [s.strip() for s in p('stations', 'A,B').split(',') if s.strip()]
        labels = [s.strip() for s in p('station_labels', '작업대1,작업대2').split(',') if s.strip()]
        self.station_labels = labels if len(labels) == len(self.stations) else self.stations
        self.camera_topic = p('camera_topic', '/camera/image_raw/compressed')
        self.rviz_config = p('rviz_config', '')
        pose_topic = p('pose_topic', '/amcl_pose')
        self.pose_mode = p('pose_source', 'auto')
        self.map_frame = p('map_frame', 'map')
        base_frame = p('base_frame', 'auto')   # auto: <robot>/base_footprint, base_footprint, base_link
        map_topic = p('map_topic', '/map')
        plan_topic = p('plan_topic', '/plan')
        odom_topic = p('odom_topic', '/odom')      # fills the gaps between amcl_pose updates ('' = off)
        self.pose_smooth = float(p('pose_smooth', 0.3))   # s, display smoothing (0 = off)
        dock_topic = p('dock_topic', 'delivery/dock')
        ns_fmt = p('robot_ns', '/{robot}')     # per-robot topics: <ns>/amcl_pose, <ns>/plan, ...
        map_yaml = p('map_yaml', 'auto')
        if map_yaml == 'auto':  # no option given (e.g. VS Code run button): newest known map in ~/maps
            here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps')
            guesses = [os.path.expanduser('~/maps/sesac_gangnam_2.yaml'), os.path.join(here, 'sesac_gangnam_2.yaml'),
                       os.path.expanduser('~/maps/sesac_gangnam.yaml')]
            map_yaml = next((g for g in guesses if os.path.isfile(g)), '')
        self.map_style = p('map_style', 'clean')
        self.map_rotate = str(p('map_rotate', 'auto'))
        floorplan = str(p('floorplan', 'auto'))
        self.points = self._load_points(p('points', ''))
        self.speed = float(p('nominal_speed', 0.15))   # m/s, refined from real legs (all robots)
        self.detour = float(p('detour_factor', 1.25))  # path length / straight line
        self.arrive_tol = float(p('arrive_tolerance', 0.3))  # Nav2 counts this close as arrived
        self.popup_on_request = bool(p('popup_on_request', True))
        self.request_lock_sec = float(p('request_lock_sec', 2.0))  # anti double-click lock
        self.dock = {'points': [x.strip() for x in str(p('dock_points', 'STORE')).split(',') if x.strip()],
                     'speed': float(p('dock_speed', 0.05)),         # m/s while tracking the marker
                     'turn_speed': float(p('turn_speed', 30.0)),    # deg/s
                     'approach': float(p('dock_approach', 0.4)),    # m, Nav2 goal -> stop point
                     'turn_after': float(p('dock_turn_after', 0.0))}
        self.dock_learned = False
        self.speed_learned = False
        names = [r.strip() for r in str(p('robots', 'pinky_1,pinky_2')).split(',') if r.strip()]
        self.robot_name = p('robot_name', names[0] if names else 'pinky_1')  # single-robot status name
        names = names or [self.robot_name]
        if self.robot_name not in names:
            names.insert(0, self.robot_name)
        self.robot_names = names
        self.tracks = {n: Track(n, base_frame) for n in names}
        self.main = self.tracks[names[0]]

        self.status = None
        self.status_time = 0.0
        self.log_cb = print
        self.map = None            # dict: img, res, ox, oy, h, angle, c0, ub, bg
        self.map_source = None     # 'file' | 'topic'
        self.floorplan = None      # hand-drawn display plan (dict), see load_floorplan
        self.fp_view = None
        self.clients_ = {}

        self.create_subscription(String, 'delivery/status', self.on_status, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, latched)
        for i, name in enumerate(names):
            tr = self.tracks[name]
            ns = ns_fmt.format(robot=name).rstrip('/') if ns_fmt else ''
            topics = []   # (kind, topic): namespaced ones, plus the plain ones for the first robot
            for kind, plain, rel in (('pose', pose_topic, 'amcl_pose'), ('plan', plan_topic, 'plan'),
                                     ('odom', odom_topic, (odom_topic or '').lstrip('/')),
                                     ('dock', dock_topic, 'delivery/dock'),
                                     ('cam', self.camera_topic, self.camera_topic.lstrip('/'))):
                if not plain:
                    continue
                if ns:
                    topics.append((kind, f'{ns}/{rel}'))
                if i == 0:
                    topics.append((kind, plain))
            for kind, topic in topics:
                if kind == 'pose':
                    self.create_subscription(PoseWithCovarianceStamped, topic,
                                             lambda m, t=tr: self.on_pose(m, t), 10)
                elif kind == 'plan':
                    self.create_subscription(Path, topic, lambda m, t=tr: self.on_plan(m, t), 10)
                elif kind == 'odom':
                    self.create_subscription(Odometry, topic, lambda m, t=tr: self.on_odom(m, t),
                                             qos_profile_sensor_data)
                elif kind == 'dock':
                    self.create_subscription(String, topic, lambda m, t=tr: self.on_dock(m, t), 10)
                elif topic.endswith('compressed'):
                    self.create_subscription(CompressedImage, topic, lambda m, t=tr: self.on_jpeg(m, t),
                                             qos_profile_sensor_data)
                else:
                    self.create_subscription(Image, topic, lambda m, t=tr: self.on_raw(m, t),
                                             qos_profile_sensor_data)
        if map_yaml:
            self.load_map_file(os.path.expanduser(map_yaml))
        fp_path = ''
        self.fp_missing = False
        if floorplan == 'auto':
            # next to the map yaml, else the plan shipped in this folder's maps/ (map may come from /map)
            here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps')
            cands = ([os.path.splitext(os.path.expanduser(map_yaml))[0] + '.floorplan.json'] if map_yaml else []) + \
                [os.path.expanduser('~/maps/sesac_gangnam_2.floorplan.json'),
                 os.path.join(here, 'sesac_gangnam_2.floorplan.json')]
            fp_path = next((c for c in cands if os.path.isfile(c)), '')
            self.fp_missing = not fp_path
        elif floorplan not in ('auto', '', 'none'):
            fp_path = os.path.expanduser(floorplan)
        if fp_path:
            self.load_floorplan(fp_path)

    # -- fleet view of the status
    def fleet(self):
        """Status in one shape for 1 or 2 robots.

        2-robot controller: {"robots": {name: {...}}, "queue": [...], ...}
        2-robot controller = fms_controller.py; 1-robot controller (parts_delivery.py):
        top-level state/current/... = one robot.
        """
        st = self.status or {}
        robots = st.get('robots')
        if isinstance(robots, dict) and robots:
            order = [n for n in self.robot_names if n in robots] + \
                [n for n in robots if n not in self.robot_names]
            rs = {n: robots[n] if isinstance(robots[n], dict) else {} for n in order}
            multi = True
        else:
            name = str(st.get('robot') or self.robot_name)
            rs, multi = {name: st}, False
        for n in rs:
            if n not in self.tracks:            # a robot the params did not list: no topics, status only
                self.tracks[n] = Track(n, 'auto')
        return {'robots': rs, 'queue': list(st.get('queue') or []), 'multi': multi,
                'dry_run': st.get('dry_run'), 'raw': st}

    def robot_status(self, name):
        return self.fleet()['robots'].get(name) or {}

    # -- subscriptions
    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_time = time.monotonic()
        except ValueError:
            return
        pts = self.status.get('points')
        if isinstance(pts, dict) and pts:      # the controller's fixed points win over the local copy
            try:
                self.points = {k: tuple(float(c) for c in v) for k, v in pts.items()}
            except (TypeError, ValueError):
                pass
        for name, rs in self.fleet()['robots'].items():   # controller may relay the feedback itself
            if isinstance(rs.get('dock'), dict):
                self.set_dock_fb(rs['dock'], self.tracks[name])

    def on_dock(self, msg, track=None):
        try:
            fb = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(fb, dict):
            if fb.get('robot') in self.tracks:
                track = self.tracks[fb['robot']]
            self.set_dock_fb(fb, track)

    def set_dock_fb(self, fb, track=None):
        """Marker lost ('visible': false) keeps the last distance/angle: still docking."""
        tr = track or self.main
        old = tr.dock_fb or {}
        if fb.get('visible') is False:
            fb = dict(fb, distance=old.get('distance'), angle_deg=old.get('angle_deg'))
        tr.dock_fb, tr.dock_fb_time = fb, time.monotonic()

    def dock_live(self, now=None, name=None):
        """Camera feedback if that Pinky is docking right now (fresh feedback, heading to a dock point)."""
        now = time.monotonic() if now is None else now
        name = name or self.main.name
        tr = self.tracks.get(name)
        rs = self.robot_status(name)
        if (tr is None or tr.dock_fb is None or now - tr.dock_fb_time > 1.0
                or rs.get('target') not in self.dock['points']
                or rs.get('state') not in MOVING_STATES + ('DOCKING',)):
            return None
        return tr.dock_fb

    def dock_lives(self):
        return {n: self.dock_live(name=n) for n in self.fleet()['robots']}

    def poses(self):
        return {n: self.tracks[n].pose for n in self.fleet()['robots']}

    def _learn_dock(self, now):
        """Docking speed from each finished docking (turning time taken out); shared by all robots."""
        for name, tr in self.tracks.items():
            live = self.dock_live(now, name)
            if live is not None and tr.dock_run is None:
                d = live.get('distance')
                if isinstance(d, (int, float)):
                    tr.dock_run = (now, d, abs(live.get('angle_deg') or 0.0))
            elif live is None and tr.dock_run is not None:
                t0, d0, a0 = tr.dock_run
                tr.dock_run = None
                turn = (a0 + abs(self.dock['turn_after'])) / max(1e-3, self.dock['turn_speed'])
                dur = now - t0 - turn
                if self.robot_status(name).get('state') == 'WAIT_LOAD' and dur > 1.0 and d0 > 0.05:
                    v = d0 / dur
                    self.dock['speed'] = min(0.5, max(0.01, 0.6 * self.dock['speed'] + 0.4 * v))
                    self.dock_learned = True
                    self.log_cb(f'[도킹] {name} 이번 정렬 {v:.3f} m/s → 평균 {self.dock["speed"]:.3f} m/s')

    @staticmethod
    def _load_points(text):
        """Named poses for the map: JSON param, else POINTS read from fms_controller.py / parts_delivery.py
        (the controller's status 'points' replaces them as soon as it arrives)."""
        try:
            if text:
                return {k: tuple(v) for k, v in json.loads(text).items()}
        except ValueError:
            return {}
        here = os.path.dirname(os.path.abspath(__file__))
        for fname in ('fms_controller.py', 'parts_delivery.py'):
            try:
                src = open(os.path.join(here, fname), encoding='utf-8').read()
                block = src[src.index('POINTS = {'):]
                block = block[:block.index('\n}') + 2]
                env = {}
                exec(block, {'math': math}, env)   # just the dict literal, no ROS imports
                return {k: tuple(v) for k, v in env['POINTS'].items()}
            except (OSError, ValueError, KeyError, SyntaxError):
                continue
        return {}

    def on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        o = msg.info.origin.position
        if self.map_source == 'file':
            self.log_cb('[지도] /map 수신 → 파일 지도 대신 map_server 지도를 표시합니다')
        self.set_grid(grid, msg.info.resolution, o.x, o.y, 'topic')

    def load_map_file(self, path):
        """Read a map_server map (yaml + pgm/png) the way map_server does (trinary mode)."""
        try:
            try:
                import yaml
                with open(path, encoding='utf-8') as f:
                    meta = yaml.safe_load(f)
            except ImportError:
                meta = {}
                with open(path, encoding='utf-8') as f:
                    for line in f:
                        if ':' in line and not line.lstrip().startswith('#'):
                            k, v = line.split(':', 1)
                            meta[k.strip()] = v.strip()
                if isinstance(meta.get('origin'), str):
                    meta['origin'] = [float(t) for t in meta['origin'].strip('[]').split(',')]
            img_path = os.path.join(os.path.dirname(os.path.abspath(path)), str(meta['image']))
            qimg = QImage(img_path)
            if qimg.isNull():
                raise ValueError(f'이미지를 읽을 수 없음: {img_path}')
            qimg = qimg.convertToFormat(QImage.Format_Grayscale8)
            w, h, bpl = qimg.width(), qimg.height(), qimg.bytesPerLine()
            pix = np.frombuffer(qimage_bytes(qimg, bpl * h), np.uint8).reshape(h, bpl)[:, :w]
            occ = float(meta.get('occupied_thresh', 0.65))
            free = float(meta.get('free_thresh', 0.196))
            negate = int(meta.get('negate', 0))
            prob = pix / 255.0 if negate else (255.0 - pix) / 255.0
            grid = np.full((h, w), -1, np.int8)
            grid[prob > occ] = 100
            grid[prob < free] = 0
            ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
            self.set_grid(np.flipud(grid), float(meta['resolution']), ox, oy, 'file')
            self.log_cb(f'[지도] 파일에서 읽음: {path} ({w}x{h}, {float(meta["resolution"])} m/칸)')
        except Exception as exc:  # noqa: BLE001
            self.log_cb(f'[지도] map_yaml 읽기 실패: {exc}')

    def set_grid(self, grid, res, ox, oy, source):
        """grid: OccupancyGrid layout (row 0 = lowest y), -1 unknown / 0 free / 100 occupied."""
        h, w = grid.shape
        cells = np.ascontiguousarray(np.flipud(grid))          # row 0 = top of screen
        shown, region, walls, bg = None, None, None, '#CDCDCD'
        if self.map_style == 'clean':
            try:
                built = build_display_map(cells, res)
                if built is not None:
                    rgb, region, walls = built
                    rgb = np.ascontiguousarray(rgb)
                    shown = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
                    bg = '#F1EFE8'
            except Exception as exc:  # noqa: BLE001
                self.log_cb(f'[지도] 표시용 지도 만들기 실패, 원본으로 표시: {exc}')
        if shown is None:
            img = np.where(cells < 0, 205, 255 - np.clip(cells, 0, 100) * 2.4).astype(np.uint8)
            img = np.ascontiguousarray(img)
            shown = QImage(img.data, w, h, w, QImage.Format_Grayscale8).copy()
            region, walls = cells >= 0, cells == 100
        if self.map_rotate == 'auto':
            angle = dominant_angle(walls)
        else:
            try:
                angle = float(self.map_rotate)
            except ValueError:
                angle = 0.0
        ys, xs = np.nonzero(region) if region.any() else (np.array([0, h]), np.array([0, w]))
        a = math.radians(angle)
        c0 = (w / 2.0, h / 2.0)
        u = (xs - c0[0]) * math.cos(a) + (ys - c0[1]) * math.sin(a)
        v = -(xs - c0[0]) * math.sin(a) + (ys - c0[1]) * math.cos(a)
        m = 6
        self.map = dict(img=shown, res=res, ox=ox, oy=oy, h=h, angle=a, c0=c0, bg=bg,
                        ub=(u.min() - m, u.max() + m, v.min() - m, v.max() + m), deg=angle)
        self.map_source = source

    def load_floorplan(self, path):
        """Hand-drawn plan for people. Display only: Nav2 still plans on the SLAM map.

        JSON (metres, map frame): walls [[x, y], ...] closed outline, glass [[[x, y], [x, y]], ...]
        outline edges drawn as windows, open [[[x, y], [x, y]], ...] outline edges with no wall (dashed),
        obstacles [{label, polygon}], tables [[[x, y] x4], ...],
        view_deg (screen rotation, same convention as map_rotate), margin_m (space around it, default 0.6).
        """
        try:
            with open(path, encoding='utf-8') as f:
                fp = json.load(f)
            walls = [tuple(map(float, pt)) for pt in fp['walls']]
            if len(walls) < 3:
                raise ValueError('walls needs 3+ points')
            res = 0.05                       # drawing units: 5 cm, like a typical map cell
            pts = walls + [tuple(pt) for t in fp.get('tables', []) for pt in t]
            if self.map_rotate == 'auto':
                deg = float(fp.get('view_deg', 0.0))
            else:
                try:
                    deg = float(self.map_rotate)
                except ValueError:
                    deg = 0.0
            a = math.radians(deg)
            px = np.array([x / res for x, _ in pts])
            py = np.array([-y / res for _, y in pts])
            c0 = (float(px.mean()), float(py.mean()))
            u = (px - c0[0]) * math.cos(a) + (py - c0[1]) * math.sin(a)
            v = -(px - c0[0]) * math.sin(a) + (py - c0[1]) * math.cos(a)
            m = float(fp.get('margin_m', 0.6)) / res     # space around the plan (zoom)
            self.fp_view = dict(img=None, res=res, ox=0.0, oy=0.0, h=0.0, angle=a, c0=c0, bg='#F1EFE8',
                                ub=(u.min() - m, u.max() + m, v.min() - m, v.max() + m), deg=deg)
            self.floorplan = fp
            self.log_cb(f'[지도] 평면도 읽음: {path} (화면 표시용, Nav2는 원래 지도 사용)')
        except Exception as exc:  # noqa: BLE001
            self.floorplan, self.fp_view = None, None
            self.log_cb(f'[지도] 평면도 읽기 실패, 지도 그림으로 표시: {exc}')

    def view(self):
        """Drawing frame for the map widget: the floor plan if loaded, else the map image."""
        return self.fp_view if self.floorplan else self.map

    def on_plan(self, msg, track=None):
        (track or self.main).plan = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    @staticmethod
    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_pose(self, msg, track=None):
        tr = track or self.main
        p = msg.pose.pose.position
        tr.amcl_pose = (p.x, p.y, self.yaw_of(msg.pose.pose.orientation))
        tr.amcl_time = time.monotonic()
        tr.amcl_ref_odom = tr.odom if tr.odom is not None and tr.amcl_time - tr.odom_time < 0.5 else None

    def on_odom(self, msg, track=None):
        tr = track or self.main
        p = msg.pose.pose.position
        tr.odom = (p.x, p.y, self.yaw_of(msg.pose.pose.orientation))
        tr.odom_time = time.monotonic()

    @staticmethod
    def compose(base, ref, cur):
        """base pose moved by the odometry change ref -> cur (both in the odom frame)."""
        dx, dy = cur[0] - ref[0], cur[1] - ref[1]
        c, s_ = math.cos(-ref[2]), math.sin(-ref[2])
        lx, ly = c * dx - s_ * dy, s_ * dx + c * dy            # motion in the robot frame at ref
        c, s_ = math.cos(base[2]), math.sin(base[2])
        yaw = base[2] + (cur[2] - ref[2])
        return (base[0] + c * lx - s_ * ly, base[1] + s_ * lx + c * ly, math.atan2(math.sin(yaw), math.cos(yaw)))

    def update_pose(self):
        """Pick the pose to show for every robot (called ~10 Hz).

        TF map->base_frame is smooth (AMCL correction + wheel odometry, tens of Hz).
        /amcl_pose is exact but only published after ~0.25 m / ~11 deg of motion,
        so it is the fallback. TF counts as live while its stamp keeps changing
        (checked on the local clock, so laptop/robot clock offset does not matter).
        Two robots: TF frames <robot>/base_footprint (frame_prefix); otherwise each robot's
        <ns>/amcl_pose is used.
        """
        now = time.monotonic()
        shown = self.fleet()['robots']          # robots the controller reports (1 with the 1-robot one)
        for tr in list(self.tracks.values()):
            if tr.name in shown:
                self._update_track_pose(tr, now, tr is self.main)
        self._learn_speed(now)
        self._learn_dock(now)

    def _update_track_pose(self, tr, now, first):
        if self.pose_mode != 'amcl':
            try:
                t = None
                if tr.base_frame != 'auto':
                    frames = [tr.base_frame.format(robot=tr.name)]
                else:
                    frames = [f'{tr.name}/base_footprint', f'{tr.name}/base_link']
                    if first:
                        frames += ['base_footprint', 'base_link']
                for fr in frames:
                    try:
                        t = self.tf_buffer.lookup_transform(self.map_frame, fr, Time())
                        if tr.base_frame == 'auto':
                            tr.base_frame = fr
                            self.log_cb(f'[위치] {tr.name} 로봇 프레임: {fr}')
                        break
                    except Exception:  # noqa: BLE001
                        if fr == frames[-1]:
                            raise
                stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
                if stamp != tr.tf_stamp:
                    tr.tf_stamp, tr.tf_fresh = stamp, now
                tr_ = t.transform.translation
                tr.tf_pose = (tr_.x, tr_.y, self.yaw_of(t.transform.rotation))
                tr.tf_error_logged = False
            except Exception as exc:  # noqa: BLE001
                tr.tf_pose = None
                if not tr.tf_error_logged and (first or tr.amcl_pose is None):
                    tr.tf_error_logged = True
                    shown = 'base_footprint/base_link' if tr.base_frame == 'auto' else tr.base_frame
                    topic = '/amcl_pose' if first else f'/{tr.name}/amcl_pose'
                    self.log_cb(f'[위치] {tr.name}: TF {self.map_frame}→{shown} 없음: '
                                f'{type(exc).__name__} - {topic}로 대체')
        if tr.tf_pose is not None and now - tr.tf_fresh < 1.0:
            tr.pose, tr.pose_source = tr.tf_pose, 'TF'
        elif tr.amcl_pose is not None and self.pose_mode != 'tf':
            if tr.amcl_ref_odom is not None and tr.odom is not None and now - tr.odom_time < 0.5:
                # amcl_pose only comes after ~0.25 m / ~11 deg: fill the gap with wheel odometry
                tr.pose, tr.pose_source = self.compose(tr.amcl_pose, tr.amcl_ref_odom, tr.odom), 'AMCL+odom'
            else:
                tr.pose, tr.pose_source = tr.amcl_pose, 'AMCL'
        elif tr.pose is not None:
            tr.pose_source = '끊김'
        self._smooth_view(tr, now)

    def _smooth_view(self, tr, now):
        """Pose drawn on the map: follows tr.pose with a short lag, ignores tiny jitter, snaps on jumps."""
        if tr.pose is None:
            tr.view_pose = None
            return
        last = getattr(tr, '_view_t', None)
        tr._view_t = now
        v = tr.view_pose
        if v is None or self.pose_smooth <= 0 or last is None:
            tr.view_pose = tr.pose
            return
        dx, dy = tr.pose[0] - v[0], tr.pose[1] - v[1]
        dyaw = math.atan2(math.sin(tr.pose[2] - v[2]), math.cos(tr.pose[2] - v[2]))
        if math.hypot(dx, dy) > 0.5:                       # relocalised / teleported: no slow glide
            tr.view_pose = tr.pose
            return
        if math.hypot(dx, dy) < 0.02 and abs(dyaw) < math.radians(1.0):
            return                                          # jitter: keep still
        k = 1.0 - math.exp(-max(0.0, now - last) / self.pose_smooth)
        tr.view_pose = (v[0] + k * dx, v[1] + k * dy, v[2] + k * dyaw)

    def _learn_speed(self, now):
        """Average Nav2 speed over each finished driving leg (stops included), all robots."""
        shown = self.fleet()['robots']
        for name, tr in self.tracks.items():
            if name not in shown:
                continue
            state = shown[name].get('state')
            if state != tr.leg_state:
                if tr.leg_state in MOVING_STATES and tr.leg_start is not None:
                    dur = (tr.leg_dock_at or now) - tr.leg_start   # Nav2 part only
                    if dur > 3.0 and tr.leg_dist > 0.3:
                        v = tr.leg_dist / dur
                        self.speed = min(1.0, max(0.03, 0.6 * self.speed + 0.4 * v))
                        self.speed_learned = True
                        who = f'{name} ' if len(self.tracks) > 1 else ''
                        self.log_cb(f'[속도] {who}이번 구간 {v:.2f} m/s → 평균 {self.speed:.2f} m/s')
                tr.leg_state, tr.leg_start, tr.leg_last, tr.leg_dist = state, None, None, 0.0
                tr.leg_dock_at = None
            if tr.leg_dock_at is None and tr.leg_start is not None and self.dock_live(now, name) is not None:
                tr.leg_dock_at = now                    # docking starts: stop the Nav2 clock
            if (state in MOVING_STATES and tr.pose is not None and tr.pose_source != '끊김'
                    and tr.leg_dock_at is None):
                if tr.leg_start is None:
                    tr.leg_start = now
                if tr.leg_last is not None:
                    d = math.hypot(tr.pose[0] - tr.leg_last[0], tr.pose[1] - tr.leg_last[1])
                    if d < 0.5:  # ignore relocalisation jumps
                        tr.leg_dist += d
                tr.leg_last = (tr.pose[0], tr.pose[1])

    def _accept_frame(self, tr):
        return time.monotonic() - tr.frame_time > 0.08  # ~12 fps is plenty

    def on_jpeg(self, msg, track=None):
        tr = track or self.main
        if self._accept_frame(tr):
            img = QImage.fromData(bytes(msg.data))
            if not img.isNull():
                tr.frame, tr.frame_time = img, time.monotonic()

    def on_raw(self, msg, track=None):
        tr = track or self.main
        if not self._accept_frame(tr):
            return
        fmt = {'rgb8': QImage.Format_RGB888, 'bgr8': QImage.Format_RGB888,
               'mono8': QImage.Format_Grayscale8}.get(msg.encoding)
        if fmt is None:
            return
        img = QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy()
        if msg.encoding == 'bgr8':
            img = img.rgbSwapped()
        tr.frame, tr.frame_time = img, time.monotonic()

    # -- services
    def service_name(self, name, robot=None):
        """Per-robot services (delivery/<robot>/loaded) with a 2-robot controller, else delivery/<name>."""
        if robot and self.fleet()['multi']:
            return f'delivery/{robot}/{name}'
        return f'delivery/{name}'

    def call(self, name, label, on_done=None, robot=None):
        srv = self.service_name(name, robot)
        if srv not in self.clients_:
            self.clients_[srv] = self.create_client(Trigger, srv)
        cli = self.clients_[srv]
        if not cli.service_is_ready():
            self.log_cb(f'[{label}] 서비스 {srv} 없음 - 컨트롤러 실행 확인')
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self._reply(f, label, on_done))

    def _reply(self, fut, label, on_done=None):
        try:
            r = fut.result()
            self.log_cb(f"[{label}] {'성공' if r.success else '거절'}: {r.message}")
            if on_done is not None:
                on_done(r)
        except Exception as exc:  # noqa: BLE001
            self.log_cb(f'[{label}] 호출 실패: {exc}')


_DOTS = {}


def dot_icon(color):
    """Small filled circle used as an attention mark on a tab."""
    if color not in _DOTS:
        pm = QPixmap(14, 14)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor(color))
        p.setPen(Qt.NoPen)
        p.drawEllipse(2, 2, 10, 10)
        p.end()
        _DOTS[color] = QIcon(pm)
    return _DOTS[color]


def card(title=None):
    f = QFrame()
    f.setProperty('card', True)
    lay = QVBoxLayout(f)
    lay.setContentsMargins(18, 16, 18, 16)
    lay.setSpacing(10)
    if title:
        t = QLabel(title)
        t.setProperty('role', 'cardTitle')
        lay.addWidget(t)
    return f, lay


def label(text='', role=None):
    lab = QLabel(text)
    if role:
        lab.setProperty('role', role)
    return lab


def pill(lab, text, fg, bg):
    lab.setText(text)
    lab.setStyleSheet(f'background:{bg}; color:{fg}; border-radius:12px;'
                      f'padding:2px 12px; font-size:12px; font-weight:600;')


POINT_NAMES = {'STORE': '픽업', 'HOME': '수납'}


class RequestPopup(QDialog):
    """Non-blocking popup for the loading/receiving person: a new request arrived.

    Shown only while the 적재 확인 · 수령 현황 tab is open (see MainWindow.on_tab_changed).
    """

    def __init__(self, window):
        super().__init__(window)
        self.win = window
        self.setWindowTitle('새 부품 요청')
        self.setModal(False)
        self.setMinimumWidth(480)
        self.setStyleSheet(QSS + 'QDialog { background: #FFFFFF; }')
        v = QVBoxLayout(self)
        v.setContentsMargins(24, 22, 24, 20)
        v.setSpacing(10)
        self.tag = QLabel()
        pill(self.tag, '● 새 요청', AMBER[2], AMBER[1])
        self.tag.setFixedHeight(26)
        v.addWidget(self.tag, 0, Qt.AlignLeft)
        self.head = QLabel()
        self.head.setWordWrap(True)
        self.head.setStyleSheet('font-size:20px; font-weight:700; color:#2C2C2A;')
        v.addWidget(self.head)
        box = QFrame()
        box.setStyleSheet(f'QFrame {{ background:{AMBER[1]}; border:1px solid {AMBER[0]}; border-radius:10px; }}'
                          'QLabel { border:none; background:transparent; }')
        bl = QVBoxLayout(box)
        bl.setContentsMargins(16, 12, 16, 12)
        bl.setSpacing(4)
        self.load = QLabel()
        self.load.setStyleSheet(f'font-size:22px; font-weight:800; color:{AMBER[2]};'
                                'min-height:34px; padding-bottom:2px;')
        bl.addWidget(self.load)
        self.how = QLabel()
        self.how.setWordWrap(True)
        self.how.setStyleSheet(f'font-size:13px; color:{AMBER[2]};')
        bl.addWidget(self.how)
        v.addWidget(box)
        self.sub = label('', 'muted')
        self.sub.setWordWrap(True)
        v.addWidget(self.sub)
        self.more = QListWidget()
        self.more.setMaximumHeight(120)
        self.more.setWordWrap(True)
        self.more.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.more.hide()
        v.addWidget(self.more)
        row = QHBoxLayout()
        row.addStretch()
        ok_btn = QPushButton('확인')
        ok_btn.setProperty('kind', 'confirm')
        ok_btn.setStyleSheet('font-size:14px; padding:12px 28px;')
        ok_btn.clicked.connect(self.close)
        row.addWidget(ok_btn)
        v.addLayout(row)
        self.entries = []

    def add(self, text, robot, when, short):
        stamp = time.strftime('%H:%M:%S')
        self.entries.insert(0, f'{stamp}  {short}')
        self.head.setText(text)
        self.load.setText(f'{robot}에 적재해 주세요')
        self.how.setText(f'{robot}{josa(robot)} 픽업대에 오면 부품을 싣고 [적재 완료]를 눌러 주세요.')
        self.sub.setText(f'{when}  ·  {stamp}')
        if len(self.entries) > 1:
            self.tag.setText(f'● 새 요청 {len(self.entries)}건')
            self.more.clear()
            self.more.addItems(self.entries[1:])
            self.more.show()
        else:
            pill(self.tag, '● 새 요청', AMBER[2], AMBER[1])
            self.more.hide()
        self.show()
        self.raise_()
        self.activateWindow()
        QApplication.beep()

    def closeEvent(self, event):
        self.entries = []
        super().closeEvent(event)


class MapWidget(QWidget):
    """Mini RViz: occupancy map, Nav2 plan, named points and the robot pose."""

    def __init__(self, node, window):
        super().__init__()
        self.node = node
        self.win = window
        self.setMinimumSize(300, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _label(self, key):
        n = self.node
        if key in n.stations:
            return n.station_labels[n.stations.index(key)]
        return POINT_NAMES.get(key, key)

    @staticmethod
    def draw_floorplan(p, fp, to_screen, px_per_m):
        def poly(points):
            path = QPainterPath(to_screen(*points[0]))
            for pt in points[1:]:
                path.lineTo(to_screen(*pt))
            path.closeSubpath()
            return path

        walls = [tuple(pt) for pt in fp['walls']]
        p.fillPath(poly(walls), QColor('#FFFFFF'))
        for t in fp.get('tables', []):
            p.setPen(QPen(QColor('#B4B2A9'), 1.5))
            p.setBrush(QColor('#EDE9E0'))
            p.drawPath(poly(t))
        wall_w = max(3.0, 0.12 * px_per_m)
        for ob in fp.get('obstacles', []):
            pts = ob.get('polygon', [])
            if len(pts) < 3:
                continue
            p.setPen(QPen(QColor('#444441'), wall_w, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.setBrush(QColor('#D3D1C7'))
            p.drawPath(poly(pts))
            box = poly(pts).boundingRect()
            f = QFont(p.font())
            f.setPointSize(8)
            f.setBold(True)
            p.setFont(f)
            text = ob.get('label', '장애물')
            if box.width() > p.fontMetrics().horizontalAdvance(text) + 2 * wall_w + 8 and box.height() > 30:
                p.setPen(QColor('#444441'))
                p.drawText(box.adjusted(0, box.height() * 0.25, 0, 0), Qt.AlignCenter, text)

        def same(a, b):
            return abs(a[0] - b[0]) < 1e-3 and abs(a[1] - b[1]) < 1e-3
        glass = [(tuple(g[0]), tuple(g[1])) for g in fp.get('glass', [])]
        opened = [(tuple(g[0]), tuple(g[1])) for g in fp.get('open', [])]
        p.setBrush(Qt.NoBrush)
        edges = list(zip(walls, walls[1:] + walls[:1]))
        for a, b in edges:
            if any((same(a, g0) and same(b, g1)) or (same(a, g1) and same(b, g0)) for g0, g1 in opened):
                p.setPen(QPen(QColor('#B4B2A9'), 1.5, Qt.DashLine))     # open side: room continues
                p.drawLine(to_screen(*a), to_screen(*b))
            elif any((same(a, g0) and same(b, g1)) or (same(a, g1) and same(b, g0)) for g0, g1 in glass):
                p.setPen(QPen(QColor('#85B7EB'), wall_w, Qt.SolidLine, Qt.FlatCap))
                p.drawLine(to_screen(*a), to_screen(*b))
                p.setPen(QPen(QColor('#E6F1FB'), max(1.0, wall_w / 3), Qt.SolidLine, Qt.FlatCap))
                p.drawLine(to_screen(*a), to_screen(*b))
            else:
                p.setPen(QPen(QColor('#444441'), wall_w, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
                p.drawLine(to_screen(*a), to_screen(*b))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        clip = QPainterPath()
        clip.addRoundedRect(r, 10, 10)
        m = self.node.view()
        p.fillPath(clip, QColor(m['bg'] if m else '#CDCDCD'))
        p.setClipPath(clip)
        if m is None:
            p.setPen(QColor('#5F5E5A'))
            p.drawText(r, Qt.AlignCenter, '지도 수신 대기 중\n(Nav2 map_server 실행 확인)')
            return

        # map image px -> rotate by the view angle -> fit the room into the widget
        ca, sa = math.cos(m['angle']), math.sin(m['angle'])
        c0x, c0y = m['c0']
        umin, umax, vmin, vmax = m['ub']
        scale = min((r.width() - 16) / max(1.0, umax - umin), (r.height() - 16) / max(1.0, vmax - vmin))
        uc, vc = (umin + umax) / 2, (vmin + vmax) / 2
        cx, cy = r.center().x(), r.center().y()
        T = QTransform(scale * ca, -scale * sa, scale * sa, scale * ca,
                       cx - scale * uc - scale * (ca * c0x + sa * c0y),
                       cy - scale * vc - scale * (-sa * c0x + ca * c0y))
        def to_screen(x, y):
            px = (x - m['ox']) / m['res']
            py = m['h'] - (y - m['oy']) / m['res']
            return T.map(QPointF(px, py))

        if self.node.floorplan:
            self.draw_floorplan(p, self.node.floorplan, to_screen, scale / m['res'])
        elif m.get('img') is not None:
            p.save()
            p.setTransform(T)
            p.drawImage(0, 0, m['img'])
            p.restore()

        fleet = self.node.fleet()
        robots = fleet['robots']
        targets = {rs.get('target') for rs in robots.values()}
        multi = len(robots) > 1

        # Nav2 global plans (one per robot)
        for name, rs in robots.items():
            tr = self.node.tracks.get(name)
            if tr is None or len(tr.plan) < 2 or rs.get('state') not in ('TO_STORE', 'TO_STATION', 'TO_HOME'):
                continue
            path = QPainterPath(to_screen(*tr.plan[0]))
            for pt in tr.plan[1:]:
                path.lineTo(to_screen(*pt))
            col = QColor(self.win.robot_colors.get(name, TEAL)[0])
            p.setPen(QPen(col, 2.5, Qt.DashLine, Qt.RoundCap))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)

        # named points
        f = QFont(self.font())
        f.setPointSize(9)
        f.setBold(True)
        p.setFont(f)
        placed = []                                     # label boxes drawn so far (no overlaps)
        for key, pose in self.node.points.items():
            c = to_screen(pose[0], pose[1])
            is_target = key in targets
            col = QColor(AMBER[0] if is_target else '#534AB7')
            p.setPen(QPen(QColor('white'), 2))
            p.setBrush(col)
            p.drawRoundedRect(QRectF(c.x() - 6, c.y() - 6, 12, 12), 3, 3)
            text = self._label(key)
            tw = p.fontMetrics().horizontalAdvance(text) + 12
            for dx, dy in ((-tw / 2, -30), (-tw / 2, 12), (10, -9), (-tw - 10, -9)):   # above, below, right, left
                box = QRectF(c.x() + dx, c.y() + dy, tw, 18)
                if not any(box.intersects(o) for o in placed):
                    break
            placed.append(box)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 255, 255, 230))
            p.drawRoundedRect(box, 9, 9)
            p.setPen(col.darker(140))
            p.drawText(box, Qt.AlignCenter, text)

        # robots
        drawn = 0
        sel = self.win.selected if multi else None
        for name in robots:
            tr = self.node.tracks.get(name)
            pose = (tr.view_pose or tr.pose) if tr else None
            if pose is None:
                continue
            drawn += 1
            c = to_screen(pose[0], pose[1])
            accent = QColor('#B4B2A9' if tr.pose_source == '끊김' else self.win.robot_colors.get(name, GRAY)[0])
            rad = max(8.0, 0.09 / m['res'] * scale)        # ~ Pinky radius, min 8 px
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(accent.red(), accent.green(), accent.blue(), 60))
            p.drawEllipse(c, rad + 7, rad + 7)
            if name == sel:                                 # robot picked in the status tab
                p.setPen(QPen(QColor('#2C2C2A'), 2, Qt.DashLine))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(c, rad + 11, rad + 11)
            p.setPen(QPen(QColor('white'), 2.5))
            p.setBrush(accent)
            p.drawEllipse(c, rad, rad)
            ahead = to_screen(pose[0] + 0.3 * math.cos(pose[2]), pose[1] + 0.3 * math.sin(pose[2]))
            dx, dy = ahead.x() - c.x(), ahead.y() - c.y()
            dn = math.hypot(dx, dy) or 1.0
            tip = QPointF(c.x() + dx / dn * (rad + 9), c.y() + dy / dn * (rad + 9))
            p.setPen(QPen(accent.darker(130), 3, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(c, tip)
            if multi:                                       # name tag under the robot
                tag = short_name(name)
                tw = p.fontMetrics().horizontalAdvance(tag) + 10
                box = QRectF(c.x() - tw / 2, c.y() + rad + 6, tw, 17)
                p.setPen(Qt.NoPen)
                p.setBrush(accent.darker(120))
                p.drawRoundedRect(box, 8, 8)
                p.setPen(QColor('white'))
                p.drawText(box, Qt.AlignCenter, tag)
        if not drawn:
            p.setPen(QColor('#5F5E5A'))
            p.drawText(r.adjusted(0, 0, 0, -8), Qt.AlignBottom | Qt.AlignHCenter,
                       '로봇 위치 없음 (RViz에서 초기 위치를 지정하세요)')


def short_name(name):
    """'pinky_2' -> 'P2' for small map tags."""
    m = re.search(r'(\d+)$', str(name))
    return f'P{m.group(1)}' if m else str(name)[:6]


class MainWindow(QWidget):
    TAB_REQ, TAB_CONF, TAB_STATUS = 0, 1, 2

    def __init__(self, node):
        super().__init__()
        self.node = node
        node.log_cb = self.log
        self.rviz = None
        self.setObjectName('root')
        self.setStyleSheet(QSS)
        self.setWindowTitle('Pinky 부품 배송 관제')
        self.resize(1360, 880)
        self.lbl = {}
        self.maps = []
        self.seen_ids = None
        self.pending_popups = []  # requests that arrived while another tab was open
        self.state_colors = GRAY
        self.robot_colors = {}    # robot -> (accent, soft, dark) of its state
        self.selected = node.robot_names[0]   # robot shown in the status tab
        self.notice_seq = 0
        self.prediction = ({}, [], [], [])    # last estimate_fleet result

        # ---------------- header (always visible)
        header = QHBoxLayout()
        tbox = QVBoxLayout()
        tbox.setSpacing(0)
        tbox.addWidget(label('Pinky 부품 배송 관제', 'title'))
        tbox.addWidget(label('픽업 → 작업대 배송 · 수납 위치 복귀', 'muted'))
        header.addLayout(tbox)
        header.addStretch()
        self.pill_robot = {}
        for name in node.robot_names:
            self.pill_robot[name] = QLabel()
        self.pill_state = self.pill_robot[node.robot_names[0]]
        self.pill_conn, self.lbl['nav'], self.lbl['mode'], self.lbl['armed'] = QLabel(), QLabel(), QLabel(), QLabel()
        for p_ in list(self.pill_robot.values()) + [self.pill_conn, self.lbl['nav'], self.lbl['mode'], self.lbl['armed']]:
            p_.setFixedHeight(26)
            header.addWidget(p_, 0, Qt.AlignVCenter)

        # ---------------- tabs
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(False)
        self.tabs.addTab(self._build_request_tab(), '')
        self.tabs.addTab(self._build_confirm_tab(), '')
        self.tabs.addTab(self._build_status_tab(), '')
        self.tabs.tabBar().setCursor(Qt.PointingHandCursor)
        self.tabs.currentChanged.connect(self.on_tab_changed)
        self.tabs.setIconSize(QSize(12, 12))

        # ---------------- log strip (always visible)
        logc, ll = card('로그')
        logc.setMaximumHeight(130)
        self.logw = QPlainTextEdit()
        self.logw.setReadOnly(True)
        ll.addWidget(self.logw)

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 16)
        root.setSpacing(12)
        root.addLayout(header)
        self.notice = QLabel()
        self.notice.setWordWrap(True)
        self.notice.hide()
        root.addWidget(self.notice)
        root.addWidget(self.tabs, 1)
        root.addWidget(logc)

        self.popup = RequestPopup(self)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(20)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(200)
        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self.tick_pose)
        self.pose_timer.start(100)
        self.last_frame_shown = (None, 0.0)
        self.refresh()
        if node.map_source == 'file':
            self.log('[지도] 저장된 지도 파일을 표시 중 (map_server의 /map이 오면 교체)')
        if getattr(node, 'fp_missing', False):
            self.log('[지도] 평면도 파일을 못 찾아 /map을 그대로 표시합니다 '
                     '(~/maps 또는 이 폴더의 maps/에 sesac_gangnam_2.floorplan.json 필요)')
        if node.floorplan:
            self.log(f'[지도] 평면도 "{node.floorplan.get("name", "")}" 표시 중 '
                     '(화면 표시용, Pinky는 원래 SLAM 지도로 주행)')

    def tick_pose(self):
        self.node.update_pose()
        for mw in self.maps:
            if mw.isVisible():
                mw.update()

    def map_card(self):
        c, cl = card('지도 · Pinky 실시간 위치')
        w = MapWidget(self.node, self)
        self.maps.append(w)
        cl.addWidget(w, 1)
        legend = '● Pinky (색 = 현재 상태, 회색 = 위치 끊김)   ■ 지점 (주황 = 목적지)   ┅ Nav2 경로'
        if self.node.floorplan:
            legend += '   ━ 유리창(파랑)   ▱ 장애물(회색)'
        cl.addWidget(label(legend, 'muted'))
        return c

    # ================================================================ tab 1
    def _build_request_tab(self):
        n = self.node
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.setSpacing(16)

        req, rl = card('작업대')
        self.req_btns = []
        self.station_panels = {}
        grid = QGridLayout()
        grid.setSpacing(12)
        for i, (st, lab) in enumerate(zip(n.stations, n.station_labels)):
            panel = QFrame()
            panel.setProperty('tile', True)
            pv = QVBoxLayout(panel)
            pv.setContentsMargins(12, 10, 12, 12)
            pv.setSpacing(8)
            top = QHBoxLayout()
            name = QLabel(lab)
            name.setStyleSheet('font-size:16px; font-weight:800;')
            top.addWidget(name)
            top.addStretch()
            badge = QLabel()
            badge.setFixedHeight(24)
            top.addWidget(badge)
            pv.addLayout(top)
            b = QPushButton('＋  부품 요청')
            b.setProperty('kind', 'primary')
            b.setStyleSheet('text-align:center; font-size:17px; padding:12px;')
            b.setMinimumHeight(52)
            b.setCursor(Qt.PointingHandCursor)
            b.setProperty('label', '＋  부품 요청')
            b.clicked.connect(lambda _, b=b, s=st, l=lab: self.on_request_click(b, s, l))
            self.req_btns.append(b)
            pv.addWidget(b)
            rb = QPushButton('수령 완료')
            rb.setProperty('kind', 'confirm')
            rb.setStyleSheet('font-size:15px; padding:10px;')
            rb.setMinimumHeight(42)
            rb.setCursor(Qt.PointingHandCursor)
            rb.clicked.connect(lambda _, s=st, l=lab: self.on_receive_click(s, l))
            pv.addWidget(rb)
            desc = label('', 'muted')
            desc.setWordWrap(True)
            pv.addWidget(desc)
            self.station_panels[st] = dict(panel=panel, badge=badge, req=b, recv=rb, desc=desc)
            grid.addWidget(panel, i // 2, i % 2)
        rl.addLayout(grid)
        self.btn_received = self.station_panels[n.stations[0]]['recv'] if n.stations else QPushButton()
        self.req_hint = label('', 'muted')
        self.req_hint.setWordWrap(True)
        rl.addWidget(self.req_hint)

        q, ql = card()
        qh = QHBoxLayout()
        self.lbl_queue = label('대기열', 'cardTitle')
        qh.addWidget(self.lbl_queue)
        qh.addStretch()
        ql.addLayout(qh)
        tiles = QHBoxLayout()
        tiles.setSpacing(10)
        self.eta_tiles = {}
        for name in n.robot_names:
            t = QFrame()
            t.setProperty('tile', True)
            tl = QVBoxLayout(t)
            tl.setContentsMargins(14, 10, 14, 12)
            tl.setSpacing(1)
            head = QHBoxLayout()
            head.setSpacing(8)
            who = QLabel(name)
            who.setFixedHeight(22)
            head.addWidget(who)
            cap = label('지금 처리 중', 'tileLabel')
            head.addWidget(cap)
            task = QLabel('-')
            task.setStyleSheet('font-size:15px; font-weight:700;')
            head.addWidget(task)
            head.addStretch()
            tl.addLayout(head)
            main = QLabel('-')
            main.setStyleSheet('font-size:28px; font-weight:800;')
            main.setMinimumHeight(40)
            tl.addWidget(main)
            sub = label('', 'muted')
            sub.setWordWrap(True)
            sub.setMinimumHeight(32)
            sub.setAlignment(Qt.AlignTop | Qt.AlignLeft)
            tl.addWidget(sub)
            self.eta_tiles[name] = {'frame': t, 'who': who, 'cap': cap, 'task': task, 'main': main, 'sub': sub}
            tiles.addWidget(t, 1)
        ql.addLayout(tiles)
        first = self.eta_tiles[n.robot_names[0]]
        self.lbl['current_req'], self.lbl['eta_main'], self.lbl['eta_sub'] = \
            first['task'], first['main'], first['sub']
        ql.addSpacing(4)
        ql.addWidget(label('다음 차례 · 예상 도착', 'tileLabel'))
        self.list_queue = QListWidget()
        self.list_queue.setMinimumHeight(44)
        ql.addWidget(self.list_queue, 1)
        self.lbl['eta_note'] = label('', 'muted')
        self.lbl['eta_note'].setWordWrap(True)
        ql.addWidget(self.lbl['eta_note'])

        col = QVBoxLayout()
        col.setSpacing(16)
        col.addWidget(req)
        col.addWidget(q, 1)
        lay.addLayout(col, 5)
        lay.addWidget(self.map_card(), 6)
        return page

    # ================================================================ tab 2
    def _status_box(self, title):
        f = QFrame()
        f.setProperty('tile', True)
        v = QVBoxLayout(f)
        v.setContentsMargins(16, 12, 16, 14)
        v.setSpacing(8)
        top = QHBoxLayout()
        top.addWidget(label(title, 'cardTitle'))
        top.addStretch()
        badge = QLabel()
        badge.setFixedHeight(26)
        top.addWidget(badge)
        v.addLayout(top)
        desc = label('', 'muted')
        desc.setWordWrap(True)
        v.addWidget(desc)
        v.addStretch()
        return f, v, badge, desc

    def _build_confirm_tab(self):
        n = self.node
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.setSpacing(16)

        self.guide = QLabel()
        self.guide.setWordWrap(True)
        self.guide.setMinimumHeight(56)
        lay.addWidget(self.guide)

        col = QVBoxLayout()
        col.setSpacing(12)
        self.robot_cards = {}
        for name in n.robot_names:
            c, cl = card()
            top_w = QWidget()
            top = QHBoxLayout(top_w)
            top.setContentsMargins(0, 0, 0, 0)
            who = QLabel(name)
            who.setFixedHeight(24)
            top.addWidget(who)
            task = label('', 'muted')
            top.addWidget(task)
            top.addStretch()
            cl.addWidget(top_w)
            row = QHBoxLayout()
            row.setSpacing(12)
            lf, lv, badge_load, desc_load = self._status_box('적재 (픽업 장소)')
            rf, rv, badge_recv, desc_recv = self._status_box('수령 (작업대)')
            btn_loaded = QPushButton('적재 완료')
            btn_loaded.setProperty('kind', 'confirm')
            btn_loaded.setStyleSheet('font-size:16px; padding:16px;')
            btn_loaded.setCursor(Qt.PointingHandCursor)
            btn_loaded.clicked.connect(lambda _, r=name: self.on_confirm('loaded', '적재 완료', r))
            lv.addWidget(btn_loaded)
            rv.addWidget(label('수령 완료는 작업대에서 요청한 사람이\n[작업 요청] 탭에서 누릅니다.', 'tileLabel'))
            row.addWidget(lf, 1)
            row.addWidget(rf, 1)
            cl.addLayout(row)
            batch_title = label('이번에 싣고 가는 작업', 'tileLabel')
            cl.addWidget(batch_title)
            batch = QListWidget()
            batch.setMaximumHeight(110)
            cl.addWidget(batch)
            self.robot_cards[name] = dict(card=c, top=top_w, who=who, task=task, badge_load=badge_load,
                                          badge_recv=badge_recv, desc_load=desc_load, desc_recv=desc_recv,
                                          btn_loaded=btn_loaded,
                                          batch=batch, batch_title=batch_title)
            col.addWidget(c, 1)
        first = self.robot_cards[n.robot_names[0]]
        self.btn_loaded = first['btn_loaded']
        self.badge_load, self.badge_recv = first['badge_load'], first['badge_recv']
        self.desc_load, self.desc_recv, self.list_batch = first['desc_load'], first['desc_recv'], first['batch']
        col.addStretch(0)
        body = QHBoxLayout()
        body.setSpacing(16)
        body.addLayout(col, 6)
        body.addWidget(self.map_card(), 5)
        lay.addLayout(body, 1)
        return page

    # ================================================================ tab 3
    def _build_status_tab(self):
        n = self.node
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.setSpacing(16)

        left = QVBoxLayout()
        left.setSpacing(14)
        self.sel_row = QWidget()
        sr = QHBoxLayout(self.sel_row)
        sr.setContentsMargins(0, 0, 0, 0)
        sr.setSpacing(8)
        self.sel_btns = {}
        for name in n.robot_names:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _, r=name: self.select_robot(r))
            self.sel_btns[name] = b
            sr.addWidget(b, 1)
        left.addWidget(self.sel_row)

        hero, hl = card()
        self.hero = hero
        self.lbl_state = QLabel('-')
        self.lbl_state.setStyleSheet('font-size:20px; font-weight:700;')
        self.lbl_state_sub = label('', 'muted')
        hl.addWidget(self.lbl_state)
        hl.addWidget(self.lbl_state_sub)
        steps = QHBoxLayout()
        steps.setSpacing(4)
        self.step_labels = []
        for i, (_, name) in enumerate(STEPS):
            s_ = QLabel(name)
            s_.setAlignment(Qt.AlignCenter)
            s_.setMinimumHeight(30)
            self.step_labels.append(s_)
            steps.addWidget(s_, 1)
            if i < len(STEPS) - 1:
                steps.addWidget(label('›', 'muted'))
        hl.addSpacing(4)
        hl.addLayout(steps)
        tiles = QGridLayout()
        tiles.setSpacing(8)
        for i, (key, text) in enumerate((('target', '목적지'), ('dist', '남은 거리'),
                                         ('timeout', '타임아웃까지'), ('pose', '위치 (x, y, θ)'),
                                         ('battery', '배터리'), ('cost', '배정 Cost'))):
            t = QFrame()
            t.setProperty('tile', True)
            tl_ = QVBoxLayout(t)
            tl_.setContentsMargins(12, 8, 12, 8)
            tl_.setSpacing(2)
            title = label(text, 'tileLabel')
            self.lbl[key + '_title'] = title
            tl_.addWidget(title)
            self.lbl[key] = label('-', 'tileValue')
            tl_.addWidget(self.lbl[key])
            tiles.addWidget(t, i // 2, i % 2)
        hl.addSpacing(4)
        hl.addLayout(tiles)
        left.addWidget(hero)

        rec, kl = card('오류 · 복구')
        self.lbl['error'] = QLabel()
        self.lbl['error'].setWordWrap(True)
        self.lbl['error'].setStyleSheet(
            'background:#FCEBEB; color:#791F1F; border-radius:10px; padding:10px 12px;')
        self.lbl['error'].hide()
        kl.addWidget(self.lbl['error'])
        self.lbl['current'] = label('', 'muted')
        kl.addWidget(self.lbl['current'])
        br = QHBoxLayout()
        self.btn_arm = QPushButton('시스템 활성화')
        self.btn_disarm = QPushButton('운행 중지 요청')
        self.btn_retry = QPushButton('↻  재시도')
        self.btn_reset = QPushButton('✕  현재 작업 취소')
        self.btn_reset.setProperty('kind', 'danger')
        btn_rviz = QPushButton('RViz 열기')
        self.btn_arm.clicked.connect(lambda: n.call('arm', 'FMS 활성화'))
        self.btn_disarm.clicked.connect(lambda: n.call('disarm', 'FMS 운행 중지'))
        self.btn_retry.clicked.connect(lambda: n.call('retry', f'{self.selected} 재시도', robot=self.selected))
        self.btn_reset.clicked.connect(self.confirm_reset)
        btn_rviz.clicked.connect(self.open_rviz)
        for b in (self.btn_arm, self.btn_disarm, self.btn_retry, self.btn_reset, btn_rviz):
            b.setCursor(Qt.PointingHandCursor)
            br.addWidget(b)
        kl.addLayout(br)
        self.btn_raw = QPushButton('상태 원본(JSON) 보기')
        self.btn_raw.setProperty('kind', 'link')
        self.btn_raw.setCursor(Qt.PointingHandCursor)
        self.btn_raw.clicked.connect(self.toggle_raw)
        kl.addWidget(self.btn_raw)
        self.raw = QPlainTextEdit()
        self.raw.setReadOnly(True)
        self.raw.setMaximumHeight(130)
        self.raw.hide()
        kl.addWidget(self.raw)
        left.addWidget(rec)
        left.addStretch()

        cam, caml = card()
        ch = QHBoxLayout()
        self.cam_title = label('카메라', 'cardTitle')
        ch.addWidget(self.cam_title)
        ch.addStretch()
        self.cam_live = QLabel()
        self.cam_live.setFixedHeight(24)
        ch.addWidget(self.cam_live)
        caml.addLayout(ch)
        self.cam = QLabel('영상 수신 없음\ncamera_topic 파라미터를 확인하세요')
        self.cam.setAlignment(Qt.AlignCenter)
        self.cam.setMinimumSize(320, 180)
        self.cam.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.cam.setStyleSheet('background:#2C2C2A; color:#B4B2A9; border-radius:10px;')
        caml.addWidget(self.cam, 1)
        self.cam_topic = label(n.camera_topic or '카메라 꺼짐', 'muted')
        caml.addWidget(self.cam_topic)

        right = QVBoxLayout()
        right.setSpacing(16)
        right.addWidget(cam, 1)
        right.addWidget(self.map_card(), 1)
        lay.addLayout(left, 5)
        lay.addLayout(right, 5)
        return page

    # ================================================================ actions
    def spin_ros(self):
        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=0)

    def log(self, text):
        self.logw.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    def show_notice(self, text, colors, seconds=10):
        self.notice_seq += 1
        seq = self.notice_seq
        self.notice.setText(text)
        self.notice.setStyleSheet(
            f'background:{colors[1]}; color:{colors[2]}; border:1px solid {colors[0]};'
            f'border-radius:12px; padding:12px 18px; font-size:15px; font-weight:600;')
        self.notice.show()
        QTimer.singleShot(seconds * 1000, lambda: seq == self.notice_seq and self.notice.hide())

    def select_robot(self, name):
        self.selected = name
        self.refresh()

    def on_confirm(self, name, lab, robot):
        multi = self.node.fleet()['multi']
        self.node.call(name, f'{robot} {lab}' if multi else lab, robot=robot)

    def _name(self, key):
        n = self.node
        return n.station_labels[n.stations.index(key)] if key in n.stations else (key or '작업대')

    def busy_text(self, st):
        """What Pinky is doing now, or None if it can leave right away."""
        state = st.get('state')
        cur = st.get('current') or {}
        dest = self._name(cur.get('station')) if isinstance(cur, dict) else '작업대'
        return {
            'TO_STORE': f'{dest} 배송을 위해 픽업 장소로 가는 중',
            'WAIT_PICKUP': f'{dest} 배송을 위해 픽업 차례를 기다리는 중',
            'DOCKING': f'{dest} 배송을 위해 픽업대 마커에 맞추는 중',
            'WAIT_LOAD': f'픽업 장소에서 {dest} 부품을 싣는 중',
            'TO_STATION': f'{dest} 쪽으로 배달하러 가는 중',
            'WAIT_RECEIVE': f'{dest}에서 부품 수령을 기다리는 중',
            'ERROR': '오류로 멈춰 있는 중',
        }.get(state)

    @staticmethod
    def dock_text(fb):
        d, a = fb.get('distance'), fb.get('angle_deg')
        if fb.get('visible') is False:
            return '마커 찾는 중'
        parts = []
        if isinstance(d, (int, float)):
            parts.append(f'{d:.2f} m')
        if isinstance(a, (int, float)):
            parts.append(f'{a:+.0f}°')
        return ', '.join(parts) or '정렬 중'

    def on_request_click(self, b, station, lab):
        """Send one request, then lock that button briefly (double-click / mis-click guard)."""
        lock = self.node.request_lock_sec
        if lock > 0:
            b.setEnabled(False)
            b.setText('요청됨 · 잠시 대기')
            QTimer.singleShot(int(lock * 1000), lambda: self.unlock_request(b))
        self.node.call(f'request_{station.lower()}', f'{lab} 요청',
                       lambda r: self.on_request_reply(lab, r, station))

    def robot_at_station(self, station, f=None):
        """(robot, status) of a Pinky waiting at that workstation for its parts to be taken, else (None, None)."""
        f = f or self.node.fleet()
        for name, rs in f['robots'].items():
            cur = rs.get('current')
            here = rs.get('target') == station or (isinstance(cur, dict) and cur.get('station') == station)
            if rs.get('state') == 'WAIT_RECEIVE' and here:
                return name, rs
        return None, None

    def on_receive_click(self, station, lab):
        """The requester at the workstation took the parts out."""
        f = self.node.fleet()
        robot, _ = self.robot_at_station(station, f)
        if robot is None:
            self.log(f'[{lab} 수령 완료] 이 작업대에서 수령을 기다리는 Pinky가 없습니다')
            return
        self.node.call('received', f'{lab} 수령 완료' + (f' ({robot})' if f['multi'] else ''), robot=robot)

    def unlock_request(self, b):
        b.setText(b.property('label'))
        b.setEnabled(True)

    def predict(self, queue):
        n = self.node
        f = n.fleet()
        return estimate_fleet(f['robots'], queue, n.poses(), n.points, n.speed, n.detour,
                              n.arrive_tol, n.dock, n.dock_lives())

    def on_request_reply(self, lab, r, station=None):
        if not r.success:
            self.show_notice(f'✕  {lab} 요청이 거절됐습니다 · {r.message}', RED)
            return
        f = self.node.fleet()
        st = f['raw']
        m = re.search(r'position (\d+)', r.message or '')
        pos = int(m.group(1)) if m else len(f['queue']) + 1
        if not f['multi']:
            busy = self.busy_text(st)
            if busy is None:
                how = '복귀를 멈추고 ' if st.get('state') == 'TO_HOME' else ''
                self.show_notice(f'✓  {lab} 요청 접수 · Pinky가 {how}곧 픽업 장소로 출발합니다.', TEAL)
            else:
                self.show_notice(
                    f'⏳  {lab} 요청 접수 · 현재 Pinky가 {busy}이라 대기해야 합니다.  '
                    f'(대기 순번 {pos}번)', AMBER, seconds=12)
            return
        queue = f['queue'][:max(0, pos - 1)] + [{'id': -1, 'station': station}]
        _, _, who, pick = self.predict(queue)
        robot = who[-1] if who else None
        free = [nm for nm, rs in f['robots'].items() if self.busy_text(rs) is None]
        if robot is not None and robot in free and pos == 1:
            how = '복귀를 멈추고 ' if f['robots'][robot].get('state') == 'TO_HOME' else ''
            self.show_notice(f'✓  {lab} 요청 접수 · {robot}{josa(robot)} {how}곧 픽업 장소로 출발합니다.', TEAL)
        else:
            why = '모든 Pinky가 배송 중이라 ' if not free else '앞선 요청이 있어 '
            when = (f'{robot}{josa(robot)} 약 {fmt_dur(pick[-1])} 뒤 픽업대에 옵니다.  '
                    if robot and pick and pick[-1] is not None else '')
            self.show_notice(f'⏳  {lab} 요청 접수 · {why}대기합니다. {when}(대기 순번 {pos}번)',
                             AMBER, seconds=12)

    def all_tasks(self, f):
        tasks = []
        for rs in f['robots'].values():
            tasks += [x for x in [rs.get('current')] + list(rs.get('batch') or [])]
        tasks += f['queue']
        out, seen = [], set()
        for x in tasks:
            if isinstance(x, dict) and 'id' in x and x['id'] not in seen:
                seen.add(x['id'])
                out.append(x)
        return out

    def detect_new_requests(self, f, live):
        """Pop up once per new task id seen in the status (from any GUI)."""
        if not live:
            self.seen_ids = None  # re-baseline after a reconnect (controller restart)
            return
        tasks = self.all_tasks(f)
        if self.seen_ids is None:
            self.seen_ids = {x['id'] for x in tasks}
            return
        for x in tasks:
            if x['id'] in self.seen_ids:
                continue
            self.seen_ids.add(x['id'])
            robot, when = self.task_robot_when(x, f)
            self.log(f'[새 요청] {self._name(x.get("station"))} → {robot} (#{x["id"]}, {when})')
            if not self.node.popup_on_request:
                continue
            if self.tabs.currentIndex() == self.TAB_CONF:
                self.show_request_popup(x)
            else:
                self.pending_popups.append(x)  # shown when the confirm tab is opened

    def show_request_popup(self, task, st=None):
        f = self.node.fleet()
        robot, when = self.task_robot_when(task, f)
        where = self._name(task.get('station'))
        self.popup.add(f'{where}에서 부품 요청이 들어왔습니다.', robot, when, f'{where} → {robot}  ({when})')

    def on_tab_changed(self, index):
        """Load popups belong to the 적재 확인 · 수령 현황 tab only."""
        if index != self.TAB_CONF:
            self.popup.close()
            return
        f = self.node.fleet()
        waiting = {q.get('id') for q in f['queue'] if isinstance(q, dict)}
        for rs in f['robots'].values():
            cur = rs.get('current')
            if isinstance(cur, dict) and rs.get('state') in ('TO_STORE', 'WAIT_PICKUP', 'DOCKING', 'WAIT_LOAD'):
                waiting.add(cur.get('id'))   # not loaded yet
        pending, self.pending_popups = self.pending_popups, []
        for task in pending:
            if task.get('id') in waiting:
                self.show_request_popup(task)

    def robot_of(self, task, st=None):
        """Robot that carries this task: task['robot'] > the robot running it > prediction > param."""
        if isinstance(task, dict) and task.get('robot'):
            return str(task['robot'])
        f = self.node.fleet()
        if isinstance(task, dict):
            for name, rs in f['robots'].items():
                cur = rs.get('current')
                if isinstance(cur, dict) and cur.get('id') == task.get('id'):
                    return name
                if any(isinstance(b, dict) and b.get('id') == task.get('id') for b in rs.get('batch') or []):
                    return name
            ids = [q.get('id') if isinstance(q, dict) else None for q in f['queue']]
            if task.get('id') in ids:
                who = self.predict(f['queue'])[2]
                guess = who[ids.index(task.get('id'))]
                if guess:
                    return guess
        if not f['multi']:
            return str((st or f['raw']).get('robot') or self.node.robot_name)
        return next(iter(f['robots']), self.node.robot_name)

    def task_robot_when(self, task, f):
        robot = self.robot_of(task)
        ids = [q.get('id') if isinstance(q, dict) else None for q in f['queue']]
        pos = ids.index(task.get('id')) + 1 if task.get('id') in ids else None
        if not f['multi']:
            return robot, self.when_text(task, pos, f['raw'], robot)
        rs = f['robots'].get(robot) or {}
        ig = josa(robot)
        if pos is None:
            return robot, {'TO_STORE': f'{robot}{ig} 지금 픽업대로 오는 중입니다',
                           'WAIT_PICKUP': f'{robot}{ig} 픽업대 근처에서 차례를 기다리는 중입니다',
                           'DOCKING': f'{robot}{ig} 픽업대 앞에서 자리를 맞추는 중입니다',
                           'WAIT_LOAD': f'{robot}{ig} 픽업대에 도착해 있습니다'}.get(
                               rs.get('state'), f'{robot}{ig} 곧 픽업대로 옵니다')
        pick = self.predict(f['queue'])[3]
        t = pick[pos - 1] if pos - 1 < len(pick) else None
        text = f'{robot}{ig} 약 {fmt_dur(t)} 뒤 픽업대에 옵니다' if t is not None else \
            f'{robot}{ig} 앞선 배송을 마친 뒤 픽업대로 옵니다'
        return robot, f'{text} · 대기 순번 {pos}번'

    def when_text(self, task, pos, st, robot):
        """One robot: when it will be at the pickup for this task."""
        ig, state, cur = josa(robot), st.get('state'), st.get('current')
        if pos is None:  # already the current task
            return {'TO_STORE': f'{robot}{ig} 지금 픽업대로 오는 중입니다',
                    'DOCKING': f'{robot}{ig} 픽업대 앞에서 자리를 맞추는 중입니다',
                    'WAIT_LOAD': f'{robot}{ig} 픽업대에 도착해 있습니다'}.get(
                        state, f'{robot}{ig} 곧 픽업대로 옵니다')
        ahead = pos - 1 + (1 if cur else 0)
        if ahead == 0:
            text = f'{robot}{ig} 곧 픽업대로 출발합니다'
        elif ahead == 1 and isinstance(cur, dict):
            text = f'{robot}{ig} {self._name(cur.get("station"))} 배송을 마친 뒤 픽업대로 옵니다'
        else:
            text = f'{robot}{ig} 앞선 배송 {ahead}건을 마친 뒤 픽업대로 옵니다'
        return f'{text} · 대기 순번 {pos}번'

    def toggle_raw(self):
        show = not self.raw.isVisible()
        self.raw.setVisible(show)
        self.btn_raw.setText('상태 원본(JSON) 숨기기' if show else '상태 원본(JSON) 보기')

    def confirm_reset(self):
        multi = self.node.fleet()['multi']
        who = f'{self.selected}의 ' if multi else ''
        ans = QMessageBox.question(
            self, '현재 작업 취소',
            f'{who}현재 작업을 버리고 대기 상태로 돌아갑니다.\n'
            '로봇에 실린 부품은 사람이 직접 정리해야 합니다. 계속할까요?')
        if ans == QMessageBox.Yes:
            self.node.call('reset', f'{self.selected} reset' if multi else 'reset', robot=self.selected)

    def open_rviz(self):
        if self.rviz is not None and self.rviz.state() != QProcess.NotRunning:
            self.log('RViz가 이미 실행 중입니다.')
            return
        args = ['-d', self.node.rviz_config] if self.node.rviz_config else []
        self.rviz = QProcess(self)
        self.rviz.start('rviz2', args)
        self.log('RViz 실행' + (f' ({self.node.rviz_config})' if args else ''))

    # ================================================================ refresh
    @staticmethod
    def phase(state, target, stations):
        """(load, receive) progress for the current trip: 'none' | 'todo' | 'now' | 'done'."""
        if state in ('TO_STORE', 'DOCKING', 'WAIT_PICKUP'):
            return 'todo', 'todo'
        if state == 'WAIT_LOAD':
            return 'now', 'todo'
        if state == 'TO_STATION':
            return 'done', 'todo'
        if state == 'WAIT_RECEIVE':
            return 'done', 'now'
        if state == 'ERROR' and target:
            return ('todo', 'todo') if target == 'STORE' else \
                (('done', 'todo') if target in stations else ('none', 'none'))
        return 'none', 'none'

    def eta_text(self, rs, eta, dock_fb):
        """(big, sub, color) for one robot's current delivery."""
        state, cur = rs.get('state'), rs.get('current')
        dest = self._name(cur.get('station')) if isinstance(cur, dict) else ''
        if not cur:
            return '-', ('수납 위치로 복귀 중' if state == 'TO_HOME' else '진행 중인 배송 없음'), GRAY[2]
        if state == 'WAIT_RECEIVE':
            return '도착', f'{dest}에서 수령을 기다리는 중', TEAL[2]
        if state == 'ERROR':
            return '멈춤', '오류로 멈춤 · 시간 계산 중지', RED[2]
        if state == 'WAIT_LOAD':
            return fmt_left(eta), f'{dest} 부품 · 픽업대에서 적재 중 (시간 멈춤)', AMBER[2]
        if state == 'WAIT_PICKUP':
            return fmt_left(eta), f'{dest} 부품 · 다른 Pinky가 적재 중이라 픽업 차례 대기', '#3C3489'
        if state in ('TO_STORE', 'DOCKING') and dock_fb is not None:
            return fmt_left(eta), f'{dest} 부품 · 픽업대 마커에 맞추는 중 ({self.dock_text(dock_fb)})', TEAL[2]
        if state == 'DOCKING':
            return fmt_left(eta), f'{dest} 부품 · 픽업대 마커에 맞추는 중', TEAL[2]
        if state == 'TO_STORE':
            d = rs.get('distance_remaining')
            left = f' (남은 거리 {d:.1f} m)' if isinstance(d, (int, float)) else ''
            return fmt_left(eta), f'{dest} 부품 · 픽업대로 이동 중{left}', TEAL[2]
        return fmt_left(eta), f'{dest} 쪽으로 배달 중', TEAL[2]

    def refresh(self):
        n = self.node
        f = n.fleet()
        st = f['raw']
        robots, queue, multi = f['robots'], f['queue'], f['multi']
        stale = n.status is None or time.monotonic() - n.status_time > STALE_SEC
        live = not stale
        if self.selected not in robots:
            self.selected = next(iter(robots))

        # per-robot colours + header pills
        for name in n.robot_names:
            pl = self.pill_robot[name]
            rs = robots.get(name)
            if rs is None:
                pl.hide()
                continue
            pl.show()
            if stale:
                text, accent, soft, dark = '컨트롤러 응답 없음', *RED
            else:
                text, accent, soft, dark = STATE_STYLE.get(rs.get('state'), (rs.get('state', '-'), *GRAY))
            self.robot_colors[name] = (accent, soft, dark)
            pill(pl, f'{name if multi else self.robot_of(None, st)} · {text.split(" · ")[0]}', dark, soft)
        for name in robots:
            if name not in self.robot_colors:
                self.robot_colors[name] = GRAY
        self.state_colors = self.robot_colors.get(self.selected, GRAY)
        if stale:
            pill(self.pill_conn, '● 컨트롤러 끊김', RED[2], RED[1])
        else:
            pill(self.pill_conn, '● 컨트롤러 연결', TEAL[2], TEAL[1])
        navs = {nm: rs.get('nav_ready') for nm, rs in robots.items()}
        if all(v is None for v in navs.values()):
            pill(self.lbl['nav'], 'Nav2 -', GRAY[2], GRAY[1])
        elif all(navs.values()):
            pill(self.lbl['nav'], 'Nav2 준비됨', TEAL[2], TEAL[1])
        else:
            bad = [nm for nm, v in navs.items() if not v]
            pill(self.lbl['nav'], 'Nav2 대기 중' + (f' ({", ".join(bad)})' if multi else ''), AMBER[2], AMBER[1])
        dry = f['dry_run']
        pill(self.lbl['mode'], '모드 -' if dry is None else ('모의 실행' if dry else '실물 주행'),
             *((AMBER[2], AMBER[1]) if dry else (GRAY[2], GRAY[1])))
        armed = bool(st.get('armed'))
        pill(self.lbl['armed'], 'FMS 활성' if armed else 'FMS 정지',
             *((TEAL[2], TEAL[1]) if armed else (AMBER[2], AMBER[1])))
        self.btn_arm.setEnabled(live and not armed)
        self.btn_disarm.setEnabled(live and armed)

        # tab titles with attention dots
        states = {nm: rs.get('state') for nm, rs in robots.items()}
        waiting = live and any(s_ == 'WAIT_LOAD' for s_ in states.values())
        receiving = live and any(s_ == 'WAIT_RECEIVE' for s_ in states.values())
        error = stale or any(s_ == 'ERROR' for s_ in states.values())
        self.tabs.setTabText(self.TAB_REQ, f'작업 요청  ·  {len(queue)}')
        self.tabs.setTabText(self.TAB_CONF, '적재 확인 · 수령 현황')
        self.tabs.setTabText(self.TAB_STATUS, 'Pinky 현재 상태')
        self.tabs.setTabIcon(self.TAB_REQ, dot_icon(AMBER[0]) if receiving else QIcon())
        self.tabs.setTabIcon(self.TAB_CONF, dot_icon(AMBER[0]) if waiting else QIcon())
        self.tabs.setTabIcon(self.TAB_STATUS, dot_icon(RED[0]) if error else QIcon())

        # ---- tab 1
        self.detect_new_requests(f, live)
        lives = n.dock_lives() if live else {}
        if live:
            cur_eta, q_etas, q_who, _ = self.predict(queue)
        else:
            cur_eta, q_etas, q_who = {}, [None] * len(queue), [None] * len(queue)
        hint = ('[부품 요청]은 누를 때마다 한 건씩 대기열에 추가됩니다 (오클릭 방지로 2초 잠김). '
                '[수령 완료]는 Pinky가 그 작업대에 도착했을 때만 켜집니다.' +
                (' 요청은 비어 있는 Pinky 중 픽업대에 먼저 갈 수 있는 쪽이 맡습니다.' if multi else ''))
        self.req_hint.setText('요청은 누를 때마다 1건 (2초 잠김) · 수령 완료는 Pinky가 도착하면 켜짐')
        self.req_hint.setToolTip(hint)
        for stn, sp in self.station_panels.items():
            robot, rs_ = self.robot_at_station(stn, f) if live else (None, None)
            coming = [(nm, rs__) for nm, rs__ in robots.items() if live and rs__.get('state') == 'TO_STATION'
                      and rs__.get('target') == stn]
            sp['recv'].setEnabled(robot is not None)
            if robot is not None:
                who = robot if multi else self.robot_of(rs_.get('current'), st)
                pill(sp['badge'], '수령 대기', 'white', AMBER[0])
                sp['desc'].setText(f'{who}{josa(who)} 도착했습니다 · 부품을 꺼낸 뒤 [수령 완료]를 눌러 주세요')
                sp['desc'].setStyleSheet(f'color:{AMBER[2]}; font-size:12px; font-weight:700;')
                sp['panel'].setStyleSheet(f'QFrame[tile="true"] {{ background:{AMBER[1]}; border:2px solid {AMBER[0]};'
                                          ' border-radius:10px; }')
            else:
                if coming:
                    nm = coming[0][0] if multi else self.robot_of(coming[0][1].get('current'), st)
                    e = cur_eta.get(coming[0][0])
                    pill(sp['badge'], '배달 중', TEAL[2], TEAL[1])
                    sp['desc'].setText(f'{nm}{josa(nm)} 오는 중' + (f' · 약 {fmt_left(e)}' if e is not None else ''))
                else:
                    pill(sp['badge'], '-', GRAY[2], GRAY[1])
                    sp['desc'].setText('Pinky가 도착하면 [수령 완료]가 켜집니다')
                sp['desc'].setStyleSheet('color:#888780; font-size:12px;')
                sp['panel'].setStyleSheet('')
        self.lbl_queue.setText(f'도착 예정 · 대기 {len(queue)}건')
        for name, tile in self.eta_tiles.items():
            rs = robots.get(name)
            tile['frame'].setVisible(rs is not None)
            if rs is None:
                continue
            tile['who'].setVisible(multi)
            tile['cap'].setVisible(not multi)
            if multi:
                acc, soft, dark = self.robot_colors[name]
                pill(tile['who'], name, dark, soft)
            cur = rs.get('current')
            tile['task'].setText(fmt_task(cur) if cur else '없음')
            if not live:
                big, sub, col = '-', '컨트롤러 연결 끊김 · 계산 중지', RED[2]
            else:
                big, sub, col = self.eta_text(rs, cur_eta.get(name), lives.get(name))
            tile['main'].setText(big)
            tile['main'].setStyleSheet(f'font-size:{28 if multi else 30}px; font-weight:800; color:{col};')
            tile['sub'].setText(sub)
        rows = []
        for i, (x, e, who) in enumerate(zip(queue, q_etas, q_who)):
            line = f'{i + 1}   {fmt_task(x)}'
            if multi and who:
                line += f'   → {who} 예정'
            if e is not None:
                line += f'      약 {fmt_left(e)}'
            rows.append(line)
        self._fill(self.list_queue, rows or ['대기 중인 요청 없음'])
        self.lbl['eta_note'].setText(
            f'평균 속도 {n.speed:.2f} m/s ({"실측" if n.speed_learned else "설정값"}) · '
            f'마커 정렬 {n.dock["speed"]:.2f} m/s ({"실측" if n.dock_learned else "설정값"}) · '
            '적재·수령 대기 중엔 시간 멈춤')
        self.lbl['eta_note'].setToolTip(
            '이동·정렬 시간만 계산하며, 적재·수령을 기다리는 동안은 시간이 멈춥니다.' +
            (' 픽업대는 한 번에 한 대: 겹치면 늦게 온 Pinky가 근처에서 기다립니다.' if multi else ''))

        # ---- tab 2
        badge = {'none': ('-', GRAY[2], GRAY[1]), 'todo': ('미완료', AMBER[2], AMBER[1]),
                 'now': ('확인 필요', 'white', AMBER[0]), 'done': ('완료', TEAL[2], TEAL[1])}
        guides = []
        for name, rc in self.robot_cards.items():
            rs = robots.get(name)
            rc['card'].setVisible(rs is not None)
            if rs is None:
                continue
            state, cur = rs.get('state'), rs.get('current')
            r = name if multi else self.robot_of(cur, st)
            rc['top'].setVisible(multi)
            if multi:
                acc, soft, dark = self.robot_colors[name]
                pill(rc['who'], name, dark, soft)
                rc['task'].setText(f'{fmt_task(cur)}  ·  {STATE_STYLE.get(state, (state,))[0].split(" · ")[0]}'
                                   if cur else STATE_STYLE.get(state, (state or '-',))[0].split(' · ')[0])
            else:
                rc['task'].setText('')
            load, recv = self.phase(state, rs.get('target'), n.stations) if live else ('none', 'none')
            for b, ph in ((rc['badge_load'], load), (rc['badge_recv'], recv)):
                t, fg, bg = badge[ph]
                pill(b, t, fg, bg)
            rc['desc_load'].setText({
                'none': '진행 중인 배송이 없습니다.',
                'todo': ('다른 Pinky가 적재 중이라 픽업 차례를 기다리는 중입니다.' if state == 'WAIT_PICKUP' else
                         '로봇이 픽업 장소로 가는 중입니다. 도착하면 버튼이 켜집니다.'),
                'now': f'부품을 {r}에 올린 뒤 [적재 완료]를 눌러 주세요.',
                'done': '적재가 끝났습니다.'}[load])
            rc['desc_recv'].setText({
                'none': '진행 중인 배송이 없습니다.',
                'todo': '작업대에 도착하면 요청한 사람이 확인합니다.',
                'now': f'{r}{josa(r)} {self._name(rs.get("target"))}에 도착해 수령을 기다리는 중입니다.',
                'done': '수령이 끝났습니다.'}[recv])
            rc['btn_loaded'].setEnabled(live and state == 'WAIT_LOAD')
            batch = rs.get('batch') or ([cur] if cur else [])
            rows = []
            for x in batch:
                here = len(batch) == 1 or (isinstance(x, dict) and x.get('station') == rs.get('target'))
                rr = recv if here else ('todo' if load != 'none' else 'none')
                rows.append(f'{fmt_task(x)}      적재 {badge[load][0]}   ·   수령 {badge[rr][0]}')
            self._fill(rc['batch'], rows or ['싣고 가는 작업 없음'])
            show_batch = not multi or len(batch) > 1
            rc['batch'].setVisible(show_batch)
            rc['batch_title'].setVisible(show_batch)
            if state == 'WAIT_LOAD':
                guides.append((f'{r}{josa(r)} 픽업 장소에 도착했습니다  ·  부품을 {r}에 올리고 [적재 완료]를 눌러 주세요',
                               AMBER, 0))
            elif state == 'WAIT_RECEIVE':
                guides.append((f'{r}{josa(r)} {self._name(rs.get("target"))}에 도착했습니다  ·  '
                               '작업대에서 부품을 꺼내고 [수령 완료]를 누르면 끝납니다', TEAL, 2))
            elif state == 'ERROR':
                guides.append((f'{r + " " if multi else ""}오류로 멈춰 있습니다  ·  '
                               '[Pinky 현재 상태] 탭에서 재시도/취소하세요', RED, 1))
            elif state in ('TO_STORE', 'TO_STATION', 'TO_HOME', 'WAIT_PICKUP', 'DOCKING'):
                txt = STATE_STYLE.get(state, (state,))[0]
                guides.append((f'{r + " · " if multi else ""}{txt}  ·  도착하면 해당 버튼이 켜집니다', TEAL, 2))
        if not live:
            gtxt, gc = '컨트롤러와 연결되지 않아 확인 버튼을 잠갔습니다.', RED
        elif guides:
            guides.sort(key=lambda g_: g_[2])
            top = [g_ for g_ in guides if g_[2] == guides[0][2]]
            gtxt, gc = '\n'.join(g_[0] for g_ in top), top[0][1]
        else:
            gtxt, gc = '대기 중  ·  [작업 요청] 탭에서 요청을 넣으면 배송이 시작됩니다', GRAY
        self.guide.setText(gtxt)
        self.guide.setStyleSheet(f'background:{gc[1]}; color:{gc[2]}; border-radius:12px;'
                                 f'padding:14px 18px; font-size:15px; font-weight:600;')

        # ---- tab 3 (selected robot)
        sel = self.selected
        rs = robots.get(sel) or {}
        g = rs.get
        state = g('state', '-')
        self.sel_row.setVisible(multi)
        for name, b in self.sel_btns.items():
            b.setVisible(name in robots)
            b.setChecked(name == sel)
            if name in robots:
                acc, soft, dark = self.robot_colors[name]
                txt = STATE_STYLE.get(robots[name].get('state'), (robots[name].get('state') or '-',))[0]
                b.setText(f'{name}  ·  {txt.split(" · ")[0]}')
                b.setStyleSheet(f'background:{soft}; color:{dark}; border:2px solid {acc}; font-weight:700;'
                                if name == sel else f'color:{dark};')
        if stale:
            text, accent, soft, dark = '컨트롤러 응답 없음', *RED
            sub = '컨트롤러(fms_controller.py)가 실행 중인지, 같은 ROS_DOMAIN_ID인지 확인하세요'
        else:
            text, accent, soft, dark = STATE_STYLE.get(state, (state, *GRAY))
            sub = f'상태 코드 {state}'
            if g('target') and state in ('TO_STORE', 'TO_STATION', 'TO_HOME', 'ERROR'):
                sub += f'  ·  목적지 {g("target")}'
        self.lbl_state.setText(f'{sel}  ·  {text}' if multi else text)
        self.lbl_state.setStyleSheet(f'font-size:20px; font-weight:700; color:{dark};')
        self.lbl_state_sub.setText(sub)
        self.hero.setStyleSheet(
            f'QFrame[card="true"] {{ background:{soft}; border:1px solid {accent};'
            f' border-left:6px solid {accent}; border-radius:14px; }}')
        step_of = {'DOCKING': 'TO_STORE', 'WAIT_PICKUP': 'TO_STORE',
                   'WAIT_TRAFFIC': g('move_state') or 'TO_STORE'}
        idx = next((i for i, (s_, _) in enumerate(STEPS) if s_ == step_of.get(state, state)), None)
        for i, lab in enumerate(self.step_labels):
            if idx is not None and i == idx:
                css = f'background:{accent}; color:white; font-weight:700;'
            elif idx is not None and i < idx:
                css = f'background:white; color:{dark}; border:1px solid {accent};'
            else:
                css = 'background:rgba(255,255,255,0.6); color:#B4B2A9; border:1px solid #E5E3DC;'
            lab.setStyleSheet(css + 'border-radius:8px; padding:4px; font-size:12px;')
        self.btn_retry.setEnabled(live and state == 'ERROR')
        self.btn_reset.setEnabled(live)
        self.btn_reset.setText(f'✕  {sel} 작업 취소' if multi else '✕  현재 작업 취소')
        tr = n.tracks.get(sel) or n.main
        self.lbl['target'].setText(str(g('target') or '-'))
        d, fb = g('distance_remaining'), (n.dock_live(name=sel) if live else None)
        if fb is not None:
            self.lbl['dist'].setText(f'마커 {self.dock_text(fb)}')
        else:
            self.lbl['dist'].setText(f'{d:.2f} m' if isinstance(d, (int, float)) else '-')
        t = g('timeout_left')
        self.lbl['timeout'].setText(f'{t:.0f} s' if isinstance(t, (int, float)) else '-')
        self.lbl['pose'].setText('-' if tr.pose is None else
                                 f'{tr.pose[0]:.2f}, {tr.pose[1]:.2f}, {math.degrees(tr.pose[2]):.0f}°')
        self.lbl['pose_title'].setText(
            f'위치 (x, y, θ) · {tr.pose_source}' if tr.pose_source else '위치 (x, y, θ)')
        battery = g('battery_percent')
        self.lbl['battery'].setText(f'{battery * 100:.0f}%' if isinstance(battery, (int, float)) else '데이터 없음')
        self.lbl['battery'].setStyleSheet(
            f'color:{RED[2] if g("low_battery") else TEAL[2]}; font-size:18px; font-weight:700;')
        cost = g('allocation_cost')
        source = g('distance_source') or '-'
        self.lbl['cost'].setText(f'{cost:.2f} · {source}' if isinstance(cost, (int, float)) else f'- · {source}')
        cur = g('current')
        self.lbl['current'].setText(f'현재 작업: {fmt_task(cur)}' if cur else '현재 작업 없음')
        err = g('error')
        self.lbl['error'].setText(f'⚠  {err}' if err else '')
        self.lbl['error'].setVisible(bool(err))
        raw = json.dumps(st, ensure_ascii=False, indent=1) if st else '-'
        if raw != self.raw.toPlainText():
            self.raw.setPlainText(raw)

        self.cam_title.setText(f'카메라 · {sel}' if multi else '카메라')
        fresh = tr.frame is not None and time.monotonic() - tr.frame_time <= 3.0
        if fresh:
            pill(self.cam_live, '● LIVE', RED[2], RED[1])
        else:
            pill(self.cam_live, '신호 없음', GRAY[2], GRAY[1])
        if tr.frame is not None and (sel, tr.frame_time) != self.last_frame_shown:
            self.last_frame_shown = (sel, tr.frame_time)
            self.cam.setPixmap(QPixmap.fromImage(tr.frame).scaled(
                self.cam.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        elif tr.frame is None and self.last_frame_shown[0] != sel:
            self.last_frame_shown = (sel, 0.0)
            self.cam.setText('영상 수신 없음\ncamera_topic 파라미터를 확인하세요')
        elif tr.frame is not None and not fresh:
            self.cam.setText('영상 끊김 · 3초 이상 수신 없음')

    @staticmethod
    def _fill(widget, items):
        if [widget.item(i).text() for i in range(widget.count())] != items:
            widget.clear()
            widget.addItems(items)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = GuiNode()
    win = MainWindow(node)
    win.show()
    try:
        code = app.exec() if QT_API == 'PyQt6' else app.exec_()
    finally:
        if win.rviz is not None:
            win.rviz.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
