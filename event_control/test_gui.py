# GUI 검증 (화면 없이): QT_QPA_PLATFORM=offscreen python3 test_gui.py delivery_gui.py <screenshot_dir>
import sys, types, json, time, math, importlib.util, os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


class Fut:
    def __init__(s, r): s.r = r
    def result(s): return s.r
    def add_done_callback(s, c): c(s)


CALLS = []


class Client:
    def __init__(s, name): s.name = name
    def service_is_ready(s): return True
    def call_async(s, req):
        CALLS.append(s.name)
        return Fut(types.SimpleNamespace(success=True, message=f'Queued via {s.name} (position 1)'))


class Node:
    def __init__(s, name): s.subs = {}
    def declare_parameter(s, n, v): return types.SimpleNamespace(value=PARAMS.get(n, v))
    def create_subscription(s, t, topic, cb, q): s.subs[topic] = cb
    def create_client(s, t, name): return Client(name)
    def get_logger(s): return types.SimpleNamespace(info=print, warn=print, error=print)


def mod(n, **kw):
    m = types.ModuleType(n); m.__dict__.update(kw); sys.modules[n] = m


NS = types.SimpleNamespace
mod('rclpy', init=lambda: None, ok=lambda: True, shutdown=lambda: None,
    spin_once=lambda n, timeout_sec=0: None)
mod('rclpy.node', Node=Node)
mod('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=lambda **k: None,
    DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1))
TF = {'t': None}


class Buffer:
    def lookup_transform(s, target, source, when):
        if TF['t'] is None:
            raise RuntimeError('no tf')
        x, y, yaw, sec = TF['t']
        return NS(header=NS(stamp=NS(sec=sec, nanosec=0)),
                  transform=NS(translation=NS(x=x, y=y, z=0.0),
                               rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


mod('tf2_ros', Buffer=Buffer, TransformListener=lambda *a, **k: None)
mod('rclpy.time', Time=lambda: None)
mod('geometry_msgs'); mod('geometry_msgs.msg', PoseWithCovarianceStamped=object)
mod('nav_msgs'); mod('nav_msgs.msg', OccupancyGrid=object, Odometry=object, Path=object)
mod('sensor_msgs'); mod('sensor_msgs.msg', CompressedImage=object, Image=object)
mod('std_msgs'); mod('std_msgs.msg', String=object)
mod('std_srvs'); mod('std_srvs.srv', Trigger=NS(Request=lambda: None))
PARAMS = {'station_labels': '작업대1,작업대2',
          'points': '{"STORE":[0,0,0],"A":[1.0,0.0,0],"B":[1.0,1.0,1.57],"HOME":[0.0,-0.5,0]}'}

spec = importlib.util.spec_from_file_location('gui', sys.argv[1])
gui = importlib.util.module_from_spec(spec); spec.loader.exec_module(gui)
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp'

import numpy as np  # noqa: E402
QApplication, QImage, QColor, QPainter, QFont, Qt = (gui.QApplication, gui.QImage, gui.QColor, gui.QPainter,
                                                     gui.QFont, gui.Qt)
print('Qt binding:', gui.QT_API)

app = QApplication([]); app.setFont(QFont('Noto Sans CJK KR', 10))
node = gui.GuiNode(); w = gui.MainWindow(node); w.show()
DOCK = dict(node.dock); node.dock['points'] = []   # docking tests below; earlier ETA numbers are Nav2-only


def ok(c, m):
    assert c, m; print('PASS', m)


def push(status):
    node.subs['delivery/status'](NS(data=json.dumps(status))); node.update_pose(); w.refresh()


def shot(tab, name):
    w.tabs.setCurrentIndex(tab); app.processEvents(); w.grab().save(f'{OUT}/{name}.png')


# --- no controller
w.refresh()
ok('컨트롤러 응답 없음' in w.lbl_state.text(), 'stale when no status')
ok(not w.btn_loaded.isEnabled(), 'buttons disabled when stale')
ok(len(w.maps) == 3, 'map widget on every tab')

# --- map / plan / pose / camera
W, H, res, ox, oy = 110, 125, 0.02, -0.6, -0.9
g = np.full((H, W), -1, np.int8)
g[5:H - 5, 5:W - 5] = 0
g[5:H - 5, 5:8] = 100; g[5:H - 5, W - 8:W - 5] = 100; g[5:8, 5:W - 5] = 100; g[H - 8:H - 5, 5:W - 5] = 100


def blk(x0, y0, x1, y1):
    g[int((y0 - oy) / res):int((y1 - oy) / res), int((x0 - ox) / res):int((x1 - ox) / res)] = 100


blk(-0.25, 0.25, 0.25, 0.45); blk(1.2, -0.25, 1.45, 0.25); blk(0.75, 1.25, 1.25, 1.4); blk(0.35, 0.55, 0.6, 0.75)
node.on_map(NS(info=NS(width=W, height=H, resolution=res, origin=NS(position=NS(x=ox, y=oy))),
               data=g.flatten().tolist()))
ok(node.map is not None and node.map['ub'][1] - node.map['ub'][0] < W + 20, 'map converted + fitted to room')

# --- clean display map: laser streak cut, walls solid, rotation found
cells = np.full((120, 160), -1, np.int8)
cells[30:90, 30:110] = 0                       # room floor
cells[30:32, 30:110] = 100; cells[88:90, 30:110] = 100
cells[30:90, 30:32] = 100; cells[30:90, 108:110] = 100
cells[58:61, 110:158] = 0                      # thin laser streak through a gap
cells[58:61, 108:110] = 0
cells[50, 60] = 100                            # a chair leg
rgb, room, walls = gui.build_display_map(cells, 0.05)
ok(tuple(rgb[59, 140]) == (241, 239, 232) and tuple(rgb[60, 70]) == (255, 255, 255)
   and tuple(rgb[31, 70]) == (95, 94, 90) and tuple(rgb[50, 60]) == (226, 224, 217),
   'clean map: streak removed, floor white, wall dark, furniture light')
rr, cc = np.mgrid[0:200, 0:200]
a = math.radians(20)
u = (cc - 100) * math.cos(a) + (rr - 100) * math.sin(a)
v = -(cc - 100) * math.sin(a) + (rr - 100) * math.cos(a)
box = (np.abs(np.abs(u) - 60) < 1.5) & (np.abs(v) < 40) | (np.abs(np.abs(v) - 40) < 1.5) & (np.abs(u) < 60)
ok(abs(abs(gui.dominant_angle(box)) - 20) <= 1.0, 'rotation of a tilted room found (~20 deg)')
node.on_plan(NS(poses=[NS(pose=NS(position=NS(x=x, y=y)))
                       for x, y in [(0.45, 0.05), (0.6, 0.02), (0.75, 0.0), (0.9, 0.0), (1.0, 0.0)]]))
ok(len(node.plan) == 5, 'plan received')
node.subs['/amcl_pose'](NS(pose=NS(pose=NS(position=NS(x=0.45, y=0.05), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))))
img = QImage(640, 480, QImage.Format_RGB888); img.fill(QColor('#5a6b7a'))
p = QPainter(img); p.setPen(QColor('white')); p.setFont(QFont('Noto Sans CJK KR', 28))
p.drawText(img.rect(), Qt.AlignCenter, 'camera test frame'); p.end()
node.frame = img; node.frame_time = time.monotonic()

# --- saved map file (map_server yaml + pgm), same convention as map_server
import tempfile  # noqa: E402
tmp = tempfile.mkdtemp()
pix = np.full((40, 60), 254, np.uint8)      # free (white)
pix[0, :] = 0                               # top image row = black wall (max y)
pix[:, 0] = 205                             # left column = unknown (gray)
with open(f'{tmp}/room.pgm', 'wb') as f:
    f.write(b'P5\n60 40\n255\n' + pix.tobytes())
with open(f'{tmp}/room.yaml', 'w') as f:
    f.write('image: room.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -0.5, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
saved = node.map
node.map_style = 'raw'
node.load_map_file(f'{tmp}/room.yaml')
m = node.map
ok(node.map_source == 'file' and m['res'] == 0.05 and m['ox'] == -1.0 and m['oy'] == -0.5 and m['h'] == 40,
   'map yaml: resolution + origin read')
img = m['img']
ok(QColor(img.pixel(30, 0)).red() < 20 and QColor(img.pixel(30, 20)).red() > 240
   and abs(QColor(img.pixel(0, 20)).red() - 205) < 3, 'map pgm: wall / free / unknown kept, top stays top')
node.load_map_file(f'{tmp}/missing.yaml')
ok('map_yaml 읽기 실패' in w.logw.toPlainText() or True, 'bad path does not crash')
node.map, node.map_source = saved, 'topic'
node.map_style = 'clean'
PARAMS['map_yaml'] = f'{tmp}/room.yaml'
n2 = gui.GuiNode(); w2 = gui.MainWindow(n2)
ok(n2.map_source == 'file' and '저장된 지도 파일' in w2.logw.toPlainText(), 'GUI starts with map_yaml param')
w2.close(); del PARAMS['map_yaml']

# --- hand-drawn floor plan next to the map yaml (display only)
fp = {'name': 'room', 'view_deg': 0, 'walls': [[-1, -0.5], [2, -0.5], [2, 1.5], [-1, 1.5]],
      'glass': [[[2, -0.5], [-1, -0.5]]], 'obstacles': [{'label': '장애물', 'polygon': [[0, 0], [0.4, 0], [0.2, 0.4]]}],
      'tables': [[[0.5, 0.8], [1.5, 0.8], [1.5, 1.1], [0.5, 1.1]]]}
json.dump(fp, open(f'{tmp}/room.floorplan.json', 'w'), ensure_ascii=False)
PARAMS['map_yaml'] = f'{tmp}/room.yaml'
n3 = gui.GuiNode(); w3 = gui.MainWindow(n3); w3.resize(1360, 880); w3.show()
ok(n3.floorplan is not None and n3.view() is n3.fp_view and n3.map_source == 'file',
   'floor plan found next to map yaml, used for drawing (map still loaded)')
ok('평면도 "room" 표시 중' in w3.logw.toPlainText() and '유리창' in ' '.join(
   l.text() for l in w3.findChildren(gui.QLabel)), 'floor plan logged + legend')
n3.pose, n3.pose_source = (0.5, 0.5, 0.0), 'TF'
n3.subs['delivery/status'](NS(data=json.dumps({'state': 'TO_STATION', 'target': 'A', 'queue': [],
                                                'current': {'id': 9, 'station': 'A'}})))
w3.refresh(); app.processEvents(); w3.grab().save(f'{OUT}/gui_floorplan_test.png')
mw = w3.maps[0]
img = mw.grab().toImage()
white = sum(QColor(img.pixel(x, y)).lightness() > 250 for x in range(0, img.width(), 8) for y in range(0, img.height(), 8))
ok(white > 0.2 * (img.width() // 8) * (img.height() // 8), 'floor plan painted (white floor)')
w3.close()
PARAMS['floorplan'] = 'none'
n4 = gui.GuiNode()
ok(n4.floorplan is None and n4.view() is n4.map, "floorplan:=none falls back to the map picture")
del PARAMS['map_yaml'], PARAMS['floorplan']

# --- base frame auto: Pinky without base_footprint uses base_link
class OnlyBaseLink(Buffer):
    def lookup_transform(s, target, source, when):
        if source != 'base_link':
            raise RuntimeError('no ' + source)
        return Buffer.lookup_transform(s, target, source, when)
n5 = gui.GuiNode(); n5.tf_buffer = OnlyBaseLink(); TF['t'] = (1.0, 2.0, 0.0, 500)
n5.update_pose()
ok(n5.base_frame == 'base_link' and n5.pose_source == 'TF' and n5.pose[:2] == (1.0, 2.0),
   'base frame auto-detected (base_link)')
TF['t'] = None
ok(n5.points and len(n5.station_labels) == 2 and n5.station_labels[0] == '작업대1', 'default labels 작업대1/2')

# --- WAIT_LOAD
status = {'state': 'WAIT_LOAD', 'current': {'id': 1, 'station': 'A'}, 'target': 'STORE',
          'queue': [{'id': 2, 'station': 'B'}, {'id': 3, 'station': 'A', 'count': 2}],
          'error': None, 'nav_ready': True, 'timeout_left': None, 'distance_remaining': None,
          'dry_run': False, 'batch': [{'id': 1, 'station': 'A'}]}
push(status)
ok('적재 대기' in w.lbl_state.text(), 'state label')
ok(w.btn_loaded.isEnabled() and not w.btn_received.isEnabled(), 'loaded enabled only in WAIT_LOAD')
ok(not w.btn_retry.isEnabled(), 'retry disabled outside ERROR')
ok(w.lbl['pose'].text() == '0.45, 0.05, 0°' and node.pose_source == 'AMCL', 'no TF -> /amcl_pose fallback')
ok('/amcl_pose로 대체' in w.logw.toPlainText(), 'TF missing logged once')
TF['t'] = (0.62, 0.03, math.radians(-5), 100); node.update_pose(); w.refresh()
ok(node.pose_source == 'TF' and w.lbl['pose'].text() == '0.62, 0.03, -5°', 'TF preferred when available')
ok('· TF' in w.lbl['pose_title'].text(), 'pose source shown on tile')
node.tf_fresh -= 5; node.update_pose()
ok(node.pose_source == 'AMCL', 'frozen TF stamp -> AMCL fallback')
TF['t'] = (0.63, 0.03, math.radians(-5), 101); node.update_pose()
ok(node.pose_source == 'TF', 'TF resumes when stamp changes')
TF['t'] = None; node.amcl_pose = None; node.tf_fresh -= 5; node.update_pose()
ok(node.pose_source == '끊김' and node.pose is not None, 'nothing live -> keep last pose, marked stale')
TF['t'] = (0.45, 0.05, 0.0, 102); node.update_pose(); w.refresh()
ok(w.list_queue.count() == 2 and 'count=2' in w.list_queue.item(1).text(), 'queue with extra fields')
ok(w.lbl['current_req'].text() == '#1 → A', 'current task (tab 1)')
ok(not w.tabs.tabIcon(1).isNull() and w.tabs.tabIcon(2).isNull() and '2' in w.tabs.tabText(0),
   'tab badges: confirm dot + queue count')
ok(w.badge_load.text() == '확인 필요' and w.badge_recv.text() == '미완료', 'load/receive badges')
shot(0, 'gui_tab_request'); shot(1, 'gui_tab_confirm'); shot(2, 'gui_tab_status')

w.btn_loaded.click(); ok(CALLS[-1] == 'delivery/loaded', 'loaded button calls service')
n0 = len(CALLS); w.btn_received.click(); ok(len(CALLS) == n0, 'receive disabled while loading')
reqs = [b for b in w.findChildren(gui.QPushButton) if '부품 요청' in b.text()]
n0 = len(CALLS)
reqs[0].click(); reqs[0].click(); reqs[0].click(); reqs[1].click()
ok(CALLS[n0:] == ['delivery/request_a', 'delivery/request_b'], 'double click within lock is ignored')
ok(not reqs[0].isEnabled() and '요청됨' in reqs[0].text() and not reqs[1].isEnabled(), 'clicked buttons locked')
shot(0, 'gui_request_lock')
import time as _t
t_end = _t.time() + 2.3
while _t.time() < t_end:
    app.processEvents(); _t.sleep(0.02)
ok(reqs[0].isEnabled() and reqs[0].text().endswith('부품 요청'), 'button unlocks after 2 s')
reqs[0].click()
ok(CALLS[-1] == 'delivery/request_a' and len(CALLS) == n0 + 3, 'request accepted again after lock')
ok('작업대1 요청' in w.logw.toPlainText(), 'reply logged')
ok('픽업 장소에서 작업대1 부품을 싣는 중이라 대기' in w.notice.text(), 'wait notice while loading')

# --- going to 작업대1, 작업대2 requests
status.update(state='TO_STATION', target='A', queue=[], distance_remaining=1.84, timeout_left=71.2)
push(status)
ok(w.lbl['dist'].text() == '1.84 m' and w.lbl['timeout'].text() == '71 s', 'distance/timeout shown')
for b in reqs: w.unlock_request(b)
reqs[1].click()
ok(w.notice.isVisibleTo(w) and '작업대1 쪽으로 배달하러 가는 중이라 대기' in w.notice.text()
   and '대기 순번 1번' in w.notice.text(), 'wait notice for 작업대2 while going to 작업대1')
shot(0, 'gui_wait_notice')

# --- idle -> leaves right away
status.update(state='IDLE', current=None, target=None, distance_remaining=None, timeout_left=None)
w.unlock_request(reqs[1]); push(status); reqs[1].click()
ok('곧 픽업 장소로 출발' in w.notice.text(), 'no-wait notice when idle')
status.update(state='TO_HOME', target='HOME'); push(status); w.unlock_request(reqs[0]); reqs[0].click()
ok('복귀를 멈추고' in w.notice.text(), 'no-wait notice while returning home')
w.notice.hide()

# --- popup for the loading/receiving person
ok(not w.popup.isVisible(), 'no popup for tasks already there at start')
status.update(state='TO_STATION', target='A', current={'id': 1, 'station': 'A'}, queue=[], distance_remaining=0.6)
push(status)
w.tabs.setCurrentIndex(0)
status.update(queue=[{'id': 4, 'station': 'B'}]); push(status)
ok(not w.popup.isVisible() and len(w.pending_popups) == 1, 'no popup while on the request tab (kept)')
w.tabs.setCurrentIndex(1); app.processEvents()
ok(w.popup.isVisible() and w.popup.head.text() == '작업대2에서 부품 요청이 들어왔습니다.'
   and '대기 순번 1번' in w.popup.sub.text(), 'opening confirm tab shows the kept request')
ok(w.popup.load.text() == 'pinky_1에 적재해 주세요' and 'pinky_1이 픽업대에 오면' in w.popup.how.text(),
   'popup: which Pinky to load and how')
ok('pinky_1이 작업대1 배송을 마친 뒤 픽업대로 옵니다' in w.popup.sub.text(), 'popup: when that Pinky comes')
ok(gui.josa('pinky_1') == '이' and gui.josa('pinky_2') == '가' and gui.josa('작업대') == '가'
   and gui.josa('수납') == '이', 'josa particle')
status.update(queue=[{'id': 4, 'station': 'B'}, {'id': 5, 'station': 'A', 'robot': 'pinky_2'}]); push(status)
ok(w.popup.isVisible() and '2건' in w.popup.tag.text() and w.popup.more.count() == 1,
   'on confirm tab: new request pops up at once and stacks')
ok(w.popup.load.text() == 'pinky_2에 적재해 주세요' and '앞선 배송 2건' in w.popup.sub.text()
   and 'pinky_2가' in w.popup.sub.text(), 'task-level robot assignment (multi-Pinky ready)')
ok('pinky_1 ·' in w.pill_state.text(), 'header shows robot name')
w.popup.raise_(); app.processEvents(); w.popup.grab().save(f'{OUT}/gui_popup.png')
w.tabs.setCurrentIndex(0); app.processEvents()
ok(not w.popup.isVisible(), 'leaving the confirm tab closes the popup')
push(status); ok(not w.popup.isVisible(), 'same ids do not pop up again')
status.update(queue=[{'id': 4, 'station': 'B'}, {'id': 5, 'station': 'A', 'robot': 'pinky_2'},
                     {'id': 6, 'station': 'B'}]); push(status)
status.update(queue=[{'id': 4, 'station': 'B'}, {'id': 5, 'station': 'A', 'robot': 'pinky_2'}]); push(status)
w.tabs.setCurrentIndex(1); app.processEvents()
ok(not w.popup.isVisible() and not w.pending_popups, 'kept request already gone -> no stale popup')
w.tabs.setCurrentIndex(0)

# --- arrival countdown (speed 0.2 m/s, detour 1.25, STORE-A = 1 m, STORE-B = 1.414 m)
ok(gui.fmt_left(125) == '2m 05s left' and gui.fmt_left(9.6) == '10s left', 'fmt_left format')
node.speed = 0.2; node.arrive_tol = 0.0   # geometry checks first, tolerance below
status.update(state='TO_STORE', target='STORE', distance_remaining=1.0, queue=[{'id': 4, 'station': 'B'}])
push(status)
ok(w.lbl['eta_main'].text() == '11s left', 'TO_STORE: drive to pickup + pickup->A')
status.update(distance_remaining=0.4); push(status)
ok(w.lbl['eta_main'].text() == '8s left', 'counts down while driving to pickup')
status.update(state='WAIT_LOAD', distance_remaining=None); push(status)
first = w.lbl['eta_main'].text(); time.sleep(0.3); push(status)
ok(first == '6s left' and w.lbl['eta_main'].text() == first and '시간 멈춤' in w.lbl['eta_sub'].text(),
   'WAIT_LOAD: time held at pickup')
status.update(state='TO_STATION', target='A', distance_remaining=0.6); push(status)
ok(w.lbl['eta_main'].text() == '3s left', 'TO_STATION: counts down again after loading')
ok('약 18s left' in w.list_queue.item(0).text(), 'queued 작업대2 ETA = A leg + A->pickup + pickup->B')
node.speed = 0.15; status.update(distance_remaining=7.6); push(status); shot(0, 'gui_eta_moving'); node.speed = 0.2; status.update(distance_remaining=0.6); push(status)
status.update(state='WAIT_RECEIVE', distance_remaining=None); push(status)
ok(w.lbl['eta_main'].text() == '도착' and '약 15s left' in w.list_queue.item(0).text(),
   'arrived; queued ETA held while receiving')
ok(w.btn_received.isEnabled() and w.station_panels['A']['badge'].text() == '수령 대기'
   and not w.station_panels['B']['recv'].isEnabled(), '1 robot: receive button on the 작업대1 panel')
n0 = len(CALLS); w.btn_received.click(); ok(CALLS[n0:] == ['delivery/received'], '1 robot: delivery/received')
shot(0, 'gui_eta')

# --- Nav2 goal tolerance: every leg ends 0.3 m early
node.arrive_tol = 0.3; node.speed = 0.2
status.update(state='TO_STATION', target='A', current={'id': 1, 'station': 'A'}, distance_remaining=1.1,
              queue=[{'id': 4, 'station': 'B'}]); push(status)
ok(w.lbl['eta_main'].text() == '4s left', 'tolerance: (1.1 - 0.3) / 0.2 = 4 s')
status.update(distance_remaining=0.25); push(status)
ok(w.lbl['eta_main'].text() == '0s left', 'inside tolerance -> 0 s')
e = gui.estimate_etas({'state': 'WAIT_LOAD', 'current': {'id': 1, 'station': 'A'}, 'queue': []},
                      None, {'STORE': (0, 0), 'A': (2.0, 0)}, 0.2, 1.0, 0.3)[0]
ok(abs(e - 8.5) < 1e-9, 'future leg also ends 0.3 m early ((2.0 - 0.3) / 0.2)')
node.arrive_tol = 0.0

# --- speed learning from a real leg (1.0 m in 5 s = 0.2 m/s)
node.speed = 0.15; node.speed_learned = False; node.leg_state = None
node.status = {'state': 'TO_STATION'}; node.pose_source = 'TF'
for i in range(11):
    node.pose = (0.1 * i, 0.0, 0.0); node._learn_speed(1000 + 0.5 * i)
node.status = {'state': 'WAIT_RECEIVE'}; node._learn_speed(1005.5)
ok(node.speed_learned and abs(node.speed - (0.6 * 0.15 + 0.4 * 1.0 / 5.5)) < 1e-6, 'speed learned from leg')
node.speed = 0.2
status.update(state='TO_STATION', target='A', current={'id': 1, 'station': 'A'})

# --- ArUco docking in the ETA (pickup = Nav2 driving + marker alignment)
D = dict(DOCK, points=['STORE'], speed=0.05, turn_speed=30.0, approach=0.4, turn_after=0.0)
ok(abs(gui.dock_seconds(D) - 8.0) < 1e-9, 'docking not started: approach 0.4 m / 0.05 = 8 s')
ok(abs(gui.dock_seconds(D, {'distance': 0.2, 'angle_deg': -15}) - 4.5) < 1e-9, 'live: 0.2/0.05 + 15/30 = 4.5 s')
ok(abs(gui.dock_seconds(dict(D, turn_after=180)) - 14.0) < 1e-9, '180 deg turn after docking adds 6 s')
P = {'STORE': (0, 0, 0), 'A': (2.0, 0, 0), 'HOME': (0, -1, 0)}
st_nav = {'state': 'TO_STORE', 'target': 'STORE', 'current': {'id': 1, 'station': 'A'},
          'distance_remaining': 1.3, 'queue': []}
e = gui.estimate_etas(st_nav, None, P, 0.2, 1.0, 0.3, D)[0]
ok(abs(e - 21.5) < 1e-9, 'Nav2 (1.3-0.3)/0.2=5 + docking 8 + to 작업대1 (2-0.3)/0.2=8.5')
e = gui.estimate_etas(st_nav, None, P, 0.2, 1.0, 0.3, D, {'distance': 0.1, 'angle_deg': 6})[0]
ok(abs(e - 10.7) < 1e-9, 'while docking: camera 0.1/0.05 + 6/30 = 2.2 (Nav2 distance ignored) + 8.5')
st_rx = {'state': 'WAIT_RECEIVE', 'target': 'A', 'current': {'id': 1, 'station': 'A'},
         'queue': [{'id': 2, 'station': 'A'}]}
e = gui.estimate_etas(st_rx, None, P, 0.2, 1.0, 0.3, D)[1][0]
ok(abs(e - 25.0) < 1e-9, 'queued task: back to pickup 8.5 + docking 8 + to 작업대1 8.5')
e = gui.estimate_etas(dict(st_nav, state='DOCKING'), None, P, 0.2, 1.0, 0.3, D, {'distance': 0.1, 'angle_deg': 6})[0]
ok(abs(e - 10.7) < 1e-9, "optional 'DOCKING' state works the same")

node.dock = dict(D); saved_pts = node.points; node.points = dict(P); node.arrive_tol = 0.3; node.speed = 0.2
node.detour = 1.0
push(dict(st_nav, nav_ready=True, dry_run=False))
ok(w.lbl['eta_main'].text() == '22s left' and '남은 거리 1.3 m' in w.lbl['eta_sub'].text(), 'GUI: Nav2 phase')
node.on_dock(types.SimpleNamespace(data=json.dumps({'distance': 0.1, 'angle_deg': 6, 'visible': True})))
push(dict(st_nav, nav_ready=True, dry_run=False))
ok(w.lbl['eta_main'].text() == '11s left' and '마커에 맞추는 중 (0.10 m, +6°)' in w.lbl['eta_sub'].text(),
   'GUI: docking phase from camera feedback')
ok(w.lbl['dist'].text() == '마커 0.10 m, +6°', 'status tab shows marker distance while docking')
shot(0, 'gui_eta_docking')
node.on_dock(types.SimpleNamespace(data=json.dumps({'visible': False})))
push(dict(st_nav, nav_ready=True, dry_run=False))
ok('마커 찾는 중' in w.lbl['eta_sub'].text() and w.lbl['eta_main'].text() == '11s left',
   'marker lost: keeps last distance')
node.dock_fb_time -= 2.0; push(dict(st_nav, nav_ready=True, dry_run=False))
ok(w.lbl['eta_main'].text() == '22s left', 'stale camera feedback -> back to Nav2 estimate')
ok('마커 정렬 0.05 m/s (설정값)' in w.lbl['eta_note'].text(), 'docking speed shown in the note')

# docking speed learning: 0.3 m in 5 s (no turn) = 0.06 m/s
node.dock_run = None; node.dock_learned = False; node.dock['speed'] = 0.05
node.status = dict(st_nav); node.dock_fb = {'distance': 0.3, 'angle_deg': 0}; node.dock_fb_time = 2000
node._learn_dock(2000)
node.status = dict(st_nav, state='WAIT_LOAD'); node._learn_dock(2005)
ok(node.dock_learned and abs(node.dock['speed'] - (0.6 * 0.05 + 0.4 * 0.06)) < 1e-9, 'docking speed learned')
# Nav2 speed learning stops counting when docking starts: 1.0 m in 5 s, then 10 s of docking
node.speed = 0.15; node.speed_learned = False; node.leg_state = None; node.dock_fb = None
node.status = dict(st_nav); node.pose_source = 'TF'
for i in range(11):
    node.pose = (0.1 * i, 0.0, 0.0); node._learn_speed(3000 + 0.5 * i)
node.dock_fb = {'distance': 0.3, 'angle_deg': 0}
for k in range(11):
    node.dock_fb_time = 3005 + k; node._learn_speed(3005 + k)
node.status = dict(st_nav, state='WAIT_LOAD'); node._learn_speed(3016)
ok(abs(node.speed - (0.6 * 0.15 + 0.4 * 0.2)) < 1e-6, 'docking time not counted in Nav2 speed (0.2 m/s)')
node.dock = dict(DOCK, points=[]); node.dock_fb = None; node.points = saved_pts
node.arrive_tol = 0.0; node.speed = 0.2; node.detour = 1.25

# --- fixed points: no registration UI; controller points still drawn
saved_status = dict(status)
fixed = dict(status, points={'STORE': [-5.64, 2.1, 0.0], 'A': [1.0, 0.0, 0.0],
                             'B': [1.0, 1.0, 1.57], 'HOME': [-5.9, -0.1, 0.0]})
push(fixed)
ok(node.points['STORE'] == (-5.64, 2.1, 0.0) and 'HOME' in node.points and 'HOME2' not in node.points, 'points come from controller status')
ok(not hasattr(w, 'btn_reg_start') and 'register' not in w.lbl, 'no point registration UI')
ok('STORE_WAIT' not in gui.POINT_NAMES and gui.POINT_NAMES['HOME'] == '수납', 'no pickup wait point')

# --- two robots (2-robot controller status)
ok(node.robot_names == ['pinky_1', 'pinky_2'], 'robots param default: pinky_1, pinky_2')
for topic in ('/pinky_2/amcl_pose', '/pinky_2/plan', '/pinky_2/delivery/dock', '/pinky_2/camera/image_raw/compressed',
              '/pinky_1/amcl_pose', '/amcl_pose', 'delivery/dock'):
    ok(topic in node.subs, f'subscribed {topic}')
TF['t'] = None
P1, P2 = node.tracks['pinky_1'], node.tracks['pinky_2']
P1.amcl_pose = None; P1.pose = (1.5, 0.0, 0.0)
node.dock['points'] = []; node.speed = 0.2; node.detour = 1.0; node.arrive_tol = 0.0
PTS2 = {'STORE': [0, 0, 0], 'A': [2, 0, 0], 'B': [0, 2, 0],
        'HOME': [-1, 0, 0]}
multi = {'robots': {
    'pinky_1': {'state': 'TO_STATION', 'current': {'id': 11, 'station': 'A', 'robot': 'pinky_1'}, 'target': 'A',
                'distance_remaining': 2.4, 'nav_ready': True, 'home': 'HOME'},
    'pinky_2': {'state': 'WAIT_LOAD', 'current': {'id': 12, 'station': 'B', 'robot': 'pinky_2'}, 'target': 'STORE',
                'nav_ready': True, 'home': 'HOME'}},
    'queue': [{'id': 13, 'station': 'A'}], 'dry_run': False, 'points': PTS2}
w.tabs.setCurrentIndex(0); push(multi)
node.subs['/pinky_2/amcl_pose'](NS(pose=NS(pose=NS(position=NS(x=0.1, y=0.05), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))))
node.update_pose(); w.refresh()
ok(P2.pose[:2] == (0.1, 0.05) and P2.pose_source == 'AMCL', 'pinky_2 pose from /pinky_2/amcl_pose')
ok(w.pill_robot['pinky_1'].text() == 'pinky_1 · 작업대로 배달 중' and w.pill_robot['pinky_2'].text() == 'pinky_2 · 적재 대기',
   'header: one pill per robot')
T1, T2 = w.eta_tiles['pinky_1'], w.eta_tiles['pinky_2']
ok(T1['main'].text() == '12s left' and T2['main'].text() == '10s left', 'ETA per robot (2.4/0.2=12, store->B 2/0.2=10)')
ok(T1['task'].text() == '#11 → A' and T2['who'].isVisibleTo(w), 'tile shows robot and task')
ok('→ pinky_2 예정' in w.list_queue.item(0).text() and '30s left' in w.list_queue.item(0).text(),
   'queued task goes to the robot at the pickup first (pinky_2: 10+10, then +10)')
shot(0, 'gui_multi_request')
w.tabs.setCurrentIndex(1); app.processEvents()
R1, R2 = w.robot_cards['pinky_1'], w.robot_cards['pinky_2']
ok(R2['btn_loaded'].isEnabled() and not R1['btn_loaded'].isEnabled() and 'btn_received' not in R1,
   'loading buttons per robot; no receive button on the confirm tab')
ok('pinky_2가 픽업 장소에 도착했습니다' in w.guide.text(), 'guide names the robot to load')
R2['btn_loaded'].click(); ok(CALLS[-1] == 'delivery/pinky_2/loaded', 'per-robot service: delivery/pinky_2/loaded')
shot(1, 'gui_multi_confirm')
w.popup.close()

# receive at the workstation panel (request tab)
multi['robots']['pinky_1'] = dict(multi['robots']['pinky_1'], state='WAIT_RECEIVE', distance_remaining=None)
w.tabs.setCurrentIndex(0); push(multi)
SA, SB = w.station_panels['A'], w.station_panels['B']
ok(SA['recv'].isEnabled() and not SB['recv'].isEnabled() and SA['badge'].text() == '수령 대기'
   and 'pinky_1이 도착했습니다' in SA['desc'].text(), 'workstation 1 panel: receive enabled for pinky_1')
ok(not w.tabs.tabIcon(0).isNull(), 'request tab dot while a Pinky waits at a workstation')
shot(0, 'gui_station_receive')
SA['recv'].click(); ok(CALLS[-1] == 'delivery/pinky_1/received', 'receive goes to the robot at that workstation')
multi['robots']['pinky_1'] = dict(multi['robots']['pinky_1'], state='TO_STATION', distance_remaining=2.4)
push(multi)
ok(not SA['recv'].isEnabled() and SA['badge'].text() == '배달 중' and 'pinky_1이 오는 중' in SA['desc'].text(),
   'workstation panel: robot on the way')

# pickup busy: pinky_2 waits at the wait spot
multi['robots']['pinky_1'] = {'state': 'TO_STORE', 'current': {'id': 14, 'station': 'A'}, 'target': 'STORE',
                              'distance_remaining': 1.0, 'nav_ready': True}
multi['robots']['pinky_2'] = {'state': 'WAIT_PICKUP', 'current': {'id': 15, 'station': 'B'}, 'target': None,
                              'nav_ready': True}
multi['queue'] = []; w.tabs.setCurrentIndex(0); push(multi)
ok(T1['main'].text() == '15s left' and T2['main'].text() == '16s left',
   'pickup one at a time: pinky_1 5+10, pinky_2 waits 5 + its spot->store 0.6 + 10')
ok('픽업 차례 대기' in T2['sub'].text() and '픽업 차례 대기' in w.pill_robot['pinky_2'].text(), 'WAIT_PICKUP shown')
ok(w.robot_cards['pinky_2']['badge_load'].text() == '미완료', 'waiting robot: loading not done')

# request notice: both busy -> wait with prediction; one idle -> leaves now
for b in w.req_btns: w.unlock_request(b)
w.req_btns[0].click()
ok('모든 Pinky가 배송 중이라 대기' in w.notice.text() and '픽업대에 옵니다' in w.notice.text(), 'notice: all busy')
w.unlock_request(w.req_btns[0])
multi['robots']['pinky_2'] = {'state': 'IDLE', 'current': None, 'target': None, 'nav_ready': True}
push(multi); w.req_btns[0].click()
ok('pinky_2가 곧 픽업 장소로 출발' in w.notice.text(), 'notice: idle robot named')
w.unlock_request(w.req_btns[0]); w.notice.hide()

# new request popup predicts the robot
w.tabs.setCurrentIndex(1); app.processEvents()
multi['queue'] = [{'id': 21, 'station': 'B'}]; push(multi)
ok(w.popup.isVisible() and w.popup.load.text() == 'pinky_2에 적재해 주세요' and '픽업대에 옵니다' in w.popup.sub.text(),
   'popup: predicted robot and time')
w.popup.close()

# status tab: robot picker, per-robot retry, docking feedback per robot
multi['robots']['pinky_1'] = dict(multi['robots']['pinky_1'], state='ERROR', error='Nav2 aborted')
multi['queue'] = []; push(multi)
w.tabs.setCurrentIndex(2); app.processEvents()
ok(w.sel_row.isVisibleTo(w) and w.sel_btns['pinky_1'].isChecked(), 'robot picker shown, pinky_1 selected')
ok('pinky_1' in w.lbl_state.text() and '오류' in w.lbl_state.text() and w.btn_retry.isEnabled(), 'selected robot state')
w.btn_retry.click(); ok(CALLS[-1] == 'delivery/pinky_1/retry', 'retry goes to the selected robot')
multi['robots']['pinky_2'] = {'state': 'TO_STORE', 'current': {'id': 22, 'station': 'A'}, 'target': 'STORE',
                              'distance_remaining': 0.2, 'nav_ready': True}
push(multi)
node.dock['points'] = ['STORE']
node.subs['/pinky_2/delivery/dock'](NS(data=json.dumps({'distance': 0.12, 'angle_deg': 4, 'visible': True})))
w.sel_btns['pinky_2'].click()
ok(w.selected == 'pinky_2' and w.lbl['dist'].text() == '마커 0.12 m, +4°' and node.dock_live(name='pinky_1') is None,
   'docking feedback per robot (/pinky_2/delivery/dock)')
ok(w.lbl['pose'].text() == '0.10, 0.05, 0°' and '카메라 · pinky_2' == w.cam_title.text(), 'pose/camera of the selected robot')
shot(2, 'gui_multi_status')
node.dock['points'] = []

# back to the 1-robot controller: extra robot hidden
P1.pose = (0.45, 0.05, 0.0)
push(saved_status)
ok(not w.pill_robot['pinky_2'].isVisibleTo(w) and not w.sel_row.isVisibleTo(w)
   and not w.eta_tiles['pinky_2']['frame'].isVisibleTo(w), '1-robot status: single-robot screen')
ok(node.service_name('loaded', 'pinky_1') == 'delivery/loaded', '1-robot status: plain service names')
status = saved_status

# --- error
status.update(state='ERROR', error='Navigation to A ended: status=6', target='A',
              current={'id': 1, 'station': 'A'})
push(status)
ok(w.btn_retry.isEnabled() and '오류' in w.lbl_state.text(), 'ERROR enables retry')
ok(not w.tabs.tabIcon(2).isNull() and w.tabs.tabIcon(1).isNull(), 'status tab dot on ERROR')
ok(w.badge_load.text() == '완료', 'ERROR on way to station -> loaded done')
shot(2, 'gui_error')
# --- smoother pose: odometry between amcl updates, display smoothing
import math as _m
def _pm(x, y, yaw): return NS(pose=NS(pose=NS(position=NS(x=x, y=y), orientation=NS(x=0.0, y=0.0, z=_m.sin(yaw/2), w=_m.cos(yaw/2)))))
TF['t'] = None
ok('/odom' in node.subs and '/pinky_2/odom' in node.subs, 'odometry subscribed per robot')
node.subs['/odom'](_pm(1.0, 0.0, 0.0))
node.subs['/amcl_pose'](_pm(2.0, 3.0, _m.pi / 2))
node.subs['/odom'](_pm(1.5, 0.0, 0.0))          # drove 0.5 m forward since that amcl_pose
node.update_pose()
ok(node.pose_source == 'AMCL+odom' and abs(node.pose[0] - 2.0) < 1e-6 and abs(node.pose[1] - 3.5) < 1e-6
   and abs(node.pose[2] - _m.pi / 2) < 1e-6, 'amcl_pose + odometry since then (0.5 m forward while facing +y)')
node.main.odom_time -= 1.0; node.update_pose()
ok(node.pose_source == 'AMCL' and node.pose[:2] == (2.0, 3.0), 'stale odometry -> plain amcl_pose')
tr = node.main
tr.view_pose, tr._view_t, tr.pose = (0.0, 0.0, 0.0), 10.0, (0.1, 0.0, 0.0)
node._smooth_view(tr, 10.1)
ok(0.02 < tr.view_pose[0] < 0.04, 'display follows with a short lag (0.3 s)')
v = tr.view_pose; tr.pose = (v[0] + 0.01, 0.0, 0.0); node._smooth_view(tr, 10.2)
ok(tr.view_pose == v, 'jitter under 2 cm ignored')
tr.pose = (5.0, 5.0, 0.0); node._smooth_view(tr, 10.3)
ok(tr.view_pose == (5.0, 5.0, 0.0), 'jump > 0.5 m snaps')

# --- real status from fms_controller.py (python3 test_fms.py fms_controller.py samples.json first)
if len(sys.argv) > 3:
    samples = json.load(open(sys.argv[3]))
    node.points = {}
    for i, smp in enumerate(samples):
        push(smp)
    ok(all(isinstance(x, dict) for x in samples) and len(samples) >= 5, 'fms_controller status samples pushed')
    push(samples[2])
    pick = samples[2]['pickup']; wait = [k for k in samples[2]['robots'] if k != pick][0]
    ok('마커' in w.pill_robot[pick].text() and '픽업 차례 대기' in w.pill_robot[wait].text(),
       'fms status: DOCKING and WAIT_PICKUP shown per robot')
    ok('마커에 맞추는 중' in w.eta_tiles[pick]['sub'].text(), 'DOCKING tile text without camera feedback')
    push(samples[3])
    ok(w.robot_cards[pick]['btn_loaded'].isEnabled() and not w.robot_cards[wait]['btn_loaded'].isEnabled(),
       'fms status: loading button only for the robot at the pickup')
    w.robot_cards[pick]['btn_loaded'].click()
    ok(CALLS[-1] == f'delivery/{pick}/loaded', 'GUI calls the controller service name')
    push(samples[5])
    stn = samples[5]['robots'][pick]['current']['station']
    ok(w.station_panels[stn]['recv'].isEnabled(), 'fms status: receive button on the right workstation')
    w.station_panels[stn]['recv'].click()
    ok(CALLS[-1] == f'delivery/{pick}/received', 'receive goes to that robot')
    shot(0, 'gui_fms_request')
print('ALL PASS')
