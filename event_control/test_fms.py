import sys, types, time, importlib.util
import tempfile
import os

# ---- fake ROS modules ----
class Fut:
    def __init__(s): s._r=None; s._cbs=[]; s.done=False
    def set(s,r):
        s._r=r; s.done=True
        for c in s._cbs: c(s)
    def result(s): return s._r
    def add_done_callback(s,c):
        s._cbs.append(c)
        if s.done: c(s)

class Node:
    def __init__(s,name): s._srv={}
    @property
    def services(s): return iter(())   # read-only like real rclpy
    def declare_parameter(s,n,v):
        return types.SimpleNamespace(value=PARAMS.get(n,v))
    def create_publisher(s,*a): return types.SimpleNamespace(publish=lambda m: STATUS.append(m.data))
    def create_service(s,t,name,cb): s._srv[name]=cb; return name
    def create_subscription(s,t,topic,cb,q): s._sub=getattr(s,'_sub',{}); s._sub[topic]=cb
    def create_client(s,t,name): c=Client(name); CLIENTS[name]=c; return c
    def create_timer(s,p,cb): return cb
    def get_logger(s):
        return types.SimpleNamespace(info=lambda m:None, warn=lambda m:None, error=lambda m:LOG.append(m))
    def get_clock(s):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))
    def call(s,name):
        return s._srv[name](None, types.SimpleNamespace(success=None,message=''))

CLIENTS={}
class Client:
    ready=True
    def __init__(s,name): s.name=name; s.calls=0
    def service_is_ready(s): return Client.ready
    def call_async(s,req): s.calls+=1; return Fut()

class Handle:
    def __init__(s): s.accepted=True; s.res=Fut(); s.cancelled=False
    def get_result_async(s): return s.res
    def cancel_goal_async(s):
        s.cancelled=True
        f=Fut(); f.set(NS(goals_canceling=[1])); return f

ACTIONS={}
class ActionClient:
    ready=True
    def __init__(s,node,typ,name): s.sent=[]; s.name=name; ACTIONS[name]=s
    def server_is_ready(s): return ActionClient.ready
    def send_goal_async(s,goal,feedback_callback=None):
        s.fb=feedback_callback
        f=Fut(); h=Handle(); s.sent.append((goal,f,h)); return f

class G:
    def __init__(s):
        P=lambda: types.SimpleNamespace(x=0,y=0,z=0,w=0)
        s.pose=types.SimpleNamespace(header=types.SimpleNamespace(frame_id=None,stamp=None),
                                     pose=types.SimpleNamespace(position=P(),orientation=P()))

class PG:
    def __init__(s):
        s.goal=G().pose

def mod(n,**kw):
    m=types.ModuleType(n); m.__dict__.update(kw); sys.modules[n]=m
mod('rclpy', init=lambda:None, spin=lambda n:None, ok=lambda:True, shutdown=lambda:None)
mod('rclpy.action', ActionClient=ActionClient); mod('rclpy.node', Node=Node)
mod('rclpy.signals', SignalHandlerOptions=types.SimpleNamespace(NO=0))
mod('action_msgs'); mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_ABORTED=6))
mod('geometry_msgs'); mod('geometry_msgs.msg', PoseStamped=object, PointStamped=object, PoseWithCovarianceStamped=object)
TFPOSE={'p':None}
class FakeBuffer:
    def lookup_transform(s,target,source,when):
        if TFPOSE['p'] is None or source!='base_link': raise RuntimeError('no tf')
        import math as _m; x,y,yaw=TFPOSE['p']
        return types.SimpleNamespace(transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=x,y=y,z=0.0),
               rotation=types.SimpleNamespace(x=0.0,y=0.0,z=_m.sin(yaw/2),w=_m.cos(yaw/2))))
mod('tf2_ros', Buffer=FakeBuffer, TransformListener=lambda *a,**k: None)
mod('rclpy.time', Time=lambda: None)
mod('nav2_msgs'); mod('nav2_msgs.action', NavigateToPose=types.SimpleNamespace(Goal=G),
    ComputePathToPose=types.SimpleNamespace(Goal=PG))
mod('sensor_msgs'); mod('sensor_msgs.msg', BatteryState=object)
mod('std_msgs'); mod('std_msgs.msg', String=lambda data: types.SimpleNamespace(data=data))
mod('std_srvs'); mod('std_srvs.srv', Trigger=object)


mod('std_srvs.srv', Trigger=types.SimpleNamespace(Request=lambda: None))
import json
PARAMS={}; STATUS=[]; LOG=[]
sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[1])))
spec=importlib.util.spec_from_file_location('fms', sys.argv[1]); fms=importlib.util.module_from_spec(spec); spec.loader.exec_module(fms)
NS=types.SimpleNamespace

def mk(**p):
    if p.get('dry_run') is False:
        p.setdefault('points_json', json.dumps(fms.POINTS))
        p.setdefault('home_slots_json', json.dumps([[0.6,0.2,1.57],[1.4,0.2,1.57]]))
        p.setdefault('station_clearance', 0.2)  # sample HOME and B are close; tests use virtual poses
        p.setdefault('traffic_enabled', False)
        p.setdefault('battery_required', False)
        p.setdefault('path_query_wait', 0.0)
    PARAMS.clear(); PARAMS.update(p); ACTIONS.clear(); CLIENTS.clear()
    n=fms.FmsController()
    if not n.dry_run:
        for r in n.robots.values():
            r.pose=n.home_slot_pose(r.home_slot)[:2]
            r.pose_seen=time.monotonic()
        n.armed=True  # test harness simulates the operator's explicit arm
    return n
def ok(c,msg):
    assert c, msg; print('PASS', msg)
def st(n): n.publish_status(); return json.loads(STATUS[-1])
def S(n): return {k: r.state for k, r in n.robots.items()}

# --- default real mode stays disarmed and requires measured configuration
PARAMS.clear()
try:
    fms.FmsController()
    raise AssertionError('default real mode accepted missing configuration')
except ValueError:
    print('PASS default real mode refuses missing measured coordinates')
PARAMS.update(points_json=json.dumps(fms.POINTS),
              home_slots_json=json.dumps([[0.6,0.2,1.57],[1.4,0.2,1.57]]),
              traffic_json=json.dumps({'corridor_axis':[[0,0],[10,0]],
                                       'entry_waypoint':[1,2], 'exit_waypoint':[3,4]}))
ACTIONS.clear()
n=fms.FmsController()
ok(n.dry_run is False and n.armed is False, 'default controller is real mode and disarmed')
n.call('delivery/request_a'); n.tick()
ok(all(not a.sent for a in ACTIONS.values()), 'default real mode sends no goals before arm')

# --- interface
n=mk(dry_run=True, dry_run_travel=0.0)
ok(list(n.robots)==['pinky_1','pinky_2'],'two robots by default')
for name in ('delivery/request_a','delivery/request_b','delivery/pinky_1/loaded','delivery/pinky_2/received',
             'delivery/pinky_1/retry','delivery/pinky_2/reset'):
    ok(name in n._srv, f'service {name}')
ok(set(ACTIONS)=={'/pinky_1/navigate_to_pose','/pinky_2/navigate_to_pose',
                  '/pinky_1/compute_path_to_pose','/pinky_2/compute_path_to_pose'},
   'one Nav2 navigation and path action client per robot')
ok('/pinky_2/amcl_pose' in n._sub,'pose subscription per robot (allocation cost)')
ok('/pinky_2/battery_state' in n._sub,'battery subscription per robot')
s0=st(n)
ok(set(s0['robots'])=={'pinky_1','pinky_2'} and s0['queue']==[] and s0['pickup'] is None and s0['points']['STORE'][0]==fms.POINTS['STORE'][0],
   'status: robots dict, queue, pickup holder, points')

# --- two requests: split across robots, pickup one at a time
r=n.call('delivery/request_a'); ok(r.success and 'position 1' in r.message,'request queued with position')
n.call('delivery/request_b'); n.tick()
ok(sorted(S(n).values())==['TO_STORE','WAIT_PICKUP'],
   'two requests -> one robot to the pickup, the other waits (WAIT_PICKUP)')
s1=st(n)
holder=s1['pickup']; other=[k for k in n.robots if k!=holder][0]
ok(s1['robots'][holder]['current']['robot']==holder and s1['robots'][other]['current']['station'] in 'AB',
   'assigned tasks carry the robot name')
ok(s1['queue']==[],'queue emptied by assignment')
n.tick()
ok(n.robots[holder].state=='WAIT_LOAD' and n.robots[other].state=='WAIT_PICKUP','holder arrives, other keeps waiting')
r=n.call(f'delivery/{other}/loaded'); ok(not r.success,'loaded on the waiting robot is refused')
n.call(f'delivery/{holder}/loaded')
ok(n.robots[holder].state=='TO_STATION' and n.pickup==holder,'loaded -> outbound goal, pickup remains locked')
n.tick()
ok(n.pickup is None and n.robots[other].state=='WAIT_PICKUP','outbound robot clears pickup, waiter still held')
n.tick()
ok(n.robots[other].state=='TO_STORE' and n.pickup==other,'waiting robot enters after clearance')
n.tick()
ok(n.robots[holder].state=='WAIT_RECEIVE' and n.robots[other].state=='WAIT_LOAD','both progress independently')
n.call(f'delivery/{holder}/received'); n.tick()
ok(n.robots[holder].state=='TO_HOME' and n.robots[holder].home_slot is not None,'delivered + empty queue -> back to the 수납대')
n.call(f'delivery/{other}/loaded'); n.tick(); n.tick(); n.call(f'delivery/{other}/received'); n.tick(); n.tick()
ok(S(n)=={'pinky_1':'IDLE','pinky_2':'IDLE'},'both back IDLE')
slots={r.home_slot for r in n.robots.values()}
ok(slots=={0,1},'shared 수납대: two parking slots')
p0,p1=n.home_slot_pose(0),n.home_slot_pose(1)
import math
ok(p0==fms.POINTS['HOME'] and abs(math.hypot(p1[0]-p0[0],p1[1]-p0[1])-0.35)<1e-9
   and abs((p1[0]-p0[0])*math.cos(p0[2])+(p1[1]-p0[1])*math.sin(p0[2]))<1e-9,'2nd slot 0.35 m to the side of HOME')

# --- allocation: nearest free robot to the pickup takes it
n=mk(dry_run=True, dry_run_travel=0.0)
sx,sy=fms.POINTS['STORE'][:2]; n.robots['pinky_1'].pose=(sx+3.0,sy); n.robots['pinky_2'].pose=(sx+0.5,sy)
n.call('delivery/request_a'); n.tick()
ok(n.robots['pinky_2'].state=='TO_STORE' and n.robots['pinky_1'].state=='IDLE','nearest robot to the pickup gets the task')
ok(fms.task_allocation.assign({'x':{'pose':None},'y':{'pose':(0,0)}},[{'id':1}],{'STORE':(1,0,0)})==[('y',{'id':1})],
   'unknown pose is chosen last')
ok(fms.task_allocation.pickup_order({'a':{'since':5},'b':{'since':3}},{})=='b','pickup: first come first served')

# --- busy robot not reassigned; return trip is not preempted by a conflicting goal
n=mk(dry_run=True, dry_run_travel=0.0)
n.call('delivery/request_a'); n.tick(); n.tick()
busy=n.pickup; idle=[k for k in n.robots if k!=busy][0]
n.call(f'delivery/{busy}/loaded'); n.tick(); n.call(f'delivery/{busy}/received'); n.tick()
ok(n.robots[busy].state=='TO_HOME','robot heading home')
n.robots[busy].pose=(fms.POINTS['STORE'][0]+0.2,fms.POINTS['STORE'][1])
n.robots[busy].nav_deadline=time.monotonic()+10
n.call('delivery/request_b'); n.tick()
ok(n.robots[busy].state=='TO_HOME' and n.robots[idle].state in ('TO_STORE', 'WAIT_PICKUP'),
   'TO_HOME robot keeps its goal; idle robot takes new task')

# --- error on one robot does not stop the other; lock released if not at the pickup
n=mk(dry_run=False)
n.call('delivery/request_a'); n.call('delivery/request_b'); n.tick()
h=n.pickup; w=[k for k in n.robots if k!=h][0]
act=ACTIONS[f'/{h}/navigate_to_pose']
ok(len(act.sent)==1 and abs(act.sent[0][0].pose.pose.position.x-fms.POINTS['STORE'][0])<1e-9,'real mode: goal to STORE on that robot\'s Nav2')
goal,f,hd=act.sent[0]; f.set(hd); hd.res.set(NS(status=6))
ok(n.robots[h].state=='ERROR' and n.pickup==h,'Nav2 abort keeps pickup locked until a new localization sample proves clearance')
n.robots[h].pose_seen=time.monotonic(); n.tick()
ok(n.pickup is None,'new localization sample outside pickup releases lock')
n.tick()
ok(n.robots[w].state=='TO_STORE' and n.pickup==w,'other robot continues into the pickup')
r=n.call(f'delivery/{h}/retry')
ok(r.success and n.robots[h].state=='WAIT_PICKUP','retry -> waits because the pickup is now busy')
r=n.call(f'delivery/{h}/reset'); n.tick()
ok(n.robots[h].current is None and n.robots[h].state in ('IDLE','TO_HOME'),'reset drops the task')

# --- timeout per robot
n=mk(dry_run=False, nav_timeout=0.001)
n.call('delivery/request_a'); n.tick(); time.sleep(0.003); n.tick()
h=n.pickup; goal,f,hd=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]
ok(n.robots[h].state=='STOPPING' and n.robots[h].goal_pending,
   'navigation timeout waits for pending Nav2 goal response')
f.set(hd)
ok(hd.cancelled and n.robots[h].state=='STOPPING', 'late accepted goal is cancelled')
hd.res.set(NS(status=5))
ok(n.robots[h].state=='ERROR' and 'timeout' in n.robots[h].error,
   'navigation timeout -> ERROR only after terminal Nav2 result')

# --- feedback distance per robot, stale callbacks ignored
n=mk(dry_run=False)
n.call('delivery/request_a'); n.tick()
h=n.pickup; act=ACTIONS[f'/{h}/navigate_to_pose']
act.fb(NS(feedback=NS(distance_remaining=1.234)))
ok(st(n)['robots'][h]['distance_remaining']==1.23,'Nav2 distance_remaining per robot')
goal,f,hd=act.sent[0]; f.set(hd)
n.call(f'delivery/{h}/reset')
ok(n.robots[h].state=='STOPPING', 'reset waits for goal cancellation')
hd.res.set(NS(status=4))
ok(n.robots[h].state in ('IDLE','TO_HOME'),'late result after reset ignored')

# --- two requests for the same station never overlap
n=mk(dry_run=True, dry_run_travel=0.0)
n.call('delivery/request_a'); n.call('delivery/request_a'); n.tick()
owner=n.station_owners['A']; other=[name for name in n.robots if name!=owner][0]
ok(owner is not None and n.robots[other].state=='IDLE' and len(n.queue)==1,
   'second request to occupied station remains queued')
n.tick(); n.call(f'delivery/{owner}/loaded'); n.tick(); n.tick()
n.call(f'delivery/{owner}/received'); n.tick()
ok(n.station_owners['A']==owner and len(n.queue)==1,
   'receiving a delivery does not release the station while robot is still there')
n.tick()
ok(n.station_owners['A'] is not None and not n.queue and n.robots[n.station_owners['A']].state=='TO_STORE',
   'station releases after departure and the second request is then assigned')

# --- real mode is inert until explicitly armed, and stale localization blocks motion
n=mk(dry_run=False)
n.armed=False
n.call('delivery/request_a'); n.tick()
ok(all(not a.sent for a in ACTIONS.values()) and len(n.queue)==1,
   'real mode does not send a goal before arm')
n.robots['pinky_2'].pose_seen=time.monotonic()-n.pose_timeout-1
ok(not n.call('delivery/arm').success, 'arm rejects stale localization')
n.robots['pinky_2'].pose_seen=time.monotonic()
ok(n.call('delivery/arm').success, 'arm accepts ready robots and clear shared areas')
n.tick()
ok(sum(len(a.sent) for name,a in ACTIONS.items() if name.endswith('/navigate_to_pose'))==1,
   'arm permits exactly one pickup goal')
h=n.pickup; n.robots[h].pose_seen=time.monotonic()-n.pose_timeout-1
n.tick()
ok(n.robots[h].state=='STOPPING', 'localization loss during navigation requests Nav2 cancellation')
n=mk(dry_run=True, dry_run_travel=0.0)
n.call('delivery/request_a'); n.tick(); n.tick(); h=n.pickup
n.call('delivery/disarm')
ok(not n.call(f'delivery/{h}/loaded').success and n.robots[h].state=='WAIT_LOAD',
   'disarm blocks loaded service from sending another navigation goal')
ok(n.call('delivery/arm').success, 'stopped task can be armed again while it owns pickup')
ok(n.call(f'delivery/{h}/loaded').success, 'loaded service resumes after arm')
n=mk(dry_run=False)
n.call('delivery/request_a'); n.tick(); h=n.pickup
_,f,hd=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]; f.set(hd); hd.res.set(NS(status=4))
n.robots[h].pose_seen=time.monotonic()-n.pose_timeout-1
ok(not n.call(f'delivery/{h}/loaded').success and n.robots[h].state=='WAIT_LOAD',
   'stale localization blocks outbound goal after loading')
with tempfile.NamedTemporaryFile(mode='w+', suffix='.json') as points_file:
    json.dump({k:list(v) for k,v in fms.POINTS.items()}, points_file); points_file.flush()
    n=mk(dry_run=False, points_file=points_file.name, points_json='')
    ok(n.points==fms.POINTS, 'measured points file is loaded for real mode')
try:
    mk(dry_run=False, points_json='', points_file='')
    raise AssertionError('real mode accepted example points')
except ValueError:
    print('PASS real mode refuses example coordinates without explicit points')
try:
    mk(dry_run=False, home_slots_json='', home_slots_file='')
    raise AssertionError('real mode accepted unmeasured home slots')
except ValueError:
    print('PASS real two-robot mode refuses unmeasured home slots')

# --- an unconfirmed result keeps the robot and pickup locked
n=mk(dry_run=False)
n.call('delivery/request_a'); n.tick(); h=n.pickup
_,f,hd=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]; f.set(hd)
hd.res.result=lambda: (_ for _ in ()).throw(RuntimeError('DDS dropped'))
hd.res.set(None)
ok(n.robots[h].state=='ERROR' and n.pickup==h and n.robots[h].stop_reason is not None,
   'unconfirmed result fails closed and keeps the shared pickup locked')
ok(not n.call(f'delivery/{h}/reset').success and not n.call(f'delivery/{h}/retry').success,
   'unconfirmed result cannot be reset or retried through normal services')
n=mk(dry_run=False)
n.call('delivery/request_a'); n.tick(); h=n.pickup
_,f,_=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]
f.result=lambda: (_ for _ in ()).throw(RuntimeError('DDS dropped'))
f.set(None)
ok(n.robots[h].state=='ERROR' and n.pickup==h and n.robots[h].stop_reason is not None,
   'unconfirmed goal acceptance also fails closed')

# --- docking
n=mk(dry_run=True, dry_run_travel=0.0, docking=True, dry_run_dock=0.0)
ok('/pinky_1/delivery/dock' in n._sub and '/pinky_2/delivery/dock_start' in CLIENTS,'docking: per-robot start service + feedback topic')
n.call('delivery/request_a'); n.tick(); n.tick()
h=n.pickup
ok(n.robots[h].state=='DOCKING' and st(n)['robots'][h]['target']=='STORE','Nav2 arrival -> DOCKING (target STORE)')
n.tick(); ok(n.robots[h].state=='WAIT_LOAD','dry run: docking done -> WAIT_LOAD')
n=mk(dry_run=False, docking=True)
n.call('delivery/request_a'); n.tick(); h=n.pickup
goal,f,hd=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]; f.set(hd); hd.res.set(NS(status=4))
n.robots[h].pose=fms.POINTS['STORE'][:2]; n.robots[h].pose_seen=time.monotonic()
ok(n.robots[h].state=='DOCKING' and CLIENTS[f'/{h}/delivery/dock_start'].calls==1,'real: dock_start called')
n._sub[f'/{h}/delivery/dock'](NS(data=json.dumps({'distance':0.1,'angle_deg':3,'visible':True,'state':'running'})))
ok(n.robots[h].state=='DOCKING','running feedback keeps DOCKING')
n._sub[f'/{h}/delivery/dock'](NS(data=json.dumps({'state':'failed','reason':'marker lost'})))
ok(n.robots[h].state=='ERROR' and 'marker lost' in n.robots[h].error and n.pickup==h,
   'docking failed -> ERROR, pickup kept (robot is there)')
r=n.call(f'delivery/{h}/retry')
ok(r.success and n.robots[h].state=='DOCKING' and CLIENTS[f'/{h}/delivery/dock_start'].calls==2,'retry docking')
n._sub[f'/{h}/delivery/dock'](NS(data=json.dumps({'state':'done'})))
ok(n.robots[h].state=='WAIT_LOAD','docking done -> WAIT_LOAD')
Client.ready=False
n=mk(dry_run=False, docking=True)
n.call('delivery/request_a'); n.tick(); h=n.pickup
goal,f,hd=ACTIONS[f'/{h}/navigate_to_pose'].sent[0]; f.set(hd); hd.res.set(NS(status=4))
ok(n.robots[h].state=='ERROR' and 'Docking service' in n.robots[h].error,'docking service missing -> ERROR')
Client.ready=True

# --- params
n=mk(dry_run=True, robots='a,b,c', nav_action='/{robot}/nav', home_slot_offset=0.0)
ok(list(n.robots)==['a','b','c'] and '/c/nav' in ACTIONS,'robots and nav_action params')
ok(n.home_slot_pose(1)==n.home_slot_pose(0),'home_slot_offset 0 -> same HOME pose')

# --- real battery data participates in allocation; missing/low data is not eligible
n=mk(dry_run=False, battery_required=True)
for name, value in (('pinky_1', 0.21), ('pinky_2', 0.90)):
    n._sub[f'/{name}/battery_state'](NS(percentage=value))
n.call('delivery/request_a'); n.tick()
ok(n.pickup=='pinky_2', 'higher fresh battery wins when distances are similar')
ok(st(n)['robots']['pinky_2']['battery_percent']==0.9, 'battery percentage is published to GUI status')
n=mk(dry_run=False, battery_required=True)
n._sub['/pinky_1/battery_state'](NS(percentage=0.10))
n.call('delivery/request_a'); n.tick()
ok(n.pickup is None and len(n.queue)==1, 'low and missing battery robots receive no task')
n=mk(dry_run=False, battery_required=True, battery_timeout=1.0)
r=n.robots['pinky_1']; r.state='TO_STORE'; r.battery_percent=0.8
r.battery_seen=time.monotonic()-2.0
n.tick()
ok(r.state=='ERROR' and 'Battery telemetry stale' in r.error,
   'stale battery telemetry stops an active trip fail-closed')

# --- Nav2 ComputePathToPose has priority over Euclidean fallback
n=mk(dry_run=False, path_query_wait=1.0)
n.call('delivery/request_a'); n.tick()
ok(n.pickup is None, 'allocation briefly waits for Nav2 path costs')
def fake_path(points):
    return NS(poses=[NS(pose=NS(position=NS(x=x,y=y))) for x,y in points])
for name, points in (('pinky_1', [(0,0),(10,0)]), ('pinky_2', [(0,0),(2,0)])):
    _, future, handle = ACTIONS[f'/{name}/compute_path_to_pose'].sent[0]
    future.set(handle)
    handle.res.set(NS(result=NS(path=fake_path(points))))
n.tick()
ok(n.pickup=='pinky_2' and n.robots['pinky_2'].path_source=='nav2_path',
   'shorter real Nav2 path wins allocation')

# --- traffic reservation is integrated before a Nav2 goal
traffic_cfg=json.dumps({'corridor_axis':[[0,0],[10,0]], 'entry_angle_deg':45,
                        'exit_angle_deg':135, 'allow_same_direction_following':False})
n=mk(dry_run=True, dry_run_travel=10.0, traffic_enabled=True, traffic_json=traffic_cfg)
p1,p2=n.robots['pinky_1'],n.robots['pinky_2']
ok(n.schedule_move(p1,'STORE','TO_STORE'), 'first move obtains traffic reservation')
ok(not n.schedule_move(p2,'A','TO_STATION') and p2.state=='WAIT_TRAFFIC',
   'opposite move waits before Nav2 goal')
n.arrived(p1); n.tick()
ok(p2.state=='TO_STATION', 'waiting opposite move starts after corridor release')
waypoint_cfg=json.dumps({'corridor_axis':[[0,0],[10,0]], 'entry_angle_deg':45,
                           'exit_angle_deg':135, 'entry_waypoint':[1,2],
                           'exit_waypoint':[3,4]})
n=mk(dry_run=True, dry_run_travel=10.0, traffic_enabled=True, traffic_json=waypoint_cfg)
p1=n.robots['pinky_1']
n.schedule_move(p1, 'STORE', 'TO_STORE')
ok(p1.goal_pose[:2] == (1.0, 2.0) and p1.traffic_waypoint_pending,
   'entry move first targets measured 45-degree gate waypoint')
n.arrived(p1)
ok(p1.state=='TO_STORE' and p1.goal_pose==n.points['STORE'] and p1.name in n.traffic.occupants,
   'gate arrival advances to destination without releasing corridor')
n.arrived(p1)
ok(p1.state=='WAIT_LOAD' and p1.name not in n.traffic.occupants,
   'corridor releases only after final destination')
try:
    mk(dry_run=False, traffic_enabled=True, traffic_json='', traffic_file='')
    raise AssertionError('real traffic started without measured config')
except ValueError:
    print('PASS real traffic requires measured corridor config')


# --- external allocator (task_allocation/allocator.py) through the bridge
import os as _os, tempfile as _tf, importlib.util as _iu
def _bridge(code):
    d=_tf.mkdtemp(); f=_os.path.join(d,'allocator.py'); open(f,'w').write(code)
    _os.environ['PINKY_ALLOCATOR']=f
    sp=_iu.spec_from_file_location('ta_test', _os.path.join(_os.path.dirname(_os.path.abspath(sys.argv[1])),'task_allocation.py'))
    m=_iu.module_from_spec(sp); sp.loader.exec_module(m); return m
FREE={'pinky_1':{'pose':(0,0)},'pinky_2':{'pose':(5,5)}}; Q=[{'id':1,'station':'A'}]; PT={'STORE':(0,0,0)}
m=_bridge("def assign(free, queue, points):\n    return [('pinky_2', queue[0])]\ndef pickup_order(w, p):\n    return sorted(w)[-1]")
ok(m.SOURCE=='task_allocation/allocator.py' and m.assign(FREE,Q,PT)==[('pinky_2',Q[0])],'external allocator used')
ok(m.pickup_order({'a':{'since':1},'b':{'since':2}},PT)=='b','external pickup_order used')
m=_bridge("def assign(free, queue, points):\n    raise RuntimeError('bug')")
ok(m.assign(FREE,Q,PT)==[('pinky_1',Q[0])],'allocator error -> default rules')
m=_bridge("def assign(free, queue, points):\n    return [('ghost', queue[0])]")
ok(m.assign(FREE,Q,PT)==[('pinky_1',Q[0])],'invalid robot name -> default rules')
m=_bridge("def assign(free, queue, points):\n    return [('pinky_1', queue[0]), ('pinky_2', queue[0])]")
ok(m.assign(FREE,Q,PT)==[('pinky_1',Q[0])],'duplicate task assignment -> default rules')
m=_bridge("this is not python")
ok(m.SOURCE=='default' and m.assign(FREE,Q,PT)==[('pinky_1',Q[0])],'broken file -> default rules')
_os.environ['PINKY_ALLOCATOR']='/nonexistent/allocator.py'

# --- launcher arguments reach ROS init; shutdown cancellation remains in place
from unittest.mock import patch
entry_args=['fms_sim.py', '--ros-args', '-p', 'dry_run:=true']
entry_events=[]
entry_node=NS(dry_run=True,
              safe_shutdown=lambda: entry_events.append('cancel'),
              destroy_node=lambda: entry_events.append('destroy'))
with patch.object(fms, 'FmsController', return_value=entry_node), \
     patch.object(fms.rclpy, 'init') as init_call, \
     patch.object(fms.rclpy, 'spin'), \
     patch.object(fms.rclpy, 'shutdown'), \
     patch.object(fms.signal, 'signal'):
    fms.main(entry_args)
    ok(init_call.call_args.kwargs['args']==entry_args,
       'entry point forwards simulation arguments to ROS init')
    ok(init_call.call_args.kwargs['signal_handler_options']==fms.SignalHandlerOptions.NO
       and entry_events==['cancel', 'destroy'], 'entry point preserves cancellation on shutdown')

# --- status samples for the GUI check (python3 test_fms.py fms_controller.py out.json)
if len(sys.argv) > 2:
    n=mk(dry_run=True, dry_run_travel=0.0, docking=True, dry_run_dock=0.0)
    samples=[]
    def snap(): samples.append(st(n))
    snap()
    n.call('delivery/request_a'); n.call('delivery/request_b'); n.tick(); snap()
    n.tick(); snap()                                   # DOCKING + WAIT_PICKUP
    n.tick(); snap()                                   # WAIT_LOAD + WAIT_PICKUP
    h=n.pickup; n.call(f'delivery/{h}/loaded'); n.tick(); snap()
    n.tick(); snap()
    json.dump(samples, open(sys.argv[2],'w'))
print('ALL PASS')
