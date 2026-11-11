import sys, types, time, importlib.util

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
    def cancel_goal_async(s): s.cancelled=True

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
class CG:
    def __init__(s): s.goal=G().pose

def mod(n,**kw):
    m=types.ModuleType(n); m.__dict__.update(kw); sys.modules[n]=m
mod('rclpy', init=lambda:None, spin=lambda n:None, ok=lambda:True, shutdown=lambda:None)
mod('rclpy.action', ActionClient=ActionClient); mod('rclpy.node', Node=Node)
mod('action_msgs'); mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_ABORTED=6))
mod('geometry_msgs'); mod('geometry_msgs.msg', PoseStamped=object, PointStamped=object, PoseWithCovarianceStamped=object)
mod('sensor_msgs'); mod('sensor_msgs.msg', BatteryState=object)
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
                      ComputePathToPose=types.SimpleNamespace(Goal=CG))
mod('std_msgs'); mod('std_msgs.msg', String=lambda data: types.SimpleNamespace(data=data))
mod('std_srvs'); mod('std_srvs.srv', Trigger=object)


mod('rclpy.signals', SignalHandlerOptions=types.SimpleNamespace(NO=0))
mod('std_srvs.srv', Trigger=types.SimpleNamespace(Request=lambda: None))
import json, os, math
HERE=os.path.dirname(os.path.abspath(sys.argv[1]))
ALLOC=os.path.join(os.path.dirname(HERE),'task_allocation','allocator.py')
PARAMS={}; STATUS=[]; LOG=[]
spec=importlib.util.spec_from_file_location('fms', sys.argv[1]); fms=importlib.util.module_from_spec(spec); spec.loader.exec_module(fms)
NS=types.SimpleNamespace
def bridge(path):
    os.environ['PINKY_ALLOCATOR']=path
    sp=importlib.util.spec_from_file_location('task_allocation_t', os.path.join(HERE,'task_allocation.py'))
    m=importlib.util.module_from_spec(sp); sp.loader.exec_module(m); fms.task_allocation=m; return m
def mk(**p):
    if p.get('dry_run') is False:
        p.setdefault('traffic_enabled', False)
        p.setdefault('battery_required', False)
        p.setdefault('path_query_wait', 0.0)
    PARAMS.clear(); PARAMS.update(p); ACTIONS.clear(); CLIENTS.clear(); return fms.FmsController()
def ok(c,msg):
    assert c, msg; print('PASS', msg)
def S(n): return {k: r.state for k, r in n.robots.items()}
def ticks(n,k=3):
    for _ in range(k): n.tick()
PT=fms.POINTS

m=bridge(ALLOC)
ok(m.SOURCE.endswith('(Allocator class)'),'bridge loads the Allocator class')

# --- dispatch: closer free robot starts, second robot waits for the pickup, station locked
n=mk(dry_run=True, dry_run_travel=100.0, detour_factor=1.0)
p1,p2=n.robots['pinky_1'],n.robots['pinky_2']
p2.pose=(PT['STORE'][0], PT['STORE'][1]-1.0)
n.call('delivery/request_a'); n.tick()
ok(p2.state=='TO_STORE' and p1.state=='IDLE' and n.station_owners['A']=='pinky_2','Allocator: robot closer to the pickup starts, station A locked')
ok(m._cls_alloc.cfg.v==0.15 and m._cls_alloc.cfg.t_load_s==20.0 and m._cls_alloc.cfg.t_receive_s==15.0 and m._cls_alloc.cfg.t_align_s==0.0,
   'Allocator config from controller parameters (no align time without docking)')
n.call('delivery/request_a'); n.tick()
ok(len(n.queue)==1 and p1.state=='IDLE','second request to a locked station stays queued')
n.call('delivery/request_b'); n.tick()
ok(p1.current and p1.current['station']=='B' and p1.state=='WAIT_PICKUP','free robot takes B and waits for the pickup')

# --- next_for: after 수령 완료 the robot takes the next assignable request (no trip home)
n=mk(dry_run=True, dry_run_travel=0.0, robots='pinky_1')
r=n.robots['pinky_1']
n.call('delivery/request_a'); ticks(n,2)
ok(r.state=='WAIT_LOAD','at the pickup'); n.call('delivery/pinky_1/loaded'); ticks(n,2)
ok(r.state=='WAIT_RECEIVE' and n.pickup is None,'at 작업대1, pickup released')
n.call('delivery/request_b'); n.tick()
ok(len(n.queue)==1,'request B queued while busy')
n.call('delivery/pinky_1/received')
ok(r.state=='TO_STORE' and r.current['station']=='B' and not n.queue and n.station_owners['B']=='pinky_1',
   'Allocator.next_for: 수령 완료 -> straight to the pickup for B')
ticks(n,2); n.call('delivery/pinky_1/loaded'); ticks(n,2)
ok(r.state=='WAIT_RECEIVE' and n.station_owners['A'] is None,'station A released after the robot left it')
n.call('delivery/pinky_1/received'); ticks(n,1)
ok(r.state=='TO_HOME','queue empty -> back to the 수납대')
# same-station request is not chained (station still locked by this robot -> no deadlock)
ticks(n,2); n.call('delivery/request_a'); ticks(n,2); n.call('delivery/pinky_1/loaded'); ticks(n,2)
n.call('delivery/request_a'); n.call('delivery/pinky_1/received')
ok(r.state=='IDLE' and len(n.queue)==1,'request to the same station is not chained')
ticks(n,1); ok(r.state=='TO_HOME','robot leaves the station (GitHub deadlock rule kept)')
ticks(n,4); ok(r.current is not None and r.current['station']=='A','after leaving, the queued A request is assigned')

# --- real mode: nothing is dispatched until /delivery/arm
pts=json.dumps({k:list(v) for k,v in PT.items()})
slots=json.dumps([[0.6,0.2,1.57],[1.2,0.2,1.57]])
n=mk(dry_run=False, points_json=pts, home_slots_json=slots)
for name,(x,y) in (('pinky_1',(1.3,0.2)),('pinky_2',(1.9,0.2))):  # HOME slot 0 is 0.51 m from B (< station_clearance)
    n._sub[f'/{name}/amcl_pose'](NS(header=NS(frame_id='map'),pose=NS(pose=NS(position=NS(x=x,y=y)))))
n.call('delivery/request_a'); n.tick()
ok(not any(a.sent for a in ACTIONS.values()) and len(n.queue)==1,'real mode: not armed -> no goal')
res=n.call('delivery/arm'); n.tick()
sent=[k for k,a in ACTIONS.items() if a.sent and k.endswith('/navigate_to_pose')]
ok(res.success and len(sent)==1 and not n.queue,'armed -> Allocator assigns, one Nav2 goal sent')

# --- STOPPING / ERROR robots are never chosen
n=mk(dry_run=True, dry_run_travel=100.0)
n.robots['pinky_1'].state='STOPPING'
n.call('delivery/request_b'); n.tick()
ok(n.robots['pinky_2'].current is not None and n.robots['pinky_1'].current is None,'STOPPING robot skipped')

# --- allocator missing -> default rules
bridge('/nonexistent/allocator.py')
n=mk(dry_run=True, dry_run_travel=100.0)
n.call('delivery/request_a'); n.tick()
ok(fms.task_allocation.SOURCE=='default' and sum(1 for r in n.robots.values() if r.current)==1,'allocator missing -> default rules still assign')
print('ALL PASS')
