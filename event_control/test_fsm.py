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
    def create_timer(s,p,cb): return cb
    def get_logger(s):
        return types.SimpleNamespace(info=lambda m:None, warn=lambda m:None, error=lambda m:LOG.append(m))
    def get_clock(s):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))
    def call(s,name):
        return s._srv[name](None, types.SimpleNamespace(success=None,message=''))

class Handle:
    def __init__(s): s.accepted=True; s.res=Fut(); s.cancelled=False
    def get_result_async(s): return s.res
    def cancel_goal_async(s): s.cancelled=True

class ActionClient:
    ready=True
    def __init__(s,*a): s.sent=[]
    def server_is_ready(s): return ActionClient.ready
    def send_goal_async(s,goal,feedback_callback=None):
        s.fb=feedback_callback
        f=Fut(); h=Handle(); s.sent.append((goal,f,h)); return f

class G:
    def __init__(s):
        P=lambda: types.SimpleNamespace(x=0,y=0,z=0,w=0)
        s.pose=types.SimpleNamespace(header=types.SimpleNamespace(frame_id=None,stamp=None),
                                     pose=types.SimpleNamespace(position=P(),orientation=P()))

def mod(n,**kw):
    m=types.ModuleType(n); m.__dict__.update(kw); sys.modules[n]=m
mod('rclpy', init=lambda:None, spin=lambda n:None, ok=lambda:True, shutdown=lambda:None)
mod('rclpy.action', ActionClient=ActionClient); mod('rclpy.node', Node=Node)
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
mod('nav2_msgs'); mod('nav2_msgs.action', NavigateToPose=types.SimpleNamespace(Goal=G))
mod('std_msgs'); mod('std_msgs.msg', String=lambda data: types.SimpleNamespace(data=data))
mod('std_srvs'); mod('std_srvs.srv', Trigger=object)

import tempfile, os
PTS=os.path.join(tempfile.mkdtemp(),'pts.json')
PARAMS={'points_file':PTS}; STATUS=[]; LOG=[]
spec=importlib.util.spec_from_file_location('pd', sys.argv[1]); pd=importlib.util.module_from_spec(spec); spec.loader.exec_module(pd)

def mk(**p):
    PARAMS.clear(); PARAMS.update(p); return pd.PartsDelivery()
def ok(c,msg):
    assert c, msg; print('PASS', msg)

# 1. dry_run full flow + queue + TO_HOME preempt
n=mk(dry_run=True, dry_run_travel=0.0)
n.call('delivery/request_a'); n.tick(); ok(n.state=='TO_STORE','dry: start to store')
n.tick(); ok(n.state=='WAIT_LOAD','dry: arrive store')
n.call('delivery/request_b')
ok(n.call('delivery/received').success is False,'received rejected in WAIT_LOAD')
n.call('delivery/loaded'); n.tick(); ok(n.state=='WAIT_RECEIVE','dry: arrive A')
n.call('delivery/received'); ok(n.state=='IDLE' and n.current is None,'done A, B queued -> IDLE')
n.tick(); ok(n.current['station']=='B' and n.state=='TO_STORE','B starts')
n.tick(); n.call('delivery/loaded'); n.tick(); n.call('delivery/received')
ok(n.state=='TO_HOME','returning home')
n.call('delivery/request_a'); n.tick(); ok(n.state=='TO_STORE','TO_HOME preempted by new request')

# 2. real mode: wait for nav2, success, failure, retry, timeout, reset
ActionClient.ready=False
n=mk(dry_run=False, nav_timeout=0.05)
n.call('delivery/request_a'); n.tick(); ok(n.state=='IDLE' and len(n.queue)==1,'waits while Nav2 not ready (no ERROR)')
ActionClient.ready=True; n.tick(); ok(n.state=='TO_STORE','starts when Nav2 ready')
g,f,h=n.nav.sent[-1]; f.set(h); h.res.set(types.SimpleNamespace(status=4))
ok(n.state=='WAIT_LOAD','real: SUCCEEDED -> WAIT_LOAD')
n.call('delivery/loaded'); g,f,h=n.nav.sent[-1]; f.set(h)
h.res.set(types.SimpleNamespace(status=6)); ok(n.state=='ERROR' and 'A' in n.error,'aborted -> ERROR')
r=n.call('delivery/request_b'); ok(r.success and len(n.queue)==1,'requests still queued during ERROR')
r=n.call('delivery/retry'); ok(r.success and n.state=='TO_STATION' and n.target=='A','retry resends goal to A')
g,f,h=n.nav.sent[-1]; f.set(h); time.sleep(0.06); n.tick()
ok(n.state=='ERROR' and h.cancelled and 'timeout' in n.error,'timeout cancels goal -> ERROR')
h.res.set(types.SimpleNamespace(status=4)); ok(n.state=='ERROR','late result of timed-out goal ignored')
r=n.call('delivery/reset'); ok(n.state=='IDLE' and n.current is None and len(n.queue)==1,'reset drops current, keeps queue')
n.tick(); ok(n.current['station']=='B' and n.state=='TO_STORE','next task runs after reset')
ok(n.call('delivery/retry').success is False,'retry rejected outside ERROR')
n.nav.fb(types.SimpleNamespace(feedback=types.SimpleNamespace(distance_remaining=1.234)))
ok(n.distance_remaining==1.23,'Nav2 feedback -> distance_remaining')
n.tick(); import json; ok(json.loads(STATUS[-1])['distance_remaining']==1.23,'distance in status JSON')
# --- fixed points, no registration
import json
n=mk(dry_run=True, dry_run_travel=0.0)
ok(not any('register' in k for k in n._srv) and not getattr(n,'_sub',{}),'no registration services/subscriptions')
ok(set(pd.POINTS)=={'STORE','A','B','HOME'},'fixed points: pickup, 2 workstations, one 수납대')
n.tick(); st=json.loads(STATUS[-1])
ok(st['robot']=='pinky_1' and st['home']=='HOME' and 'register' not in st and round(st['points']['STORE'][0], 3)==round(pd.POINTS['STORE'][0], 3),
   'status: robot name, home point, points')
n.call('delivery/request_a'); n.tick(); n.tick(); n.call('delivery/loaded'); n.tick(); n.call('delivery/received')
ok(n.state=='TO_HOME' and n.target=='HOME','returns to the 수납대 (HOME)')
print('ALL PASS')
