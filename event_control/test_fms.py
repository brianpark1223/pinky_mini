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


mod('std_srvs.srv', Trigger=types.SimpleNamespace(Request=lambda: None))
import json
PARAMS={}; STATUS=[]; LOG=[]
spec=importlib.util.spec_from_file_location('fms', sys.argv[1]); fms=importlib.util.module_from_spec(spec); spec.loader.exec_module(fms)
NS=types.SimpleNamespace

def mk(**p):
    PARAMS.clear(); PARAMS.update(p); ACTIONS.clear(); CLIENTS.clear(); return fms.FmsController()
def ok(c,msg):
    assert c, msg; print('PASS', msg)
def st(n): n.publish_status(); return json.loads(STATUS[-1])
def S(n): return {k: r.state for k, r in n.robots.items()}

# --- interface
n=mk(dry_run=True, dry_run_travel=0.0)
ok(list(n.robots)==['pinky_1','pinky_2'],'two robots by default')
for name in ('delivery/request_a','delivery/request_b','delivery/pinky_1/loaded','delivery/pinky_2/received',
             'delivery/pinky_1/retry','delivery/pinky_2/reset'):
    ok(name in n._srv, f'service {name}')
ok(set(ACTIONS)=={'/pinky_1/navigate_to_pose','/pinky_2/navigate_to_pose'},'one Nav2 action client per robot')
ok('/pinky_2/amcl_pose' in n._sub,'pose subscription per robot (allocation cost)')
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
ok(n.robots[holder].state=='TO_STATION' and n.pickup is None,'loaded -> leaves, pickup released')
n.tick()
ok(n.robots[other].state in ('TO_STORE','WAIT_LOAD') and n.pickup==other,'waiting robot enters the pickup')
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

# --- busy robot not reassigned, TO_HOME preempted
n=mk(dry_run=True, dry_run_travel=0.0)
n.call('delivery/request_a'); n.tick(); n.tick()
busy=n.pickup; idle=[k for k in n.robots if k!=busy][0]
n.call(f'delivery/{busy}/loaded'); n.tick(); n.call(f'delivery/{busy}/received'); n.tick()
ok(n.robots[busy].state=='TO_HOME','robot heading home')
n.robots[busy].pose=(fms.POINTS['STORE'][0]+0.2,fms.POINTS['STORE'][1])
n.call('delivery/request_b'); n.tick()
ok(n.robots[busy].state=='TO_STORE' and n.robots[busy].home_slot is None,'TO_HOME robot can be preempted by a new request (cost)')

# --- error on one robot does not stop the other; lock released if not at the pickup
n=mk(dry_run=False)
n.call('delivery/request_a'); n.call('delivery/request_b'); n.tick()
h=n.pickup; w=[k for k in n.robots if k!=h][0]
act=ACTIONS[f'/{h}/navigate_to_pose']
ok(len(act.sent)==1 and abs(act.sent[0][0].pose.pose.position.x-fms.POINTS['STORE'][0])<1e-9,'real mode: goal to STORE on that robot\'s Nav2')
goal,f,hd=act.sent[0]; f.set(hd); hd.res.set(NS(status=6))
ok(n.robots[h].state=='ERROR' and n.pickup is None,'Nav2 abort -> ERROR, pickup released (not there yet)')
n.tick()
ok(n.robots[w].state=='TO_STORE' and n.pickup==w,'other robot continues into the pickup')
r=n.call(f'delivery/{h}/retry')
ok(r.success and n.robots[h].state=='WAIT_PICKUP','retry -> waits because the pickup is now busy')
r=n.call(f'delivery/{h}/reset'); n.tick()
ok(n.robots[h].current is None and n.robots[h].state in ('IDLE','TO_HOME'),'reset drops the task')

# --- timeout per robot
n=mk(dry_run=False, nav_timeout=0.0)
n.call('delivery/request_a'); n.tick(); n.tick()
ok(any(r.state=='ERROR' and 'timeout' in r.error for r in n.robots.values()),'navigation timeout -> ERROR on that robot')

# --- feedback distance per robot, stale callbacks ignored
n=mk(dry_run=False)
n.call('delivery/request_a'); n.tick()
h=n.pickup; act=ACTIONS[f'/{h}/navigate_to_pose']
act.fb(NS(feedback=NS(distance_remaining=1.234)))
ok(st(n)['robots'][h]['distance_remaining']==1.23,'Nav2 distance_remaining per robot')
goal,f,hd=act.sent[0]; f.set(hd)
n.call(f'delivery/{h}/reset')
hd.res.set(NS(status=4))
ok(n.robots[h].state in ('IDLE','TO_HOME'),'late result after reset ignored')

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
