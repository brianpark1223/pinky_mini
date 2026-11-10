# ROS 없이 notifier 검증: python3 test_notifier.py robot_notifier.py
import sys, types, time, json, math, importlib.util
class Node:
    def __init__(s,name): s.subs={}; s.logs=[]
    def declare_parameter(s,n,v): return types.SimpleNamespace(value=PARAMS.get(n,v))
    def create_subscription(s,t,topic,cb,q): s.subs[topic]=cb
    def get_logger(s):
        L=lambda m: s.logs.append(m)
        return types.SimpleNamespace(info=L,warn=L,error=L)
def mod(n,**kw):
    m=types.ModuleType(n); m.__dict__.update(kw); sys.modules[n]=m
mod('rclpy', init=lambda:None, spin=lambda n:None, ok=lambda:True, shutdown=lambda:None)
mod('rclpy.node', Node=Node); mod('rclpy.qos', qos_profile_sensor_data=None)
mod('sensor_msgs'); mod('sensor_msgs.msg', LaserScan=object)
mod('std_msgs'); mod('std_msgs.msg', String=object)
PARAMS={}
spec=importlib.util.spec_from_file_location('rn', sys.argv[1]); rn=importlib.util.module_from_spec(spec); spec.loader.exec_module(rn)

PLAYED=[]
class FakePlayer:
    def __init__(s,*a): s.stopped=[]
    def play(s,k,p): PLAYED.append(k)
    def stop(s,k): s.stopped.append(k)

def ok(c,m): assert c,m; print('PASS',m)
def scan(front, side=5.0):
    # 360 rays, 1 deg each, angle_min=-pi ; index 180 = 0 deg
    r=[side]*360
    for d in range(-10,11): r[180+d]=front
    return types.SimpleNamespace(angle_min=-math.pi, angle_increment=math.radians(1), range_min=0.05, range_max=12.0, ranges=r)

# geometry helper
ok(abs(rn.front_min_distance(scan(0.3),50,0,0.08)-0.3)<1e-9,'front object detected')
s=scan(5.0); s.ranges[180+90]=0.2   # object at +90deg (left)
ok(rn.front_min_distance(s,50,0,0.08)==5.0,'side object ignored')
ok(abs(rn.front_min_distance(s,50,90,0.08)-0.2)<1e-9,'front_offset rotates sector')
s=scan(0.03); ok(rn.front_min_distance(s,50,0,0.08)==5.0,'too-close (robot body) ignored')
s=scan(float('inf')); ok(rn.front_min_distance(s,50,0,0.08)==5.0,'inf ignored')

n=rn.RobotNotifier(); n.player=FakePlayer(); n.beep_interval=0.0
st=lambda **k: n.subs['/delivery/status'](types.SimpleNamespace(data=json.dumps(k)))
sc=lambda v: n.subs['/scan'](scan(v))

st(state='IDLE'); sc(0.2); ok(PLAYED==[],'no beep when IDLE')
st(state='TO_STORE', distance_remaining=3.0); sc(1.0); ok(PLAYED==[],'no beep when far')
sc(0.3); ok(PLAYED==['beep'],'beep when close while moving')
st(state='TO_STORE', distance_remaining=0.3); sc(0.3); ok(PLAYED==['beep'],'muted near goal')
st(state='WAIT_LOAD'); ok(PLAYED[-1]=='song','song on arrival at pickup')
sc(0.1); ok(PLAYED.count('beep')==1,'no beep while waiting')
st(state='WAIT_LOAD'); ok(PLAYED.count('song')==1,'no immediate repeat')
n.last_song-=100; st(state='WAIT_LOAD'); ok(PLAYED.count('song')==2,'song repeats after song_repeat')
st(state='TO_STATION', distance_remaining=None); ok('song' in n.player.stopped,'song stopped when leaving')
sc(0.3); ok(PLAYED[-1]=='beep','beep with unknown distance (dry_run)')
st(state='WAIT_RECEIVE'); ok(PLAYED[-1]=='song','song at workstation')
n2=rn.RobotNotifier(); n2.player=FakePlayer(); n2.beep_interval=10
st2=lambda **k: n2.subs['/delivery/status'](types.SimpleNamespace(data=json.dumps(k)))
PLAYED.clear(); st2(state='TO_HOME'); n2.subs['/scan'](scan(.2)); n2.subs['/scan'](scan(.2))
ok(PLAYED==['beep'],'beep_interval cooldown')
# two robots: follow only my robot in the robots dict
PARAMS.update(robot_name='pinky_2'); n3=rn.RobotNotifier(); n3.player=FakePlayer(); PLAYED.clear()
st3=lambda d: n3.subs['/delivery/status'](types.SimpleNamespace(data=json.dumps(d)))
st3({'robots': {'pinky_1': {'state': 'WAIT_LOAD'}, 'pinky_2': {'state': 'TO_STORE'}}})
ok(PLAYED==[] and n3.state=='TO_STORE','2 robots: other robot waiting -> no song here')
st3({'robots': {'pinky_1': {'state': 'TO_STATION'}, 'pinky_2': {'state': 'WAIT_LOAD'}}})
ok(PLAYED==['song'],'2 robots: my robot waiting -> song')
PARAMS.clear(); n4=rn.RobotNotifier(); ok(n4.robot_name=='pinky_1','default robot name pinky_1')
print('ALL PASS')
