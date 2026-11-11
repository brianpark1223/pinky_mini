import math
import types

from fleet_metrics import battery_fraction, euclidean_distance, path_distance
from traffic_manager import TrafficManager


def ok(value, message):
    assert value, message
    print('PASS', message)


NS = types.SimpleNamespace
path = NS(poses=[NS(pose=NS(position=NS(x=0.0, y=0.0))),
                 NS(pose=NS(position=NS(x=3.0, y=0.0))),
                 NS(pose=NS(position=NS(x=3.0, y=4.0)))])
ok(path_distance(path) == 7.0, 'Nav2 path distance sums every segment')
ok(path_distance(NS(poses=[])) is None, 'empty Nav2 path is unavailable')
ok(euclidean_distance((0, 0), (3, 4, 0)) == 5.0, 'Euclidean fallback distance')
ok(battery_fraction(NS(percentage=0.73)) == 0.73, 'BatteryState percentage accepted')
ok(battery_fraction(NS(percentage=math.nan)) is None, 'missing battery is not converted to a real value')
ok(battery_fraction(NS(percentage=1.2)) is None, 'invalid battery range rejected')


class Clock:
    value = 0.0
    def __call__(self): return self.value


clock = Clock()
cfg = {'corridor_axis': [[0, 0], [10, 0]], 'allow_same_direction_following': True,
       'min_follow_distance': 2.0, 'heartbeat_timeout': 3.0, 'wait_timeout': 5.0,
       'entry_angle_deg': 45, 'exit_angle_deg': 135}
t = TrafficManager(cfg, clock)
ok(t.request('p1', 'entry', (4, 0))[0], 'first robot reserves corridor')
ok(not t.request('p2', 'exit', (8, 0))[0], 'opposite direction is mutually exclusive')
ok(not t.request('p2', 'entry', (3, 0))[0], 'same direction blocked inside safety distance')
ok(t.request('p2', 'entry', (1, 0))[0], 'same direction allowed after measured safety gap')
t.release('p1'); t.release('p2')
ok(t.request('p1', 'exit', (8, 0))[0], 'exit direction reservation')
clock.value = 4.0
ok('p1' in t.stale_occupants(), 'lost robot is detected from heartbeat timeout')
ok(not t.request('p2', 'entry', (1, 0))[0], 'lost owner keeps corridor locked fail-closed')
clock.value = 10.0
ok(t.waiting_timed_out('p2'), 'traffic waiting timeout detected')
t.release('p1')
ok(t.request('p2', 'entry', (1, 0))[0], 'operator-safe release breaks stale lock')

try:
    TrafficManager(dict(cfg, entry_angle_deg=100), clock)
    raise AssertionError('bad entry angle accepted')
except ValueError:
    print('PASS entry direction constrained to 0..90 degrees')
try:
    TrafficManager(dict(cfg, exit_angle_deg=80), clock)
    raise AssertionError('bad exit angle accepted')
except ValueError:
    print('PASS exit direction constrained to 90..180 degrees')

t = TrafficManager(dict(cfg, entry_waypoint=[1, 2], exit_waypoint=[3, 4]), clock)
ok(t.waypoint('entry') == (1.0, 2.0, math.pi / 4),
   'entry gate pose uses 45 degree heading')
ok(t.waypoint('exit') == (3.0, 4.0, 3 * math.pi / 4),
   'exit gate pose uses 135 degree heading')

print('ALL PASS')
