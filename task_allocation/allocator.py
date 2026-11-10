"""Cost-based task allocation for the 2-robot FMS (cost 담당 파일).

event_control/fms_controller.py calls the two functions below through
event_control/task_allocation.py. Keep the names and the input/output shapes;
everything inside can change. If this file is missing or raises an error,
the controller falls back to the default rules in event_control/task_allocation.py.

    assign(free, queue, points)      -> [(robot_name, task), ...]
    pickup_order(waiting, points)    -> robot_name to let into the pickup next

Inputs
  free    : {robot_name: {'pose': (x, y) | None, 'state': 'IDLE' | 'TO_HOME'}}
            robots that can take a new task right now
  queue   : [{'id': 3, 'station': 'A'}, ...] in request order
  points  : {'STORE': (x, y, yaw), 'A': ..., 'B': ..., 'HOME': ...}
  waiting : {robot_name: {'pose': (x, y) | None, 'since': monotonic seconds, 'task': {...}}}
            robots in WAIT_PICKUP

Starting rules (replace with the cost algorithm)
  assign       : requests are served in order; each goes to the free robot with the
                 lowest cost = straight-line distance robot -> pickup (unknown pose = far)
  pickup_order : first come, first served
"""
import math

UNKNOWN_POSE_COST = 1e6


def travel_cost(pose, target):
    """Straight-line distance (m); a robot whose pose is unknown is chosen last."""
    if pose is None or target is None:
        return UNKNOWN_POSE_COST
    return math.hypot(pose[0] - target[0], pose[1] - target[1])


def assign(free, queue, points):
    store = points.get('STORE')
    left = dict(free)
    pairs = []
    for task in queue:
        if not left:
            break
        name = min(left, key=lambda n: (travel_cost(left[n].get('pose'), store), n))
        pairs.append((name, task))
        del left[name]
    return pairs


def pickup_order(waiting, points):
    if not waiting:
        return None
    return min(waiting, key=lambda n: (waiting[n].get('since', 0.0), n))
