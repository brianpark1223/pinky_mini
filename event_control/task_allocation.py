"""Bridge to the task allocation module (cost-based priority).

fms_controller.py calls assign() and pickup_order() from here. If the repository has
task_allocation/allocator.py (the cost owner's folder, next to event_control/), those
functions are used; if it is missing or raises an error, the default rules below are used,
so the demo never stops because of the allocator.

    assign(free, queue, points)      -> [(robot_name, task), ...]
    pickup_order(waiting, points)    -> robot_name to let into the pickup next

Inputs
  free    : {robot_name: {'pose': (x, y) | None, 'state': 'IDLE' | 'TO_HOME'}}
  queue   : [{'id': 3, 'station': 'A'}, ...] in request order
  points  : {'STORE': (x, y, yaw), 'A': ..., 'B': ..., 'HOME': ...}
  waiting : {robot_name: {'pose': (x, y) | None, 'since': monotonic seconds, 'task': {...}}}
"""
import importlib.util
import math
import os
import sys

UNKNOWN_POSE_COST = 1e6
SOURCE = 'default'          # 'task_allocation/allocator.py' when the external module is used


def travel_cost(pose, target):
    """Straight-line distance (m); a robot whose pose is unknown is chosen last."""
    if pose is None or target is None:
        return UNKNOWN_POSE_COST
    return math.hypot(pose[0] - target[0], pose[1] - target[1])


def default_assign(free, queue, points):
    """Requests in order; each goes to the free robot closest to the pickup."""
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


def default_pickup_order(waiting, points):
    """First come, first served."""
    if not waiting:
        return None
    return min(waiting, key=lambda n: (waiting[n].get('since', 0.0), n))


def _load_external():
    path = os.environ.get('PINKY_ALLOCATOR') or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'task_allocation', 'allocator.py')
    if not os.path.isfile(path):
        return None
    try:
        spec = importlib.util.spec_from_file_location('pinky_allocator', path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception as exc:  # noqa: BLE001
        print(f'[task_allocation] {path} could not be loaded ({exc}); using default rules', file=sys.stderr)
        return None


_ext = _load_external()
if _ext is not None:
    SOURCE = 'task_allocation/allocator.py'


def _valid_pairs(pairs, free, queue):
    ids = [id(t) for t in queue]
    seen = set()
    seen_tasks = set()
    for item in pairs:
        name, task = item
        if name not in free or name in seen or id(task) not in ids or id(task) in seen_tasks:
            return False
        seen.add(name)
        seen_tasks.add(id(task))
    return True


def assign(free, queue, points):
    if _ext is not None and hasattr(_ext, 'assign'):
        try:
            pairs = list(_ext.assign(dict(free), list(queue), points))
            if _valid_pairs(pairs, free, queue):
                return pairs
            print('[task_allocation] allocator.assign() returned an invalid result; using default rules',
                  file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] allocator.assign() failed ({exc}); using default rules', file=sys.stderr)
    return default_assign(free, queue, points)


def pickup_order(waiting, points):
    if _ext is not None and hasattr(_ext, 'pickup_order'):
        try:
            name = _ext.pickup_order(dict(waiting), points)
            if name is None or name in waiting:
                return name
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] allocator.pickup_order() failed ({exc}); using default rules',
                  file=sys.stderr)
    return default_pickup_order(waiting, points)
