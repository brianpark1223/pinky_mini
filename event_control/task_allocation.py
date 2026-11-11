"""Bridge to the task allocation module (cost-based priority).

fms_controller.py calls assign() and pickup_order() from here. If the repository has
task_allocation/allocator.py (the cost owner's folder, next to event_control/), those
functions are used; if it is missing, raises an error or returns something invalid,
the default rules below are used, so the demo never stops because of the allocator.

    assign(robots, queue, points, ctx) -> [(robot_name, task), ...]
    pickup_order(waiting, points)      -> robot_name to let into the pickup next

assign() inputs
  robots : {robot_name: {
              'state'      : 'IDLE' | 'TO_STORE' | 'WAIT_PICKUP' | 'DOCKING' | 'WAIT_LOAD' |
                             'TO_STATION' | 'WAIT_RECEIVE' | 'TO_HOME' | 'ERROR',
              'pose'       : (x, y) | None,
              'battery'    : 0..100 | None,
              'current'    : task being delivered | None,
              'next'       : task already reserved after it | None,
              'target'     : point name it is driving to | None,
              'free'       : True -> a task given now starts right away
              'can_reserve': True -> busy, but a task given now is done right after the current one
              'elapsed'    : seconds in the current state (for remaining loading/unloading time)
              'route_m'    : equivalent metres until it could be at the pickup for a new task
                             (remaining driving + remaining stops x speed; None = cannot take one)
           }}  (every robot, also ERROR ones; only free / can_reserve robots may be returned)
  queue  : [{'id': 3, 'station': 'A'}, ...] unassigned requests in request order
  points : {'STORE': (x, y, yaw), 'A': ..., 'B': ..., 'HOME': ...}
  ctx    : {'speed': m/s, 't_loading': s, 't_unloading': s, 'detour': factor,
            'pickup': robot holding the pickup | None, 'home': 'HOME'}
assign() output
  pairs (robot, task): task must be an item of queue, at most one per robot.
  Returning [] leaves the requests queued; assign() is asked again 5 times a second.

pickup_order() input
  waiting : {robot_name: {'pose': (x, y) | None, 'since': monotonic seconds, 'task': {...}}}

An allocator written for the old 3-argument form assign(free, queue, points) still works:
it gets only the free robots ({name: {'pose', 'state', ...}}).

Class form (current task_allocation/allocator.py): if the module defines an Allocator class,
it is used instead of assign():
    Allocator(zones, Config(...), dist=...)  -- built once; v / t_load_s / t_receive_s come
                                                from the controller parameters, t_align_s is 0
                                                when docking is off, dist includes detour_factor
    alloc.queue = [Request, ...]             -- refreshed from the controller queue every call
    alloc.dispatch([Robot, ...])             -- -> assign()
    alloc.next_for(robot_name)               -- -> next_for() when a robot finishes a delivery
  The controller queue stays the single source of truth; DOCKING is passed as TO_STORE at the
  pickup, and a free robot whose Nav2 is not ready is passed as ERROR (never chosen).

    next_for(robot_name, queue, points) -> task from queue to chain right after 수령 완료, or None
"""
import importlib.util
import inspect
import math
import os
import sys

UNKNOWN_COST = 1e6
SOURCE = 'default'          # 'task_allocation/allocator.py' when the external module is used


def travel_cost(pose, target):
    """Straight-line distance (m); a robot whose pose is unknown is chosen last."""
    if pose is None or target is None:
        return UNKNOWN_COST
    return math.hypot(pose[0] - target[0], pose[1] - target[1])


def _cost(info, store):
    route = info.get('route_m')
    return route if isinstance(route, (int, float)) else travel_cost(info.get('pose'), store)


def default_assign(robots, queue, points, ctx=None):
    """Requests in order; each goes to the FREE robot closest to the pickup (no reservations)."""
    store = points.get('STORE')
    left = {n: i for n, i in robots.items() if i.get('free', True)}
    pairs = []
    for task in queue:
        if not left:
            break
        name = min(left, key=lambda n: (_cost(left[n], store), n))
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
        sys.modules[spec.name] = mod     # needed by @dataclass (looks the module up while defining)
        spec.loader.exec_module(mod)
        return mod
    except Exception as exc:  # noqa: BLE001
        sys.modules.pop('pinky_allocator', None)
        print(f'[task_allocation] {path} could not be loaded ({exc}); using default rules', file=sys.stderr)
        return None


def _takes_ctx(fn):
    try:
        params = inspect.signature(fn).parameters.values()
    except (TypeError, ValueError):
        return True
    if any(p.kind in (p.VAR_POSITIONAL,) for p in params):
        return True
    return len([p for p in params if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]) >= 4


_ext = _load_external()
if _ext is not None:
    SOURCE = 'task_allocation/allocator.py' + (' (Allocator class)' if hasattr(_ext, 'Allocator') else '')


def _valid_pairs(pairs, robots, queue):
    ids = [id(t) for t in queue]
    seen_r, seen_t = set(), set()
    for item in pairs:
        name, task = item
        info = robots.get(name)
        if info is None or not (info.get('free', True) or info.get('can_reserve')):
            return False
        if name in seen_r or id(task) not in ids or id(task) in seen_t:
            return False
        seen_r.add(name)
        seen_t.add(id(task))
    return True


# ---------------------------------------------------------------- class-form allocator
_cls_alloc = None
_cls_key = None


def _has_class():
    return _ext is not None and hasattr(_ext, 'Allocator') and hasattr(_ext, 'Robot')


def _get_alloc(points, ctx):
    """Build (or reuse) the external Allocator with the controller's timing parameters."""
    global _cls_alloc, _cls_key
    zones = {k: (float(v[0]), float(v[1])) for k, v in points.items()}
    detour = float(ctx.get('detour', 1.0) or 1.0)
    key = (tuple(sorted(zones.items())), detour, ctx.get('speed'), ctx.get('t_loading'),
           ctx.get('t_unloading'), ctx.get('docking'), ctx.get('w_dist'),
           ctx.get('w_wait'), ctx.get('w_battery'))
    if _cls_alloc is not None and key == _cls_key:
        return _cls_alloc
    over = {}
    if ctx.get('speed'):
        over['v'] = float(ctx['speed'])
    if ctx.get('t_loading') is not None:
        over['t_load_s'] = float(ctx['t_loading'])
    if ctx.get('t_unloading') is not None:
        over['t_receive_s'] = float(ctx['t_unloading'])
    if ctx.get('docking') is False:
        over['t_align_s'] = 0.0
    for key in ('w_dist', 'w_wait', 'w_battery'):
        if ctx.get(key) is not None:
            over[key] = float(ctx[key])
    cfg = _ext.Config()
    if over:
        from dataclasses import replace
        cfg = replace(cfg, **{k: v for k, v in over.items() if hasattr(cfg, k)})
    dist = (lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]) * detour)
    alloc = _ext.Allocator(zones, cfg, dist=dist)
    fix = {k: 1.0 for k in ('d_max', 'w_max_s')                 # all times 0 -> normaliser 0 -> x/0
           if isinstance(getattr(alloc.cfg, k, None), (int, float)) and getattr(alloc.cfg, k) <= 0}
    if fix:
        from dataclasses import replace
        alloc.cfg = replace(alloc.cfg, **fix)
    _cls_alloc, _cls_key = alloc, key
    return _cls_alloc


def _to_requests(queue):
    return [_ext.Request(id=t['id'], station=t['station'], created_at=float(i))
            for i, t in enumerate(queue)]


def _to_robot(name, info, points):
    S = _ext.State
    state = info.get('state')
    pos = info.get('pose')
    if state == 'DOCKING':                   # aligning at the pickup ~ just arrived there
        state, pos = 'TO_STORE', points['STORE'][:2]
    if state in ('IDLE', 'TO_HOME') and not info.get('free', True):
        state = 'ERROR'                      # e.g. Nav2 not ready: never pick it
    try:
        st = S(state)
    except ValueError:
        st = S('ERROR')
    if pos is None:                          # no AMCL pose yet: assume where it is heading / HOME
        tgt = info.get('target')
        pos = points.get(tgt, points.get('HOME'))[:2] if tgt in points else points['HOME'][:2]
    cur = info.get('current')
    task = _ext.Request(id=cur['id'], station=cur['station'], created_at=0.0) \
        if isinstance(cur, dict) and cur.get('station') in points else None
    if task is None and st not in (S('IDLE'), S('TO_HOME'), S('ERROR')):
        st = S('ERROR')                      # busy without a task: cannot estimate
    return _ext.Robot(name=name, state=st, pos=(float(pos[0]), float(pos[1])), task=task,
                      state_elapsed_s=float(info.get('elapsed') or 0.0),
                      battery=info.get('battery'), route_m=info.get('route_m'))


def _class_assign(robots, queue, points, ctx):
    alloc = _get_alloc(points, ctx)
    alloc.queue = _to_requests(queue)
    snap = [_to_robot(n, i, points) for n, i in robots.items()]
    by_id = {t['id']: t for t in queue}
    return [(name, by_id[req.id]) for name, req in alloc.dispatch(snap)]


def assign(robots, queue, points, ctx=None):
    ctx = ctx or {}
    if _has_class():
        try:
            pairs = _class_assign(robots, queue, points, ctx)
            if _valid_pairs(pairs, robots, queue):
                return pairs
            print('[task_allocation] Allocator.dispatch() returned an invalid result; using default rules',
                  file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] Allocator.dispatch() failed ({exc}); using default rules',
                  file=sys.stderr)
        return default_assign(robots, queue, points, ctx)
    if _ext is not None and hasattr(_ext, 'assign'):
        try:
            if _takes_ctx(_ext.assign):
                pairs = list(_ext.assign(dict(robots), list(queue), points, dict(ctx)))
            else:                                   # old form: free robots only
                free = {n: i for n, i in robots.items() if i.get('free', True)}
                pairs = list(_ext.assign(free, list(queue), points)) if free else []
            if _valid_pairs(pairs, robots, queue):
                return pairs
            print('[task_allocation] allocator.assign() returned an invalid result; using default rules',
                  file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] allocator.assign() failed ({exc}); using default rules', file=sys.stderr)
    return default_assign(robots, queue, points, ctx)


def allocation_costs(robots, points, ctx=None):
    """Return the actual comparable costs used by the active allocator."""
    if _has_class():
        try:
            alloc = _get_alloc(points, ctx or {})
            snap = [_to_robot(n, i, points) for n, i in robots.items()]
            return {estimate.robot: estimate.C for estimate in alloc.estimates(snap)}
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] cost reporting failed ({exc})', file=sys.stderr)
    return {name: _cost(info, points.get('STORE')) for name, info in robots.items()
            if info.get('free', True)}


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


def default_next_for(robot_name, queue, points):
    """The robot that just finished takes the oldest queued request."""
    return queue[0] if queue else None


def next_for(robot_name, queue, points, ctx=None):
    """Called on 수령 완료 when the robot has no reservation: task to chain, or None (-> 수납대)."""
    queue = list(queue)
    if _has_class() and hasattr(_ext.Allocator, 'next_for'):
        try:
            alloc = _get_alloc(points, ctx or {})
            alloc.queue = _to_requests(queue)
            req = alloc.next_for(robot_name)
            if req is None:
                return None
            for t in queue:
                if t['id'] == req.id:
                    return t
            print('[task_allocation] Allocator.next_for() returned an unknown request; using default rules',
                  file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            print(f'[task_allocation] Allocator.next_for() failed ({exc}); using default rules',
                  file=sys.stderr)
    return default_next_for(robot_name, queue, points)
