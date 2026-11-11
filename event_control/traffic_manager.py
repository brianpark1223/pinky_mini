"""Fail-closed shared-corridor reservations for a small robot fleet.

This module does not command velocity. It decides whether the FMS may send a
Nav2 goal into the configured shared corridor. Opposite directions are mutually
exclusive. Same-direction following is optional and needs a measured separation.
"""
import math
import time


class TrafficManager:
    DIRECTIONS = ('entry', 'exit')

    def __init__(self, config, clock=time.monotonic):
        self.clock = clock
        self.enabled = bool(config.get('enabled', True))
        self.allow_following = bool(config.get('allow_same_direction_following', False))
        self.min_follow_distance = float(config.get('min_follow_distance', 1.0))
        self.heartbeat_timeout = float(config.get('heartbeat_timeout', 3.0))
        self.wait_timeout = float(config.get('wait_timeout', 60.0))
        self.entry_angle_deg = float(config.get('entry_angle_deg', 45.0))
        self.exit_angle_deg = float(config.get('exit_angle_deg', 135.0))
        self.waypoints = {}
        for direction in self.DIRECTIONS:
            value = config.get(f'{direction}_waypoint')
            if value is None:
                self.waypoints[direction] = None
                continue
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise ValueError(f'{direction}_waypoint must be [x,y]')
            point = tuple(float(v) for v in value)
            if not all(math.isfinite(v) for v in point):
                raise ValueError(f'{direction}_waypoint must contain finite coordinates')
            self.waypoints[direction] = point
        axis = config.get('corridor_axis', [[0.0, 0.0], [1.0, 0.0]])
        if len(axis) != 2 or any(len(p) != 2 for p in axis):
            raise ValueError('corridor_axis must be [[x1,y1],[x2,y2]]')
        dx, dy = float(axis[1][0]) - float(axis[0][0]), float(axis[1][1]) - float(axis[0][1])
        length = math.hypot(dx, dy)
        if length <= 0 or self.min_follow_distance <= 0 or self.heartbeat_timeout <= 0 \
                or self.wait_timeout <= 0:
            raise ValueError('invalid traffic safety parameters')
        if not 0.0 <= self.entry_angle_deg <= 90.0:
            raise ValueError('entry_angle_deg must be in 0..90')
        if not 90.0 <= self.exit_angle_deg <= 180.0:
            raise ValueError('exit_angle_deg must be in 90..180')
        self.origin = (float(axis[0][0]), float(axis[0][1]))
        self.unit = (dx / length, dy / length)
        self.occupants = {}  # robot -> {direction, pose, seen, entered}
        self.waiters = {}    # robot -> {direction, since}

    def waypoint(self, direction):
        """Return the measured gate pose using the configured entry/exit heading."""
        point = self.waypoints.get(direction)
        if point is None:
            return None
        angle = self.entry_angle_deg if direction == 'entry' else self.exit_angle_deg
        return point[0], point[1], math.radians(angle)

    def projection(self, pose):
        if pose is None:
            return None
        return ((float(pose[0]) - self.origin[0]) * self.unit[0]
                + (float(pose[1]) - self.origin[1]) * self.unit[1])

    def update(self, robot, pose, seen=None):
        if robot not in self.occupants or pose is None:
            return
        self.occupants[robot]['pose'] = tuple(pose[:2])
        self.occupants[robot]['seen'] = self.clock() if seen is None else float(seen)

    def stale_occupants(self, now=None):
        now = self.clock() if now is None else now
        return [name for name, data in self.occupants.items()
                if data.get('seen') is None or now - data['seen'] > self.heartbeat_timeout]

    def request(self, robot, direction, pose, seen=None):
        """Return (allowed, reason). Stale owners keep the corridor locked."""
        if not self.enabled:
            return True, 'traffic disabled'
        if direction not in self.DIRECTIONS:
            return False, 'invalid direction'
        now = self.clock()
        if robot in self.occupants:
            return True, 'already reserved'
        stale = self.stale_occupants(now)
        if stale:
            self.waiters.setdefault(robot, {'direction': direction, 'since': now})
            return False, 'stale corridor owner: ' + ','.join(stale)
        others = list(self.occupants.items())
        if any(data['direction'] != direction for _, data in others):
            self.waiters.setdefault(robot, {'direction': direction, 'since': now})
            return False, 'opposite direction has priority'
        if others:
            if not self.allow_following:
                self.waiters.setdefault(robot, {'direction': direction, 'since': now})
                return False, 'corridor occupied'
            here = self.projection(pose)
            if here is None:
                self.waiters.setdefault(robot, {'direction': direction, 'since': now})
                return False, 'requester pose unavailable'
            sign = 1.0 if direction == 'entry' else -1.0
            gaps = []
            for _, data in others:
                ahead = self.projection(data.get('pose'))
                if ahead is None:
                    return False, 'leader pose unavailable'
                gaps.append(sign * (ahead - here))
            if min(gaps) < self.min_follow_distance:
                self.waiters.setdefault(robot, {'direction': direction, 'since': now})
                return False, 'same-direction safety distance'
        self.occupants[robot] = {'direction': direction, 'pose': tuple(pose[:2]),
                                 'seen': now if seen is None else float(seen), 'entered': now}
        self.waiters.pop(robot, None)
        return True, 'reserved'

    def waiting_timed_out(self, robot, now=None):
        data = self.waiters.get(robot)
        if data is None:
            return False
        now = self.clock() if now is None else now
        return now - data['since'] > self.wait_timeout

    def release(self, robot):
        self.occupants.pop(robot, None)
        self.waiters.pop(robot, None)

    def cancel_wait(self, robot):
        self.waiters.pop(robot, None)

    def status(self):
        return {
            'enabled': self.enabled,
            'entry_angle_deg': self.entry_angle_deg,
            'exit_angle_deg': self.exit_angle_deg,
            'occupants': {k: v['direction'] for k, v in self.occupants.items()},
            'waiters': {k: v['direction'] for k, v in self.waiters.items()},
            'stale_occupants': self.stale_occupants(),
        }
