"""Pure helpers for fleet distance and battery data."""
import math


def euclidean_distance(start, goal):
    if start is None or goal is None:
        return None
    values = (start[0], start[1], goal[0], goal[1])
    if not all(math.isfinite(float(v)) for v in values):
        return None
    return math.hypot(float(start[0]) - float(goal[0]), float(start[1]) - float(goal[1]))


def path_distance(path):
    """Return nav_msgs/Path length, or None for an empty/invalid path."""
    poses = getattr(path, 'poses', None)
    if not poses:
        return None
    points = []
    for stamped in poses:
        p = stamped.pose.position
        if not all(math.isfinite(float(v)) for v in (p.x, p.y)):
            return None
        points.append((float(p.x), float(p.y)))
    if len(points) == 1:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def battery_fraction(msg):
    """Read BatteryState.percentage without inventing a value from voltage."""
    value = getattr(msg, 'percentage', math.nan)
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value < 0.0 or value > 1.0:
        return None
    return value
