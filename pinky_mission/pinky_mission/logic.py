"""Pure mission planning and occupancy checks, safe to test without ROS."""
import math


DELIVERY_ROUTES = {
    'delivery_1': [0, 1, 2, 0],
    'delivery_2': [0, 1, 3, 0],
    'delivery_3': [0, 1, 2, 3, 0],
    'delivery_4': [0, 1, 3, 2, 0],
    'delivery_5': [0, 1, 2, 1, 3, 0],
    'delivery_6': [0, 1, 3, 1, 2, 0],
}

POINT_NAMES = ('1 待机', '2 取货', '3 送货一', '4 送货二')


def point_name(index):
    return POINT_NAMES[index] if index < len(POINT_NAMES) else str(index + 1)


def route(kind, count, cycles=1):
    if count < 1:
        raise ValueError('No waypoints selected')
    if kind in DELIVERY_ROUTES:
        if count != 4:
            raise ValueError('Delivery missions require exactly four waypoints')
        return DELIVERY_ROUTES[kind].copy()
    if kind in ('sequence', 'wait', 'photo'):
        return list(range(count))
    if kind == 'roundtrip':
        return list(range(count)) + list(range(count - 2, -1, -1))
    if kind == 'patrol':
        if cycles < 1:
            raise ValueError('Patrol cycles must be positive')
        return (list(range(count)) + [0]) * cycles
    raise ValueError('Unknown mission type: ' + kind)


def heading(points, index, current_xy=None, next_index=None):
    p = points[index]
    if p.get('yaw') is not None:
        return float(p['yaw'])
    if next_index is not None and next_index != index:
        target = points[next_index]
        return math.atan2(target['y'] - p['y'], target['x'] - p['x'])
    if next_index is None and index + 1 < len(points):
        target = points[index + 1]
        return math.atan2(target['y'] - p['y'], target['x'] - p['x'])
    if index and current_xy is None:
        prev = points[index - 1]
        return math.atan2(p['y'] - prev['y'], p['x'] - prev['x'])
    if current_xy is not None:
        return math.atan2(p['y'] - current_xy[1], p['x'] - current_xy[0])
    return 0.0


def clear_in_grid(x, y, grid, radius, occupied=50):
    """Reject unknown, occupied or out-of-grid cells within footprint radius."""
    resolution = grid['resolution']
    if resolution <= 0 or radius <= 0:
        return False
    dx, dy = x - grid['origin_x'], y - grid['origin_y']
    angle = -grid.get('origin_yaw', 0.0)
    gx = (math.cos(angle) * dx - math.sin(angle) * dy) / resolution
    gy = (math.sin(angle) * dx + math.cos(angle) * dy) / resolution
    width, height = grid['width'], grid['height']
    reach = math.ceil(radius / resolution)
    for row in range(math.floor(gy) - reach, math.floor(gy) + reach + 1):
        for col in range(math.floor(gx) - reach, math.floor(gx) + reach + 1):
            if math.hypot((col + 0.5 - gx) * resolution,
                          (row + 0.5 - gy) * resolution) > radius + resolution * 0.71:
                continue
            if col < 0 or row < 0 or col >= width or row >= height:
                return False
            value = grid['data'][row * width + col]
            if value < 0 or value >= occupied:
                return False
    return True
