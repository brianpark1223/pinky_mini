import math
import unittest

from pinky_mission.logic import route, heading, clear_in_grid


class MissionLogicTest(unittest.TestCase):
    def setUp(self):
        self.points = [dict(x=0., y=0., yaw=None), dict(x=1., y=0., yaw=None),
                       dict(x=1., y=1., yaw=None), dict(x=0., y=1., yaw=None)]
        self.grid = dict(width=20, height=20, resolution=.1, origin_x=-.5,
                         origin_y=-.5, data=[0] * 400)

    def test_routes(self):
        self.assertEqual(route('delivery_1', 4), [0, 1, 2, 0])
        self.assertEqual(route('delivery_2', 4), [0, 1, 3, 0])
        self.assertEqual(route('delivery_3', 4), [0, 1, 2, 3, 0])
        self.assertEqual(route('delivery_4', 4), [0, 1, 3, 2, 0])
        self.assertEqual(route('delivery_5', 4), [0, 1, 2, 1, 3, 0])
        self.assertEqual(route('delivery_6', 4), [0, 1, 3, 1, 2, 0])
        self.assertEqual(route('sequence', 4), [0, 1, 2, 3])
        self.assertEqual(route('patrol', 4, 2), [0, 1, 2, 3, 0] * 2)
        self.assertEqual(route('roundtrip', 4), [0, 1, 2, 3, 2, 1, 0])
        self.assertEqual(route('wait', 4), [0, 1, 2, 3])
        self.assertEqual(route('photo', 4), [0, 1, 2, 3])

    def test_heading(self):
        self.assertAlmostEqual(heading(self.points, 1), math.pi / 2)
        self.points[1]['yaw'] = -.7
        self.assertAlmostEqual(heading(self.points, 1), -.7)
        self.points[1]['yaw'] = None
        self.assertAlmostEqual(heading(self.points, 3, next_index=2), 0.0)

    def test_costmap_footprint(self):
        self.assertTrue(clear_in_grid(0, 0, self.grid, .14))
        self.grid['data'][5 * 20 + 5] = 254
        self.assertFalse(clear_in_grid(0, 0, self.grid, .14))
        self.grid['data'][5 * 20 + 5] = 255
        self.assertFalse(clear_in_grid(0, 0, self.grid, .14))
        self.assertFalse(clear_in_grid(2, 2, self.grid, .14))


if __name__ == '__main__':
    unittest.main()
