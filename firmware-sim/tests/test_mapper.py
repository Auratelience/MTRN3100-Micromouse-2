import math
import unittest

from ..mapper import LOG_ODDS_LIMIT, Mapper
from ..plant import LIDAR_MOUNTS
from ..types import Pose
from ..world import World


def hit_at(sensor, distance_mm, theta=0.0):
    """Where a reading of `distance_mm` on `sensor` lands in world mm, for a
    robot at the origin heading `theta`.

    Computed from LIDAR_MOUNTS rather than written out, because the mounts are
    constants.h's measured values and these tests should follow them there
    rather than pinning last year's numbers.
    """
    m = LIDAR_MOUNTS[sensor]
    c, s = math.cos(theta), math.sin(theta)
    ox = m.x * c - m.y * s
    oy = m.x * s + m.y * c
    heading = theta + m.theta
    return (ox + distance_mm * math.cos(heading), oy + distance_mm * math.sin(heading))


class TestMapper(unittest.TestCase):
    def setUp(self):
        self.world = World.blank(2000.0, 2000.0, 5.0)
        self.m = Mapper.matching(self.world)

    def _odds_at_world(self, x_mm, y_mm):
        px, py = self.m.world_to_px(x_mm, y_mm)
        return self.m.log_odds_at(int(math.floor(px)), int(math.floor(py)))

    def test_ray_marks_terminal_cell_occupied(self):
        self.m.integrate(Pose(0.0, 0.0, 0.0), {"front": 100}, 300.0)
        # 100 mm along the ray, from a mount that is not at the robot centre
        self.assertGreater(self._odds_at_world(*hit_at("front", 100)), 0.0)

    def test_ray_marks_intervening_cells_free(self):
        self.m.integrate(Pose(0.0, 0.0, 0.0), {"front": 200}, 300.0)
        self.assertLess(self._odds_at_world(100.0, 0.0), 0.0)

    def test_saturated_reading_marks_no_obstacle(self):
        self.m.integrate(Pose(0.0, 0.0, 0.0), {"front": 300}, 300.0)
        self.assertLessEqual(self._odds_at_world(*hit_at("front", 300)), 0.0)

    def test_repeated_observations_increase_confidence(self):
        self.m.integrate(Pose(0, 0, 0), {"front": 100}, 300.0)
        once = self._odds_at_world(*hit_at("front", 100))
        for _ in range(5):
            self.m.integrate(Pose(0, 0, 0), {"front": 100}, 300.0)
        self.assertGreater(self._odds_at_world(*hit_at("front", 100)), once)

    def test_confidence_saturates(self):
        for _ in range(500):
            self.m.integrate(Pose(0, 0, 0), {"front": 100}, 300.0)
        self.assertLessEqual(self._odds_at_world(*hit_at("front", 100)), LOG_ODDS_LIMIT)

    def test_side_sensors_use_their_own_bearings(self):
        # left mount looks along +y, so its hit lands off the +x axis
        self.m.integrate(Pose(0.0, 0.0, 0.0), {"left": 100}, 300.0)
        self.assertGreater(self._odds_at_world(*hit_at("left", 100)), 0.0)

    def test_rotating_the_robot_rotates_the_rays(self):
        self.m.integrate(Pose(0.0, 0.0, math.pi / 2), {"front": 100}, 300.0)
        # facing +y, the front hit swings onto the +y axis with the robot
        self.assertGreater(self._odds_at_world(*hit_at("front", 100, math.pi / 2)), 0.0)

    def test_unknown_sensor_name_is_ignored(self):
        self.m.integrate(Pose(0, 0, 0), {"rear": 100}, 300.0)
        # nothing was traced at all, so the cell behind the robot is untouched
        self.assertEqual(self._odds_at_world(*hit_at("front", 100)), 0.0)

    def test_rle_covers_every_cell(self):
        runs = self.m.to_rle()
        self.assertEqual(sum(runs), self.m.width * self.m.height)

    def test_rle_reports_the_hit(self):
        self.m.integrate(Pose(0, 0, 0), {"front": 100}, 300.0)
        runs = self.m.to_rle()
        self.assertEqual(sum(runs), self.m.width * self.m.height)
        self.assertGreater(len(runs), 1)  # something crossed the threshold

    def test_clear_wipes_the_map(self):
        self.m.integrate(Pose(0, 0, 0), {"front": 100}, 300.0)
        self.m.clear()
        self.assertEqual(self._odds_at_world(*hit_at("front", 100)), 0.0)

    def test_matching_world_shares_geometry(self):
        self.assertEqual(
            (self.m.width, self.m.height), (self.world.width, self.world.height)
        )
        self.assertEqual(self.m.origin_px, self.world.origin_px)
        self.assertEqual(self.m.mm_per_pixel, self.world.mm_per_pixel)


if __name__ == "__main__":
    unittest.main()
