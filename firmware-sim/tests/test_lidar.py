import unittest

from ..lidar import L, LIDAR
from ..plant import Plant, PlantConfig
from ..types import Pose
from ..world import World

from .test_world import fill_wall_at_x


def _wall_at(x_mm):
    w = World.blank(2000.0, 2000.0, 5.0)
    fill_wall_at_x(w, x_mm)
    return w


def _open():
    return World.blank(4000.0, 4000.0, 5.0)


class TestLidar(unittest.TestCase):
    def test_reading_quantised_to_2mm(self):
        lid = LIDAR(Plant(_wall_at(157.0), PlantConfig(lidar_noise_mm=0.0), Pose(0, 0, 0)))
        lid.update(0.0)
        self.assertEqual(lid.getReading(LIDAR.Front) % 2, 0)

    def test_open_space_clamps_to_max_dist(self):
        lid = LIDAR(Plant(_open(), PlantConfig(lidar_noise_mm=0.0), Pose(0, 0, 0)))
        lid.update(0.0)
        self.assertEqual(lid.getReading(LIDAR.Front), L.MAX_DIST)

    def test_reading_is_stale_between_refresh_periods(self):
        plant = Plant(_wall_at(250.0), PlantConfig(lidar_noise_mm=0.0), Pose(0, 0, 0))
        lid = LIDAR(plant)
        lid.update(0.0)
        first = lid.getReading(LIDAR.Front)
        plant._set_pose(Pose(100.0, 0.0, 0.0))
        lid.update(0.002)  # 2 ms < 10 ms period
        self.assertEqual(lid.getReading(LIDAR.Front), first)
        lid.update(0.020)  # past the period
        self.assertNotEqual(lid.getReading(LIDAR.Front), first)

    def test_all_three_sensors_report(self):
        lid = LIDAR(Plant(_open(), PlantConfig(lidar_noise_mm=0.0), Pose(0, 0, 0)))
        lid.update(0.0)
        for s in (LIDAR.Front, LIDAR.Left, LIDAR.Right):
            self.assertGreaterEqual(lid.getReading(s), L.MIN_DIST)
            self.assertLessEqual(lid.getReading(s), L.MAX_DIST)

    def test_close_wall_reads_below_max(self):
        lid = LIDAR(Plant(_wall_at(150.0), PlantConfig(lidar_noise_mm=0.0), Pose(0, 0, 0)))
        lid.update(0.0)
        self.assertLess(lid.getReading(LIDAR.Front), L.MAX_DIST)
        self.assertGreater(lid.getReading(LIDAR.Front), 0)

    def test_init_returns_true(self):
        lid = LIDAR(Plant(World.blank(1000.0, 1000.0, 5.0), PlantConfig(), Pose(0, 0, 0)))
        self.assertTrue(lid.init())


if __name__ == "__main__":
    unittest.main()
