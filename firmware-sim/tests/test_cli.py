"""End-to-end runs through cli.main().

Nothing here touches --plan: that shells out to path-planning under uv, which
needs OpenCV, a maze photo and several seconds of RRT*. The seam it crosses --
generated header in, simulation out -- is covered by test_maze_header.py on both
sides instead.
"""

import io
import json
import pathlib
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

from ..cli import main
from ..maze_header import DEFAULT_MAP, DEFAULT_PATH


def run(*argv):
    """cli.main() with output captured. Returns (exit code, stdout).

    stderr goes to a sink too, so argparse's usage dump on the rejection tests
    below does not scroll past the suite's own results.
    """
    buf = io.StringIO()
    with redirect_stdout(buf), redirect_stderr(io.StringIO()):
        code = main(list(argv))
    return code, buf.getvalue()


class TestTaskScenarios(unittest.TestCase):
    def test_task31_reaches_the_end_of_its_path(self):
        code, out = run("task31", "--max-seconds", "20")
        self.assertEqual(code, 0)
        self.assertIn("planner idle", out)
        self.assertIn("segment    3 of 3", out)

    def test_task33_rotates_and_stops(self):
        code, out = run("task33", "--max-seconds", "10")
        self.assertEqual(code, 0)

    def test_task34_walks_every_grid_pose(self):
        code, out = run("task34", "--max-seconds", "30")
        self.assertEqual(code, 0)
        self.assertIn("segment    8 of 9", out)

    def test_task32_gets_a_wall_to_range_against(self):
        """Without one the beam saturates and DistancePlanner reverses for ever
        against a constant -- see scenarios.task32_bench."""
        _, out = run("task32", "--fusion-fix", "--max-seconds", "10")
        self.assertIn("front", out)
        self.assertNotIn("front 300", out)

    def test_an_unfinished_run_exits_non_zero(self):
        code, _ = run("task31", "--max-seconds", "0.2")
        self.assertEqual(code, 1)


class TestPlannedScenario(unittest.TestCase):
    """The default: the two headers the sketch compiles against."""

    def setUp(self):
        if not (DEFAULT_MAP.exists() and DEFAULT_PATH.exists()):
            self.skipTest("no generated headers -- run path-planning/build_maze.sh")

    def test_dead_reckoning_drives_the_whole_path(self):
        code, out = run("--no-localisation", "--max-seconds", "60")
        self.assertEqual(code, 0, out)
        self.assertIn("planner idle", out)

    def test_lidar_localisation_drives_the_whole_path(self):
        code, out = run("--fusion-fix", "--max-seconds", "60")
        self.assertEqual(code, 0, out)
        self.assertIn("planner idle", out)
        self.assertIn("clearance  clean", out)

    def test_the_fix_recovers_a_start_error(self):
        """The robot is put 25 mm and 4 deg from where sf.set() says it is. Dead
        reckoning can only carry that error along; the lidar has to remove it."""
        _, blind = run("--no-localisation", "--start-error", "25,15,4",
                       "--max-seconds", "60")
        _, fixed = run("--fusion-fix", "--start-error", "25,15,4",
                       "--max-seconds", "60")
        self.assertGreater(error_mm(blind), 20.0)
        self.assertLess(error_mm(fixed), 5.0)

    def test_the_observer_actually_uses_beams(self):
        _, out = run("--fusion-fix", "--max-seconds", "60")
        line = next(l for l in out.splitlines() if l.startswith("fixes"))
        self.assertNotIn(" 0 loops corrected", line)

    def test_json_output_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            out = pathlib.Path(d) / "run.json"
            run("--fusion-fix", "--max-seconds", "60", "--json", str(out), "--quiet")
            data = json.loads(out.read_text())
        self.assertTrue(data["finished"])
        self.assertEqual(data["scenario"], "planned")
        self.assertEqual(data["geometry"]["kind"], "segments")
        self.assertEqual(len(data["trail_true"]), len(data["trail_est"]))

    def test_svg_output_is_written(self):
        with tempfile.TemporaryDirectory() as d:
            out = pathlib.Path(d) / "run.svg"
            run("--fusion-fix", "--max-seconds", "60", "--svg", str(out), "--quiet")
            text = out.read_text()
        self.assertTrue(text.startswith("<svg"))
        self.assertTrue(text.rstrip().endswith("</svg>"))

    def test_a_missing_header_is_reported_not_crashed_through(self):
        from ..maze_header import MazeHeaderError

        with self.assertRaises(MazeHeaderError):
            run("--path", "/nonexistent/maze_path.h", "--quiet")


def error_mm(output):
    for line in output.splitlines():
        if line.startswith("error "):
            return float(line.split()[1])
    raise AssertionError(f"no error line in:\n{output}")


class TestArgumentParsing(unittest.TestCase):
    def test_start_error_wants_three_values(self):
        with self.assertRaises(SystemExit):
            run("task31", "--start-error", "1,2")

    def test_start_error_wants_numbers(self):
        with self.assertRaises(SystemExit):
            run("task31", "--start-error", "a,b,c")

    def test_unknown_scenario_is_rejected(self):
        with self.assertRaises(SystemExit):
            run("task99")


if __name__ == "__main__":
    unittest.main()
