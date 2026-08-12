"""Mirrors micromouse.ino's setup() and loop().

Runner.step() is one iteration of the firmware's control loop with a plant
underneath it. The planner and controller see only what SensorFusion produces;
ground truth lives in `plant` and goes nowhere near them.
"""

import math
from dataclasses import dataclass

from .constants import AXLE_LEN, WHEEL_RADIUS
from .control import MotionController
from .kinematics import Kinematics
from .lidar import L, LIDAR
from .mapper import Mapper
from .observers import (
    FrontLidarObserver,
    ImuObserver,
    LidarObserver,
    SimIMU,
    SimMotor,
    WheelObserver,
)
from .planners import DistancePlanner, HeadingPlanner, MotionPlanner, PSPlanner
from .plant import Plant, PlantConfig
from .types import Pose, Segment, Velocity

# How long the planner must command zero velocity before the run is treated as
# finished. Planners like HeadingPlanner have no terminal state -- they simply
# stop asking for motion -- so the runner needs its own idle detector.
IDLE_SECONDS = 0.25

# Trail points are kept at this fraction of the loop rate. At 1 kHz a decimation
# of 5 gives 200 Hz, far more than the display needs and cheap to hold.
TRAIL_DECIMATION = 5

# Lidar readings folded into the robot-built map at this fraction of the loop
# rate. The sensors only refresh at 100 Hz anyway.
MAP_DECIMATION = 10


@dataclass
class Frame:
    t: float
    true_pose: Pose
    est_pose: Pose
    velocity: Velocity
    desired: Velocity
    pwm: tuple
    readings: dict
    planner_idx: int
    planner_done: bool
    # Beams that survived gating in LidarObserver's last solve. 0 means the fix
    # was a no-op and the pose is pure dead reckoning for that tick, which is
    # the first thing to look at when a run drifts anyway.
    beams: int = 0


@dataclass
class Hardware:
    """Everything the firmware would have wired up in setup()."""

    plant: Plant
    kinematics: Kinematics
    leftMotor: SimMotor
    rightMotor: SimMotor
    wheel_obsv: WheelObserver
    imu_obsv: ImuObserver
    lidar: LIDAR
    lidar_obsv: LidarObserver
    fl_obsv: FrontLidarObserver


class Runner:
    """One scenario, one world, one control loop.

    `true_start_pose` is where the robot physically is; the scenario's
    `start_pose` is what sf.set() is told. They are normally the same -- the
    sketch resets odometry on the cell build_maze.sh named -- and separating
    them is how you ask what the estimator does when the robot was not put
    quite where it was supposed to be.
    """

    def __init__(
        self,
        scenario,
        world,
        loop_hz=1000.0,
        plant_config=None,
        true_start_pose=None,
    ):
        self.scenario = scenario
        self.world = world
        self.loop_hz = loop_hz
        self.dt = 1.0 / loop_hz
        self.plant_config = plant_config if plant_config is not None else PlantConfig()
        self.true_start_pose = (
            true_start_pose if true_start_pose is not None else scenario.start_pose
        )
        self.mapper = Mapper.matching(world)
        self._build()

    # --- setup() --------------------------------------------------------

    def _build(self):
        s = self.scenario
        self.plant = Plant(self.world, self.plant_config, self.true_start_pose)

        kinematics = Kinematics(WHEEL_RADIUS, AXLE_LEN)
        leftMotor = SimMotor(self.plant, "left")
        rightMotor = SimMotor(self.plant, "right")

        wheel_obsv = WheelObserver(leftMotor, rightMotor, kinematics)
        imu_obsv = ImuObserver(SimIMU(self.plant))
        lidar = LIDAR(self.plant)
        # LidarObserver lidar_obsv(lidar, MAZE_MAP)
        lidar_obsv = LidarObserver(lidar, s.map)
        fl_obsv = FrontLidarObserver(lidar)

        self.hw = Hardware(
            self.plant,
            kinematics,
            leftMotor,
            rightMotor,
            wheel_obsv,
            imu_obsv,
            lidar,
            lidar_obsv,
            fl_obsv,
        )

        # setup(): imu_obsv.init() measures gyro drift at rest
        imu_obsv.init()
        lidar.init()

        self.sf = s.fusion_factory(self.hw, s.seed_pose_mean)

        # Wired here rather than at construction, exactly as setup() does it:
        # the prior is SensorFusion's own dead-reckoned pose, and sf does not
        # exist where lidar_obsv is declared.
        lidar_obsv.setPrior(lambda: self.sf.estimate.pose())

        self.sf.set(Pose(s.start_pose.x, s.start_pose.y, s.start_pose.theta))

        self.planner = s.planner_factory()

        kp, ki, kd = s.controller_gains
        self.mc = MotionController(leftMotor, rightMotor, kinematics, kp, ki, kd)

        self.t = 0.0
        self.steps = 0
        self._idle_for = 0.0
        # Running account of what the pose fix actually did. The lidar only
        # refreshes every LIDAR_CONTINUOUS_PERIOD_MS, so at 1 kHz roughly one
        # loop in ten can solve at all; of those, the ones with zero beams are
        # the ticks where the estimate was pure dead reckoning.
        self.beam_loops = 0
        self.beams_total = 0
        self.peak_error = 0.0
        self.true_trail = [(self.plant.pose.x, self.plant.pose.y)]
        self.est_trail = [(self.sf.estimate.pose().x, self.sf.estimate.pose().y)]
        self.last_frame = None

    def reset(self):
        self.mapper.clear()
        self._build()

    # --- loop() ---------------------------------------------------------

    def step(self):
        dt = self.dt

        # Ground truth advances first: the sensors read the state that the
        # previous iteration's PWM produced.
        self.plant.step(dt)

        # The sensors are free-running against millis() on hardware; here that
        # clock is advanced by hand. LidarObserver.update() calls lidar.update()
        # itself, as the firmware's does -- this only decides whether that call
        # finds a fresh conversion or the held one.
        self.hw.lidar.advance(dt)
        # Kept for the map and the display: a scenario with no pose source
        # never calls lidar.update() at all, and the readings would sit at 0.
        self.hw.lidar.update()

        # SensorFusion::update() steps velocity/pose observers
        self.sf.update(dt)
        pose = self.sf.estimate.pose()
        current = self.sf.estimate.velocity()

        desired = self.planner.update(pose, dt)

        self.mc.update(desired, current, dt)

        self.t += dt
        self.steps += 1

        if desired.v == 0.0 and desired.omega == 0.0:
            self._idle_for += dt
        else:
            self._idle_for = 0.0

        beams = self.hw.lidar_obsv.beams()
        if beams:
            self.beam_loops += 1
            self.beams_total += beams
        self.peak_error = max(self.peak_error, self.position_error())

        if self.steps % TRAIL_DECIMATION == 0:
            self.true_trail.append((self.plant.pose.x, self.plant.pose.y))
            self.est_trail.append((pose.x, pose.y))

        readings = {
            "front": self.hw.lidar.getReading(LIDAR.Front),
            "left": self.hw.lidar.getReading(LIDAR.Left),
            "right": self.hw.lidar.getReading(LIDAR.Right),
        }

        if self.steps % MAP_DECIMATION == 0:
            # Integrated against the ESTIMATE, so the map carries the
            # estimator's error the same way the robot's own map would.
            self.mapper.integrate(pose, readings, L.MAX_DIST)

        frame = Frame(
            t=self.t,
            true_pose=Pose(self.plant.pose.x, self.plant.pose.y, self.plant.pose.theta),
            est_pose=Pose(pose.x, pose.y, pose.theta),
            velocity=Velocity(current.v, current.omega),
            desired=Velocity(desired.v, desired.omega),
            pwm=(self.plant.pwm_left, self.plant.pwm_right),
            readings=readings,
            planner_idx=self._planner_idx(),
            planner_done=self.done(),
            beams=self.hw.lidar_obsv.beams(),
        )
        self.last_frame = frame
        return frame

    # --- state ----------------------------------------------------------

    @property
    def true_pose(self):
        return self.plant.pose

    @property
    def est_pose(self):
        return self.sf.estimate.pose()

    def idle(self):
        return self._idle_for >= IDLE_SECONDS

    def done(self):
        planner_done = getattr(self.planner, "done", None)
        if callable(planner_done) and planner_done():
            return True
        return self.idle()

    def run_until_done(self, max_seconds):
        limit = int(max_seconds * self.loop_hz)
        for _ in range(limit):
            self.step()
            if self.done():
                return True
        return False

    def position_error(self):
        return math.hypot(
            self.true_pose.x - self.est_pose.x, self.true_pose.y - self.est_pose.y
        )

    def _planner_idx(self):
        idx = getattr(self.planner, "idx", None)
        return idx() if callable(idx) else 0

    # --- geometry for the display ---------------------------------------

    def geometry(self):
        """The planner's target geometry as JSON-serialisable data."""
        p = self.planner

        if isinstance(p, MotionPlanner):
            return {
                "kind": "segments",
                "segments": [
                    {
                        "start": [s.start.x, s.start.y],
                        "end": [s.end.x, s.end.y],
                        "centre": [s.c.x, s.c.y],
                        "curvature": s.curvature,
                        "direction": "left"
                        if s.direction == Segment.Direction.Left
                        else "right",
                    }
                    for s in p.path
                ],
            }

        if isinstance(p, PSPlanner):
            return {
                "kind": "grid",
                "targets": [
                    {
                        "cell": [g.x, g.y],
                        "world": [
                            PSPlanner.gridToWorld(g).x,
                            PSPlanner.gridToWorld(g).y,
                        ],
                        "theta": PSPlanner.gridToWorld(g).theta,
                    }
                    for g in p.instructions
                ],
            }

        if isinstance(p, HeadingPlanner):
            return {"kind": "heading", "theta": p.targetTheta}

        if isinstance(p, DistancePlanner):
            return {"kind": "distance", "target": p.targetDistance}

        return {"kind": "none"}
