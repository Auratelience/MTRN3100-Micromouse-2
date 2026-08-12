"""Scenario wiring, mirroring the TASK comment blocks in micromouse.ino.

Each scenario reproduces one block's observer/fusion configuration, planner and
gains. Changing a task in the .ino should mean changing exactly one entry here.
"""

import math
from dataclasses import dataclass
from typing import Callable

from .fusion import (
    FusionWeights,
    ObserverPTrust,
    ObserverVTrust,
    PoseSource,
    SensorFusion,
    VelocitySource,
)
from .constants import MAZE_WALL_THICKNESS, PI_TWO
from .planners import DistancePlanner, HeadingPlanner, MotionPlanner, PSPlanner
from .types import Pose, Segment, Vec2D

# MotionController(leftMotor, rightMotor, kinematics, 20.0f, 3.0f, 0.0f)
# kd injects noise since loop speed means minimum alpha = domega/dt is 9 rad/s
DEFAULT_CONTROLLER_GAINS = (20.0, 3.0, 0.0)

# const std::array<PoseSource, 1> obs_p = {{{&lidar_obsv, {0.2, 0.2, 0.1}}}};
#
# XYPTrust-shaped rather than XYPTrust itself: the .ino trusts heading a little,
# not not-at-all. Heading off a square-on wall is only second-order observable
# and the gyro's is better -- see the note above LidarObserver.
LIDAR_POSE_TRUST = ObserverPTrust(0.2, 0.2, 0.1)


@dataclass
class Scenario:
    name: str
    description: str
    start_pose: Pose
    planner_factory: Callable
    fusion_factory: Callable
    controller_gains: tuple = DEFAULT_CONTROLLER_GAINS
    # The obstacle map LidarObserver casts into, i.e. what MAZE_MAP is in the
    # sketch. None means the observer has nothing to match against and every
    # solve returns the prior, so the run is dead reckoning.
    map: object = None
    # Passed to SensorFusion. False is the header verbatim; True applies the
    # one-line fusePose() fix. See fusion.SensorFusion._fusePose.
    seed_pose_mean: bool = False


def _wheel_and_imu_fusion(hw, seed=False):
    """The TASK 3.1 / 3.4 configuration.

    const std::array<VelocitySource, 2> obs_v = {{
        {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
        {&imu_obsv, FusionWeights::OmegaVTrust}
    }};
    """
    return SensorFusion(
        [
            VelocitySource(hw.wheel_obsv, ObserverVTrust(1.0, 0.2)),
            VelocitySource(hw.imu_obsv, FusionWeights.OmegaVTrust),
        ]
    )


def _wheel_v_imu_omega_fusion(hw, seed=False):
    """The TASK 3.3 configuration: wheels for v only, IMU for omega only."""
    return SensorFusion(
        [
            VelocitySource(hw.wheel_obsv, FusionWeights.VVTrust),
            VelocitySource(hw.imu_obsv, FusionWeights.OmegaVTrust),
        ]
    )


def _wheel_imu_and_front_lidar_fusion(hw, seed=False):
    """The TASK 3.2 configuration.

    const std::array<PoseSource, 1> obs_p = {
        {&fl_obsv, FusionWeights::XPTrust}
    };
    SensorFusion sf(obs_v, obs_p, 1);

    A correction gain of 1 rather than the usual 0.2: the front range *is* the
    x measurement for this test, so there is nothing to blend it with.
    """
    return SensorFusion(
        [
            VelocitySource(hw.wheel_obsv, ObserverVTrust(1.0, 0.2)),
            VelocitySource(hw.imu_obsv, FusionWeights.OmegaVTrust),
        ],
        [PoseSource(hw.fl_obsv, FusionWeights.XPTrust)],
        1.0,
        seedPoseMeanWithModel=seed,
    )


def _wheel_imu_and_lidar_pose_fusion(hw, seed=False):
    """The LIDAR LOCALISATION configuration -- what the .ino runs today.

    const std::array<VelocitySource, 2> obs_v = {{
        {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
        {&imu_obsv, FusionWeights::OmegaVTrust}
    }};
    const std::array<PoseSource, 1> obs_p = {{{&lidar_obsv, {0.2, 0.2, 0.1}}}};
    SensorFusion sf(obs_v, obs_p);
    """
    return SensorFusion(
        [
            VelocitySource(hw.wheel_obsv, ObserverVTrust(1.0, 0.2)),
            VelocitySource(hw.imu_obsv, FusionWeights.OmegaVTrust),
        ],
        [PoseSource(hw.lidar_obsv, LIDAR_POSE_TRUST)],
        seedPoseMeanWithModel=seed,
    )


def _task31_planner():
    # NOTE THAT X-AXIS IS FORWARDS: Y-AXIS IS LEFT!!!
    p = MotionPlanner(10, 0.06)
    p.appendSegment(Segment(Vec2D(0, 0), Vec2D(1000, 0)))
    p.appendSegment(
        Segment(Vec2D(1000, 0), Vec2D(1000, -50), 1.0 / 25.0, Segment.Direction.Right)
    )
    p.appendSegment(Segment(Vec2D(1000, -50), Vec2D(0, 0)))
    return p


def _task32_planner():
    p = DistancePlanner(3, 0.06)
    p.setTarget(200.0)
    return p


def _task33_planner():
    p = HeadingPlanner(5)
    p.setTarget(math.pi / 2.0)
    return p


def _task34_planner():
    p = PSPlanner(10, 5)
    p.setStart(PSPlanner.GridPose(0, 0, PSPlanner.Direction.North))
    p.addInstructions("lfrfrflf")
    return p


def task31():
    return Scenario(
        "task31",
        "MotionPlanner: out 1000 mm, right U-turn, back",
        Pose(0.0, 0.0, 0.0),
        _task31_planner,
        _wheel_and_imu_fusion,
    )


def task32():
    return Scenario(
        "task32",
        "DistancePlanner to 200 mm, x corrected by the front lidar",
        Pose(0.0, 0.0, 0.0),
        _task32_planner,
        _wheel_imu_and_front_lidar_fusion,
    )


def task33():
    return Scenario(
        "task33",
        "HeadingPlanner: rotate to +90 degrees",
        Pose(0.0, 0.0, 0.0),
        _task33_planner,
        _wheel_v_imu_omega_fusion,
    )


def task34():
    return Scenario(
        "task34",
        'PSPlanner running "lfrfrflf" (matches the live .ino)',
        Pose(0.0, 0.0, 0.0),
        _task34_planner,
        _wheel_and_imu_fusion,
    )


SCENARIOS = {
    "task31": task31,
    "task32": task32,
    "task33": task33,
    "task34": task34,
}


# TASK 3.2's bench: a wall to range against.
#
# The other three scenarios are planner tests and run in an empty arena, but
# DistancePlanner regulates -pose.x and FrontLidarObserver *is* pose.x, so
# without a wall in front the beam saturates at L::MAX_DIST, the observer
# reports a fixed -300 mm, and the robot reverses for ever against a constant.
# The wall plane is the x origin: FrontLidarObserver writing -range is what
# puts the robot at negative x facing it, and setTarget(200) then means "hold
# 200 mm off the wall".
TASK32_WALL_LENGTH_MM = 1000.0
TASK32_START_X_MM = -250.0


def task32_bench():
    """(world, true start pose) for TASK 3.2.

    The estimator is still told (0, 0, 0), as the sketch does -- with a pose
    correction gain of 1 the first reading snaps x onto the measurement, so
    there is nothing to initialise.
    """
    from .types import Map, Obstacle, WallObstacle
    from .world import MapWorld

    wall = Obstacle(
        WallObstacle(TASK32_WALL_LENGTH_MM, MAZE_WALL_THICKNESS, PI_TWO),
        Vec2D(0.0, 0.0),
    )
    return MapWorld(Map([wall])), Pose(TASK32_START_X_MM, 0.0, 0.0)


# --- PLANNED PATH / LIDAR LOCALISATION ---------------------------------
#
# The block micromouse.ino actually has uncommented today:
#
#     SensorFusion sf(obs_v, obs_p);          // wheels + IMU + lidar pose
#     MotionPlanner planner(10, 0.06f, 200.0f);
#     ...
#     lidar_obsv.setPrior(...);
#     sf.set(Pose{0, 0, 0});                  // the frame maze_map.h was exported in
#     #include "maze_path.h"                  // in setup(), as a body include
#
# Unlike the four task scenarios this one is parameterised, because the path and
# the map are generated artefacts rather than source. cli.py loads them out of
# firmware/micromouse/ by default -- the same two files the sketch compiles
# against -- so "run what is on the robot" needs no arguments at all.

# MotionPlanner planner(10, 0.06f, 200.0f)
PLANNED_KP_HEADING = 10.0
PLANNED_KP_LATERAL = 0.06
PLANNED_CRUISE_MM_S = 200.0


def planned(
    segments,
    map=None,
    localise=True,
    cruise=PLANNED_CRUISE_MM_S,
    start_pose=None,
    seed_pose_mean=False,
    name="planned",
    description=None,
):
    """The live .ino configuration, driving `segments` from maze_path.h.

    `map` is what LidarObserver casts into. With `localise` off the pose source
    is dropped entirely, which is the PLANNED PATH block without the LIDAR
    LOCALISATION one below it: dead reckoning, and the drift it accumulates is
    the thing the fix exists to remove.

    `start_pose` is the pose the *estimator* is told it starts at, i.e.
    sf.set(Pose{0, 0, 0}). Where the robot truly starts is the plant's business
    -- Runner takes that from the scenario too, so offsetting one from the other
    is how the CLI's --start-error puts the robot somewhere the estimator does
    not know about.
    """
    segments = list(segments)

    def planner_factory():
        p = MotionPlanner(PLANNED_KP_HEADING, PLANNED_KP_LATERAL, cruise)
        for s in segments:
            p.appendSegment(s)
        return p

    fusion = _wheel_imu_and_lidar_pose_fusion if localise else _wheel_and_imu_fusion

    if description is None:
        fix = "lidar localisation" if localise and map else "dead reckoning"
        description = (
            f"MotionPlanner over {len(segments)} planned segments "
            f"at {cruise:.0f} mm/s, {fix}"
        )

    return Scenario(
        name,
        description,
        start_pose if start_pose is not None else Pose(0.0, 0.0, 0.0),
        planner_factory,
        fusion,
        map=map if localise else None,
        seed_pose_mean=seed_pose_mean,
    )
