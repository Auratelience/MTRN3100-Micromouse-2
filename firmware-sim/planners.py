"""Mirrors micromouse/planners.h."""

import math
from dataclasses import dataclass
from enum import Enum

from .constants import (
    MAXIMUM_FORWARD_VELOCITY,
    MAZE_CELL_SIZE,
    MAZE_INSTRUCTION_MAX_LEN,
    PATH_SEGMENTS_MAX_LEN,
    PI_TWO,
    PS_POSITION_TOL,
    SEGMENT_ADVANCE_THRESHOLD,
    STD_ANG_TOL,
    STD_DIST_TOL,
    STRAIGHT_TOLERANCE,
)
from .types import Pose, Vec2D, Velocity, arg, dist, dot, wrapAngle


class MotionPlanner:
    class State(Enum):
        Run = "Run"
        Wait = "Wait"

    def __init__(self, KPHeading, KPLateral, cruiseVelocity=MAXIMUM_FORWARD_VELOCITY):
        self.KPHeading = KPHeading
        self.KPLateral = KPLateral
        self.cruiseVelocity = cruiseVelocity
        self.path = []
        self.pathIdx = 0
        self.state = MotionPlanner.State.Wait

    def appendSegment(self, s):
        if len(self.path) >= PATH_SEGMENTS_MAX_LEN:
            return False
        self.path.append(s)
        self.state = MotionPlanner.State.Run
        return True

    def progress(self, pose):
        if self.pathIdx >= len(self.path):
            return 1.0
        return self.path[self.pathIdx].progress(Vec2D(pose.x, pose.y))

    def update(self, pose, dt):
        if self.state == MotionPlanner.State.Run:
            return self._run(pose, dt)
        return self._wait()

    def s(self):
        return self.state

    def idx(self):
        return self.pathIdx

    def done(self):
        return self.state == MotionPlanner.State.Wait

    # --- internals ------------------------------------------------------

    def _wait(self):
        return Velocity(0.0, 0.0)

    def _run(self, pose, dt):
        if self.pathIdx >= len(self.path):
            self.state = MotionPlanner.State.Wait
            return Velocity(0.0, 0.0)

        pos = Vec2D(pose.x, pose.y)

        while (
            self.pathIdx < len(self.path)
            and self.path[self.pathIdx].progress(pos) >= SEGMENT_ADVANCE_THRESHOLD
        ):
            self.pathIdx += 1

        if self.pathIdx >= len(self.path):
            self.state = MotionPlanner.State.Wait
            return Velocity(0.0, 0.0)

        s = self.path[self.pathIdx]
        v = self.cruiseVelocity
        return Velocity(v, self._omega(s, pose, v))

    def _omega(self, s, p, v):
        pos = Vec2D(p.x, p.y)

        if s.curvature <= STRAIGHT_TOLERANCE:
            lineAngle = arg(s.end - s.start)
            headingError = wrapAngle(lineAngle - p.theta)

            line = s.end - s.start
            perpRight = Vec2D(line.y, -line.x)
            lateralError = dot(pos - s.start, perpRight) / dist(line)

            return self.KPHeading * headingError + self.KPLateral * lateralError

        from .types import Segment

        dirScalar = 1.0 if s.direction == Segment.Direction.Left else -1.0
        c = s.c
        nearestPoint = s.lateralPoint(pos)
        radius = nearestPoint - c
        tangentAngle = arg(radius) + dirScalar * PI_TWO
        headingError = wrapAngle(tangentAngle - p.theta)

        lateralError = dirScalar * s.lateralDistance(pos)

        return (
            dirScalar * s.curvature * v
            + self.KPHeading * headingError
            + self.KPLateral * lateralError
        )


class PosePlanner:
    class State(Enum):
        Seek = "Seek"
        Align = "Align"
        Done = "Done"

    def __init__(
        self,
        KPLinear,
        KPAngular,
        positionTolerance=STD_DIST_TOL,
        angleTolerance=STD_ANG_TOL,
    ):
        self.KPLinear = KPLinear
        self.KPAngular = KPAngular
        self.positionTolerance = positionTolerance
        self.angleTolerance = angleTolerance
        self.target = Pose(0.0, 0.0, 0.0)
        self.state = PosePlanner.State.Done
        self.headingOnly = False

    def setTarget(self, t):
        self.target = t
        # Heading-only skips Seek and rotates in place to target.theta. Used
        # for pure rotations (turns), where the target position is the current
        # cell: seeking a point the robot has coasted past yields a near-180
        # deg heading error and a runaway spin instead of a clean turn.
        self.state = (
            PosePlanner.State.Align if self.headingOnly else PosePlanner.State.Seek
        )

    def setHeadingOnly(self, enabled):
        """When enabled, the next setTarget() ignores the target position and
        only rotates to target.theta. Toggle per segment."""
        self.headingOnly = enabled

    def done(self):
        return self.state == PosePlanner.State.Done

    def update(self, current, dt):
        if self.state == PosePlanner.State.Seek:
            return self._seek(current)
        if self.state == PosePlanner.State.Align:
            return self._align(current)
        return Velocity(0.0, 0.0)

    def _seek(self, current):
        dx = self.target.x - current.x
        dy = self.target.y - current.y
        distance = math.hypot(dx, dy)

        if distance < self.positionTolerance:
            self.state = PosePlanner.State.Align
            return self._align(current)

        angleToTarget = arg(Vec2D(dx, dy))
        headingError = wrapAngle(angleToTarget - current.theta)

        v = self.KPLinear * distance
        if v > MAXIMUM_FORWARD_VELOCITY:
            v = MAXIMUM_FORWARD_VELOCITY
        return Velocity(v, self.KPAngular * headingError)

    def _align(self, current):
        headingError = wrapAngle(self.target.theta - current.theta)

        if abs(headingError) < self.angleTolerance:
            self.state = PosePlanner.State.Done
            return Velocity(0.0, 0.0)

        return Velocity(0.0, self.KPAngular * headingError)


class PSPlanner:
    """POSE SEQUENCE PLANNER"""

    class Direction:
        North = 0
        West = 1
        South = 2
        East = -1

    class Instruction(Enum):
        Forwards = "Forwards"
        Left = "Left"
        Right = "Right"

    @dataclass
    class GridPose:
        x: int
        y: int
        direction: int

    def __init__(self, KPLinear, KPAngular):
        self.pp = PosePlanner(KPLinear, KPAngular, PS_POSITION_TOL)
        self.instructions = []
        self.pathIdx = 0

    def setStart(self, g):
        self.instructions.append(g)

    def addInstructions(self, instructions):
        for c in instructions:
            if c == "f":
                ok = self.addInstruction(PSPlanner.Instruction.Forwards)
            elif c == "r":
                ok = self.addInstruction(PSPlanner.Instruction.Right)
            elif c == "l":
                ok = self.addInstruction(PSPlanner.Instruction.Left)
            else:
                continue
            if not ok:
                return False
        return True

    def addInstruction(self, i):
        curr = self.instructions[-1]
        if i == PSPlanner.Instruction.Forwards:
            nxt = PSPlanner.forwards(curr)
        elif i == PSPlanner.Instruction.Right:
            nxt = PSPlanner.right(curr)
        else:
            nxt = PSPlanner.left(curr)
        return self._appendGridPose(nxt)

    def update(self, pose, dt):
        if self.pathIdx >= len(self.instructions):
            return Velocity(0.0, 0.0)

        if self.pp.done():
            self.pathIdx += 1
            if self.pathIdx >= len(self.instructions):
                return Velocity(0.0, 0.0)
            prev = self.instructions[self.pathIdx - 1]
            curr = self.instructions[self.pathIdx]
            # A pure rotation keeps the same cell; only the heading changes.
            # Drive it as a heading-only move so the planner rotates in place
            # rather than seeking a point the robot may have coasted past.
            isTurn = (curr.x == prev.x) and (curr.y == prev.y)
            self.pp.setHeadingOnly(isTurn)
            self.pp.setTarget(PSPlanner.gridToWorld(curr))

        return self.pp.update(pose, dt)

    def done(self):
        return self.pathIdx >= len(self.instructions) - 1 and self.pp.done()

    def idx(self):
        return self.pathIdx

    # --- grid transitions -----------------------------------------------

    @staticmethod
    def forwards(curr):
        D = PSPlanner.Direction
        if curr.direction == D.North:
            return PSPlanner.GridPose(curr.x + 1, curr.y, D.North)
        if curr.direction == D.East:
            return PSPlanner.GridPose(curr.x, curr.y - 1, D.East)
        if curr.direction == D.South:
            return PSPlanner.GridPose(curr.x - 1, curr.y, D.South)
        if curr.direction == D.West:
            return PSPlanner.GridPose(curr.x, curr.y + 1, D.West)
        return curr

    @staticmethod
    def left(curr):
        D = PSPlanner.Direction
        mapping = {D.North: D.West, D.East: D.North, D.South: D.East, D.West: D.South}
        if curr.direction not in mapping:
            return curr
        return PSPlanner.GridPose(curr.x, curr.y, mapping[curr.direction])

    @staticmethod
    def right(curr):
        D = PSPlanner.Direction
        mapping = {D.North: D.East, D.East: D.South, D.South: D.West, D.West: D.North}
        if curr.direction not in mapping:
            return curr
        return PSPlanner.GridPose(curr.x, curr.y, mapping[curr.direction])

    @staticmethod
    def gridToWorld(g):
        return Pose(
            g.x * MAZE_CELL_SIZE, g.y * MAZE_CELL_SIZE, wrapAngle(g.direction * PI_TWO)
        )

    @staticmethod
    def thetaToDirection(theta):
        D = PSPlanner.Direction
        d = int(round(theta / PI_TWO))
        return {0: D.North, 1: D.West, 2: D.South, -1: D.East, -2: D.North}.get(
            d, D.North
        )

    @staticmethod
    def worldToGrid(p):
        return PSPlanner.GridPose(
            int(round(p.x / MAZE_CELL_SIZE)),
            int(round(p.y / MAZE_CELL_SIZE)),
            PSPlanner.thetaToDirection(p.theta),
        )

    def _appendGridPose(self, g):
        if len(self.instructions) >= MAZE_INSTRUCTION_MAX_LEN:
            return False
        self.instructions.append(g)
        return True


class HeadingPlanner:
    def __init__(self, KPAngular, angleTolerance=STD_ANG_TOL):
        self.KPAngular = KPAngular
        self.angleTolerance = angleTolerance
        self.targetTheta = 0.0

    def setTarget(self, theta):
        self.targetTheta = theta

    def update(self, current, dt):
        headingError = wrapAngle(self.targetTheta - current.theta)

        if abs(headingError) < self.angleTolerance:
            return Velocity(0.0, 0.0)

        return Velocity(0.0, self.KPAngular * headingError)

    def done(self):
        return False


class DistancePlanner:
    def __init__(self, KpDistance, KpHeading):
        self.KpDistance = KpDistance
        self.KpHeading = KpHeading
        self.targetDistance = 0.0

    def setTarget(self, distance):
        self.targetDistance = distance

    def update(self, current, dt):
        current_distance = -current.x
        distance_error = current_distance - self.targetDistance

        # No deadband: the header has none either. A proportional-only law with
        # nothing to hold it means the robot creeps until the PWM falls under
        # the motor deadband, which is exactly the behaviour worth seeing here.
        forward_velocity = self.KpDistance * distance_error
        if forward_velocity > MAXIMUM_FORWARD_VELOCITY:
            forward_velocity = MAXIMUM_FORWARD_VELOCITY
        if forward_velocity < -MAXIMUM_FORWARD_VELOCITY:
            forward_velocity = -MAXIMUM_FORWARD_VELOCITY

        heading_error = wrapAngle(0.0 - current.theta)
        omega = self.KpHeading * heading_error

        return Velocity(forward_velocity, omega)

    def done(self):
        return False
