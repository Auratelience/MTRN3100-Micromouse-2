#!/usr/bin/env python3
"""
Kinematic simulation of the Micromouse MotionPlanner + steering law.

SUPERSEDED by the package next to it -- `python3 run.py task31` runs the same
path with a plant, sensors and an estimator underneath, and its MotionPlanner is
kept in step with planners.h by tests. This file is the standalone version that
came first: one script, no imports, an ASCII plot, and the perfect-tracking
assumption written out where you can see it. Kept because that makes it the
quickest way to answer "is this path geometry sane" without any of the rest.

It is a *geometry / planner* sim, not a dynamics sim. It assumes the inner
velocity PID perfectly tracks the commanded body velocity, and it applies the
same IK magnitude-clamp as kinematics.h. It has no motor lag, no encoder
quantisation and no sensor noise -- so it is the right tool for checking path
geometry and planner logic (segment advancement, turn direction, does the
robot reach the goal), and the WRONG tool for tuning kp/ki.

Constants mirror constants.h; gains/path mirror micromouse.ino's TASK 3.1
block. Frame matches the firmware: +x is forward, +y is left, theta grows CCW.

Run:  python3 sim.py
"""

import math

# ---- constants (mirror constants.h) ----
WHEEL_RADIUS = 31.4 / 2.0
MAX_WHEEL_ANGULAR_VELOCITY = 25.0
MAX_FORWARD_VELOCITY = WHEEL_RADIUS * MAX_WHEEL_ANGULAR_VELOCITY  # ~392.5 mm/s
AXLE_LEN = 92.5
STRAIGHT_TOLERANCE = 0.001
SEGMENT_ADVANCE_THRESHOLD = 0.995
TWOPI = 2.0 * math.pi

# ---- gains (mirror micromouse.ino) ----
KP_HEADING = 10.0
KP_LATERAL = 0.06

DT = 0.001          # integration step (s)
MAX_ITERS = 20000


# ---- vector helpers ----
def sub(a, b):   return (a[0] - b[0], a[1] - b[1])
def add(a, b):   return (a[0] + b[0], a[1] + b[1])
def mul(s, a):   return (s * a[0], s * a[1])
def dot(a, b):   return a[0] * b[0] + a[1] * b[1]
def cross(a, b): return a[0] * b[1] - a[1] * b[0]
def norm(a):     return math.hypot(a[0], a[1])
def arg(a):      return math.atan2(a[1], a[0])


def wrap_angle(a):
    a = math.fmod(a, TWOPI)
    if a > math.pi:
        a -= TWOPI
    elif a < -math.pi:
        a += TWOPI
    return a


class Segment:
    """Port of Segment in types.h (line + arc primitives)."""

    LEFT, RIGHT = "L", "R"

    def __init__(self, start, end, curvature=0.0, direction=LEFT):
        self.start = start
        self.end = end
        self.curvature = curvature
        self.direction = direction
        self.c = self._centre()

    def _centre(self):
        m = mul(0.5, add(self.start, self.end))
        if self.curvature <= STRAIGHT_TOLERANCE:
            return m
        r = 1.0 / self.curvature
        chord = sub(self.end, self.start)
        half = 0.5 * norm(chord)
        dir_scalar = 1.0 if self.direction == self.LEFT else -1.0
        disc = max(0.0, r * r - half * half)          # guard infeasible radius
        h = math.sqrt(disc)
        scale = (dir_scalar * h) / norm(chord)
        perp_l90 = (-chord[1], chord[0])
        return add(m, mul(scale, perp_l90))

    def lateral_point(self, pos):
        if self.curvature <= STRAIGHT_TOLERANCE:
            line = sub(self.end, self.start)
            t = max(0.0, min(1.0, dot(sub(pos, self.start), line) / dot(line, line)))
            return add(self.start, mul(t, line))
        r = 1.0 / self.curvature
        pc = sub(pos, self.c)
        return add(self.c, mul(r / norm(pc), pc))

    def lateral_distance(self, pos):
        if self.curvature <= STRAIGHT_TOLERANCE:
            line = sub(self.end, self.start)
            t = max(0.0, min(1.0, dot(sub(pos, self.start), line) / dot(line, line)))
            return norm(sub(pos, add(self.start, mul(t, line))))
        return norm(sub(pos, self.c)) - 1.0 / self.curvature

    def _arc_travel(self, frm, to):
        delta = (to - frm) if self.direction == self.LEFT else (frm - to)
        while delta < 0.0:
            delta += TWOPI
        while delta >= TWOPI:
            delta -= TWOPI
        return delta

    def progress(self, pos):
        if self.curvature <= STRAIGHT_TOLERANCE:
            line = sub(self.end, self.start)
            return dot(sub(pos, self.start), line) / dot(line, line)
        start_angle = arg(sub(self.start, self.c))
        sweep = self._arc_travel(start_angle, arg(sub(self.end, self.c)))
        if sweep <= 0.0:
            return 1.0
        travelled = self._arc_travel(start_angle, arg(sub(pos, self.c)))
        if travelled > 0.5 * (sweep + TWOPI):   # behind start -> negative
            travelled -= TWOPI
        return travelled / sweep

    def omega(self, pose, v):
        pos = (pose[0], pose[1])
        theta = pose[2]
        if self.curvature <= STRAIGHT_TOLERANCE:
            line_angle = arg(sub(self.end, self.start))
            heading_err = wrap_angle(line_angle - theta)
            line = sub(self.end, self.start)
            perp_right = (line[1], -line[0])
            lateral_err = dot(sub(pos, self.start), perp_right) / norm(line)
            return KP_HEADING * heading_err + KP_LATERAL * lateral_err
        dir_scalar = 1.0 if self.direction == self.LEFT else -1.0
        nearest = self.lateral_point(pos)
        tangent = arg(sub(nearest, self.c)) + dir_scalar * (math.pi / 2.0)
        heading_err = wrap_angle(tangent - theta)
        lateral_err = dir_scalar * self.lateral_distance(pos)
        return dir_scalar * self.curvature * v + KP_HEADING * heading_err + KP_LATERAL * lateral_err


class MotionPlanner:
    """Port of MotionPlanner in planners.h."""

    def __init__(self, path, cruise_velocity=MAX_FORWARD_VELOCITY):
        self.path = path
        self.idx = 0
        self.cruise = cruise_velocity
        self.done = False

    def update(self, pose):
        if self.idx >= len(self.path):
            self.done = True
            return (0.0, 0.0)
        pos = (pose[0], pose[1])
        while self.idx < len(self.path) and self.path[self.idx].progress(pos) >= SEGMENT_ADVANCE_THRESHOLD:
            self.idx += 1
        if self.idx >= len(self.path):
            self.done = True
            return (0.0, 0.0)
        v = self.cruise
        return (v, self.path[self.idx].omega(pose, v))


def ik_scaled(v, omega):
    """Body (v, omega) -> body (v, omega) after IK magnitude clamp (kinematics.h)."""
    left = (1.0 / WHEEL_RADIUS) * (v - omega * AXLE_LEN / 2.0)
    right = (1.0 / WHEEL_RADIUS) * (v + omega * AXLE_LEN / 2.0)
    exceeder = max(abs(left), abs(right))
    if exceeder > MAX_WHEEL_ANGULAR_VELOCITY:
        s = MAX_WHEEL_ANGULAR_VELOCITY / exceeder
        left *= s
        right *= s
    v_out = WHEEL_RADIUS * (left + right) / 2.0
    omega_out = WHEEL_RADIUS * (right - left) / AXLE_LEN
    return v_out, omega_out


def simulate(path, cruise=MAX_FORWARD_VELOCITY, start_pose=(0.0, 0.0, 0.0), verbose=True):
    planner = MotionPlanner(path, cruise)
    pose = list(start_pose)
    traj = [tuple(pose)]
    prev_idx = -1
    i = 0
    for i in range(MAX_ITERS):
        v_cmd, omega_cmd = planner.update(pose)
        if planner.done:
            break
        if verbose and planner.idx != prev_idx:
            print(f"  -> entering segment {planner.idx} at "
                  f"x={pose[0]:7.1f} y={pose[1]:7.1f} th={math.degrees(pose[2]):7.1f} deg")
            prev_idx = planner.idx
        v, omega = ik_scaled(v_cmd, omega_cmd)
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
        pose[2] += omega * DT
        traj.append(tuple(pose))
    if verbose:
        status = "reached goal (planner idle)" if planner.done else "hit iteration cap"
        print(f"  {status}: final x={pose[0]:.1f} y={pose[1]:.1f} "
              f"th={math.degrees(pose[2]):.1f} deg after {i} steps")
    return traj


def ascii_plot(traj, width=72, height=22):
    xs = [p[0] for p in traj]
    ys = [p[1] for p in traj]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    sx = (width - 1) / (x1 - x0) if x1 > x0 else 0
    sy = (height - 1) / (y1 - y0) if y1 > y0 else 0
    grid = [[" "] * width for _ in range(height)]
    for (x, y, _) in traj:
        cx = int((x - x0) * sx)
        cy = int((y1 - y) * sy)          # +y up
        grid[cy][cx] = "."
    sx0, sy0 = int((traj[0][0] - x0) * sx), int((y1 - traj[0][1]) * sy)
    grid[sy0][sx0] = "S"
    ex0, ey0 = int((traj[-1][0] - x0) * sx), int((y1 - traj[-1][1]) * sy)
    grid[ey0][ex0] = "E"
    print(f"\n  x:[{x0:.0f},{x1:.0f}] y:[{y0:.0f},{y1:.0f}] mm  (aspect NOT 1:1; +y=up)")
    for row in grid:
        print("  |" + "".join(row) + "|")


if __name__ == "__main__":
    # Path mirrors the TASK 3.1 setup in micromouse.ino.
    # NOTE: +x is forward, +y is LEFT, so Direction.RIGHT curves toward -y.
    path = [
        Segment((0, 0), (1000, 0)),
        Segment((1000, 0), (1000, -50), 1.0 / 25.0, Segment.RIGHT),
        Segment((1000, -50), (0, 0)),
    ]
    print("Simulating TASK 3.1 path (out, right U-turn, back):")
    traj = simulate(path)
    ascii_plot(traj)
