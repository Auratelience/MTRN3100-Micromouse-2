#!/usr/bin/env python3
"""
Reproduction sim for the PSPlanner turn failure -- a historical artefact.

This is the before/after that justified the heading-only turn fix now in
planners.h, and it is kept as the record of that: run it and the failure is
still there in the "NO fix" columns. Do not extend it. The live PSPlanner lives
in planners.py, is exercised by `python3 run.py task34`, and is kept in step
with the header by tests; this script's copy is frozen at the shape the bug had.

Faithfully ports PosePlanner + PSPlanner from planners.h as they were, and
drives them with the SAME kinematic model as sim.py, then adds the one effect
sim.py explicitly omits -- first-order motor/velocity lag (coast) -- to see what
breaks turns.

Frame: +x forward, +y left, theta CCW, theta=0 -> +x  (mirrors firmware).
"""

import math

# ---- constants (mirror constants.h / .ino) ----
WHEEL_RADIUS = 31.4 / 2.0
MAX_WHEEL_ANGULAR_VELOCITY = 25.0
MAX_FORWARD_VELOCITY = WHEEL_RADIUS * MAX_WHEEL_ANGULAR_VELOCITY  # ~392.5 mm/s
AXLE_LEN = 92.5
MAZE_CELL_SIZE = 180.0
STD_DIST_TOL = 2.0     # positionTolerance default
# constants.h says 0.05; this run was done at 0.1 and the numbers below are
# quoted against it, so it stays 0.1 here. Another reason not to extend this
# file -- planners.py takes STD_ANG_TOL from constants.py and cannot drift.
STD_ANG_TOL = 0.1
TWOPI = 2.0 * math.pi

# ---- gains (mirror .ino: PSPlanner planner(10, 5)) ----
KP_LINEAR = 10.0
KP_ANGULAR = 5.0

DT = 0.001
MAX_ITERS = 60000


def wrap_angle(a):
    if not math.isfinite(a):
        return 0.0
    a = math.fmod(a, TWOPI)
    if a > math.pi:
        a -= TWOPI
    elif a < -math.pi:
        a += TWOPI
    return a


# ---------------- PosePlanner (port of planners.h) ----------------
class PosePlanner:
    SEEK, ALIGN, DONE = "Seek", "Align", "Done"

    def __init__(self, kp_lin, kp_ang, pos_tol=STD_DIST_TOL, ang_tol=STD_ANG_TOL):
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self.pos_tol = pos_tol
        self.ang_tol = ang_tol
        self.target = (0.0, 0.0, 0.0)
        self.state = self.DONE
        self.heading_only = False

    def set_heading_only(self, enabled):
        self.heading_only = enabled

    def set_target(self, t):
        self.target = t
        # Heading-only: skip Seek, rotate in place to target.theta. Used for
        # pure rotations (turns), where the target position == current cell and
        # seeking it would chase a point the robot has coasted past.
        self.state = self.ALIGN if self.heading_only else self.SEEK

    def done(self):
        return self.state == self.DONE

    def update(self, cur):
        if self.state == self.SEEK:
            return self.seek(cur)
        if self.state == self.ALIGN:
            return self.align(cur)
        return (0.0, 0.0)

    def seek(self, cur):
        dx = self.target[0] - cur[0]
        dy = self.target[1] - cur[1]
        dist = math.hypot(dx, dy)
        if dist < self.pos_tol:
            self.state = self.ALIGN
            return self.align(cur)
        angle_to_target = math.atan2(dy, dx)
        heading_err = wrap_angle(angle_to_target - cur[2])
        v = self.kp_lin * dist
        if v > MAX_FORWARD_VELOCITY:
            v = MAX_FORWARD_VELOCITY
        return (v, self.kp_ang * heading_err)

    def align(self, cur):
        heading_err = wrap_angle(self.target[2] - cur[2])
        if abs(heading_err) < self.ang_tol:
            self.state = self.DONE
            return (0.0, 0.0)
        return (0.0, self.kp_ang * heading_err)


# ---------------- PSPlanner (port of planners.h) ----------------
NORTH, WEST, SOUTH, EAST = 0, 1, 2, -1


def grid_to_world(g):
    return (g[0] * MAZE_CELL_SIZE, g[1] * MAZE_CELL_SIZE, wrap_angle(g[2] * (math.pi / 2.0)))


def forwards(g):
    x, y, d = g
    return {NORTH: (x + 1, y, NORTH), EAST: (x, y - 1, EAST),
            SOUTH: (x - 1, y, SOUTH), WEST: (x, y + 1, WEST)}[d]


def left(g):
    x, y, d = g
    return {NORTH: (x, y, WEST), EAST: (x, y, NORTH),
            SOUTH: (x, y, EAST), WEST: (x, y, SOUTH)}[d]


def right(g):
    x, y, d = g
    return {NORTH: (x, y, EAST), EAST: (x, y, SOUTH),
            SOUTH: (x, y, WEST), WEST: (x, y, NORTH)}[d]


class PSPlanner:
    def __init__(self, kp_lin, kp_ang, apply_fix=True):
        self.pp = PosePlanner(kp_lin, kp_ang)
        self.instructions = []
        self.path_idx = 0
        self.apply_fix = apply_fix

    def set_start(self, g):
        self.instructions = [g]

    def add_instructions(self, s):
        for c in s:
            cur = self.instructions[-1]
            if c == 'f':
                self.instructions.append(forwards(cur))
            elif c == 'r':
                self.instructions.append(right(cur))
            elif c == 'l':
                self.instructions.append(left(cur))

    def update(self, pose):
        if self.path_idx >= len(self.instructions):
            return (0.0, 0.0)
        if self.pp.done():
            self.path_idx += 1
            if self.path_idx >= len(self.instructions):
                return (0.0, 0.0)
            curr = self.instructions[self.path_idx]
            prev = self.instructions[self.path_idx - 1]
            # A pure rotation keeps the same cell (x, y); only the heading changes.
            is_turn = (curr[0], curr[1]) == (prev[0], prev[1])
            if self.apply_fix:
                self.pp.set_heading_only(is_turn)
            self.pp.set_target(grid_to_world(curr))
        return self.pp.update(pose)


def ik_scaled(v, omega):
    left_w = (1.0 / WHEEL_RADIUS) * (v - omega * AXLE_LEN / 2.0)
    right_w = (1.0 / WHEEL_RADIUS) * (v + omega * AXLE_LEN / 2.0)
    exceeder = max(abs(left_w), abs(right_w))
    if exceeder > MAX_WHEEL_ANGULAR_VELOCITY:
        s = MAX_WHEEL_ANGULAR_VELOCITY / exceeder
        left_w *= s
        right_w *= s
    v_out = WHEEL_RADIUS * (left_w + right_w) / 2.0
    omega_out = WHEEL_RADIUS * (right_w - left_w) / AXLE_LEN
    return v_out, omega_out


def simulate(instr, tau=0.0, label="", apply_fix=True):
    """tau = velocity first-order lag time constant (s). tau=0 -> ideal (sim.py model)."""
    planner = PSPlanner(KP_LINEAR, KP_ANGULAR, apply_fix=apply_fix)
    planner.set_start((0, 0, NORTH))
    planner.add_instructions(instr)
    print(f"\n=== {label}  (instr='{instr}', tau={tau*1000:.0f} ms) ===")
    print(f"    grid targets: {planner.instructions}")

    pose = [0.0, 0.0, 0.0]
    av, aomega = 0.0, 0.0          # actual (lagged) velocities
    prev_idx = -1
    finished = False
    for i in range(MAX_ITERS):
        v_cmd, omega_cmd = planner.update(pose)
        if planner.pp.done() and planner.path_idx >= len(planner.instructions):
            finished = True
            break

        if planner.path_idx != prev_idx:
            g = planner.instructions[planner.path_idx]
            kind = "TURN " if (planner.path_idx > 0 and
                               (g[0], g[1]) == (planner.instructions[planner.path_idx - 1][0],
                                                planner.instructions[planner.path_idx - 1][1])) else "fwd  "
            print(f"  -> seg {planner.path_idx:2d} {kind} target={grid_to_world(g)}  "
                  f"robot@ x={pose[0]:7.1f} y={pose[1]:7.1f} th={math.degrees(pose[2]):7.1f}")
            prev_idx = planner.path_idx

        v, omega = ik_scaled(v_cmd, omega_cmd)
        if tau > 0.0:
            k = DT / tau
            av += (v - av) * k
            aomega += (omega - aomega) * k
        else:
            av, aomega = v, omega
        pose[0] += av * math.cos(pose[2]) * DT
        pose[1] += av * math.sin(pose[2]) * DT
        pose[2] += aomega * DT

    status = "FINISHED cleanly" if finished else "DID NOT FINISH (hit iter cap)"
    # expected final cell for "ffrfllfrlf" is grid (2,1) -> world (360,180)
    exp = grid_to_world(planner.instructions[-1])
    err = math.hypot(pose[0] - exp[0], pose[1] - exp[1])
    print(f"  {status}: final x={pose[0]:.1f} y={pose[1]:.1f} th={math.degrees(pose[2]):.1f}  "
          f"| expected x={exp[0]:.0f} y={exp[1]:.0f} th={math.degrees(exp[2]):.0f} | pos err={err:.1f} mm")


if __name__ == "__main__":
    INSTR = "ffrfllfrlf"     # exactly what micromouse.ino runs
    print("################  BEFORE FIX (turns routed through Seek)  ################")
    simulate(INSTR, tau=0.060,  label="60 ms lag, NO fix", apply_fix=False)
    simulate(INSTR, tau=0.120,  label="120 ms lag, NO fix", apply_fix=False)

    print("\n\n################  AFTER FIX (heading-only turns)  ################")
    simulate(INSTR, tau=0.000,  label="ideal, fix on", apply_fix=True)
    simulate(INSTR, tau=0.060,  label="60 ms lag, fix on", apply_fix=True)
    simulate(INSTR, tau=0.120,  label="120 ms lag, fix on", apply_fix=True)
