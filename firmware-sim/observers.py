"""Mirrors micromouse/observers.h.

SimMotor and SimIMU are sim-only adapters standing in for Motor<ID> and IMU,
exposing just the surface the observers use.

One deliberate deviation: the header's LidarObserver rate-limits itself against
``millis()``. There is no global clock here, so it accumulates the ``dt`` it is
already handed. At loop rate the two are the same thing.
"""

import math

from .constants import (
    IMU_STARTUP_READING_COUNT,
    LIDAR_CONTINUOUS_PERIOD_MS,
    LIDAR_MOUNT_FRONT_THETA,
    LIDAR_MOUNT_FRONT_X,
    LIDAR_MOUNT_FRONT_Y,
    LIDAR_MOUNT_LEFT_THETA,
    LIDAR_MOUNT_LEFT_X,
    LIDAR_MOUNT_LEFT_Y,
    LIDAR_MOUNT_RIGHT_THETA,
    LIDAR_MOUNT_RIGHT_X,
    LIDAR_MOUNT_RIGHT_Y,
    LIDAR_OBSERVER_ITERATIONS,
    LIDAR_OBSERVER_LAMBDA,
    LIDAR_OBSERVER_MAX_CANDIDATES,
    LIDAR_OBSERVER_MAX_RESIDUAL_MM,
    LIDAR_OBSERVER_MAX_STEP_MM,
    LIDAR_OBSERVER_MAX_STEP_RAD,
    LIDAR_OBSERVER_MIN_INCIDENCE_COS,
    LIDAR_OBSERVER_PRIOR_SIGMA_MM,
    LIDAR_OBSERVER_PRIOR_SIGMA_RAD,
    LIDAR_OBSERVER_SEARCH_RADIUS_MM,
    LIDAR_OBSERVER_STEP_TOL_MM,
    LIDAR_OBSERVER_STEP_TOL_RAD,
    STD_TOL,
)
from .lidar import L, LIDAR
from .types import Pose, Vec2D, Velocity, WheelVelocities, dot, perp, wrapAngle


class ObserverP:
    """Produces pose estimates."""

    def estimate(self):
        raise NotImplementedError

    def set(self, p):
        raise NotImplementedError

    def update(self, dt):
        raise NotImplementedError

    def ready(self):
        """False if an estimate is not yet ready."""
        raise NotImplementedError


class ObserverV:
    """Produces velocity estimates. dt is in seconds."""

    def estimate(self):
        raise NotImplementedError

    def set(self, v):
        raise NotImplementedError

    def update(self, dt):
        raise NotImplementedError

    def ready(self):
        raise NotImplementedError


# --- hardware adapters (sim-only) ---------------------------------------


class SimMotor:
    """Stands in for Motor<ID>. Encoder reads come from the plant's quantised
    counts; move()/stop() write PWM back into it."""

    def __init__(self, plant, side):
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        self.plant = plant
        self.side = side

    def angularDisplacement(self):
        if self.side == "left":
            return self.plant.angular_displacement_left()
        return self.plant.angular_displacement_right()

    def linearDisplacement(self):
        from .constants import WHEEL_RADIUS

        return self.angularDisplacement() * WHEEL_RADIUS

    def count(self):
        if self.side == "left":
            return self.plant.count_left()
        return self.plant.count_right()

    def move(self, speed):
        self.plant.set_pwm_side(self.side, speed)

    def stop(self):
        self.plant.set_pwm_side(self.side, 0)


class SimIMU:
    """Stands in for IMU, matching the accessor names ImuObserver calls."""

    def __init__(self, plant):
        self.plant = plant

    def gyroZ(self):
        return self.plant.gyro_z()

    def accelX(self):
        return self.plant.accel_x()

    def accelY(self):
        return self.plant.accel_y()


# --- observers ----------------------------------------------------------


class WheelObserver(ObserverV):
    def __init__(self, left, right, model):
        self.left = left
        self.right = right
        self.model = model
        self.left_prev_rad = left.angularDisplacement()
        self.right_prev_rad = right.angularDisplacement()
        self.velocity = Velocity(0.0, 0.0)

    def estimate(self):
        return self.velocity

    def set(self, v):
        """No-op since there is nothing to set."""

    def update(self, dt):
        left_curr_rad = self.left.angularDisplacement()
        right_curr_rad = self.right.angularDisplacement()

        # Note that dt is assumed to be in seconds, not millis
        vw = WheelVelocities(
            (left_curr_rad - self.left_prev_rad) / dt,
            (right_curr_rad - self.right_prev_rad) / dt,
        )

        self.left_prev_rad = left_curr_rad
        self.right_prev_rad = right_curr_rad

        self.velocity = self.model.fk.velocity(vw)

    def ready(self):
        """Can calculate whenever."""
        return True


class ImuObserver(ObserverV):
    def __init__(self, imu):
        self.imu = imu
        self.imu_velocity = 0.0
        self.velocity = Velocity(0.0, 0.0)
        self.gyro_z_drift = 0.0
        self.accel_x_drift = 0.0
        self.accel_y_drift = 0.0
        self.obsv_init = False

    def init(self):
        """Measures gyro drift at rest. The hardware version sleeps
        IMU_STARTUP_SETTLE_MS then samples every IMU_STARTUP_READING_DELAY_MS;
        there is nothing to wait for in simulation, so the delays are dropped
        and only the averaging is kept."""
        gz = ax = ay = 0.0
        for _ in range(IMU_STARTUP_READING_COUNT):
            gz += self.imu.gyroZ()
            ax += self.imu.accelX()
            ay += self.imu.accelY()
        self.gyro_z_drift = gz / IMU_STARTUP_READING_COUNT
        self.accel_x_drift = ax / IMU_STARTUP_READING_COUNT
        self.accel_y_drift = ay / IMU_STARTUP_READING_COUNT
        self.obsv_init = True

    def estimate(self):
        if not self.obsv_init:
            return Velocity(0.0, 0.0)
        return self.velocity

    def update(self, dt):
        # Accelerometer integration is commented out in the firmware as "too
        # unreliable for use"; the same path is left inert here.
        self.velocity = Velocity(self.imu_velocity, self.imu.gyroZ() - self.gyro_z_drift)

    def set(self, v):
        self.imu_velocity = v.v

    def ready(self):
        return self.obsv_init


class ModelObserver(ObserverP):
    """Dead reckoning. velocity_func and pose_func stand in for the
    etl::delegate pair the firmware wires up; SensorFusion depends on that
    indirection to feed its own fused velocity back in."""

    def __init__(self, velocity_func, pose_func=None):
        self.velocity_func = velocity_func
        self.pose_func = pose_func
        self.previous_pose = Pose(0.0, 0.0, 0.0)
        self.pose = Pose(0.0, 0.0, 0.0)

    def estimate(self):
        return self.pose

    def set(self, p):
        self.pose = Pose(p.x, p.y, p.theta)

    def update(self, dt):
        if self.pose_func is None:
            self.previous_pose = self.pose
        else:
            self.previous_pose = self.pose_func()

        current_velocity = self.velocity_func()
        prev = self.previous_pose
        self.pose = Pose(
            prev.x + current_velocity.v * math.cos(prev.theta) * dt,
            prev.y + current_velocity.v * math.sin(prev.theta) * dt,
            prev.theta + current_velocity.omega * dt,
        )

    def ready(self):
        return True

    def setFunctionPointers(self, vf, pf):
        self.velocity_func = vf
        self.pose_func = pf


class FrontLidarObserver(ObserverP):
    """Pose x from the front range alone, for the TASK 3.2 wall-distance test.

    Only meaningful when the robot is square on to a wall it started a known
    distance from; it is not a general pose observer. LidarObserver is.
    """

    def __init__(self, lidar):
        self.lidar = lidar
        self.pose = Pose(0.0, 0.0, 0.0)

    def update(self, dt):
        self.lidar.update()
        self.pose.x = -self.lidar.getReading(LIDAR.Front)

    def set(self, p):
        self.pose = Pose(p.x, p.y, p.theta)

    def estimate(self):
        return self.pose

    def ready(self):
        return True


class LidarObserver(ObserverP):
    """Pose from the lidar, by matching what the beams should read against what
    they do read. Port of LidarObserver<S> in observers.h.

    Every solve starts from a prior -- the dead-reckoned pose SensorFusion is
    about to correct -- casts the three beams into the map from there, and forms
    a residual per beam, r = measured - expected. Each residual is one equation
    in the pose error, r = J . delta, so three beams give three equations in
    three unknowns, solved as a damped least squares step against the prior.

    Three beams are rarely three independent equations, and the prior is what
    makes that ordinary rather than special: each axis carries one extra
    equation saying it should stay where dead reckoning put it, weighted by how
    far dead reckoning is trusted to be out. Unobservable directions then sit on
    the prior, and observable ones are still free to move. The full argument,
    and the Jacobian's derivation from the hit normal, are in observers.h.

    Wiring, as in setup():

        lidar_obsv = LidarObserver(lidar, MAZE_MAP)
        sf = SensorFusion(obs_v, [PoseSource(lidar_obsv, ObserverPTrust(...))])
        lidar_obsv.setPrior(lambda: sf.estimate.pose())

    Left unwired it falls back to its own last estimate, which is only useful
    for bench testing a single solve. With no map it has nothing to cast
    against, so every solve returns the prior unchanged -- a zero correction,
    which is the dead-reckoning behaviour the old stub had.
    """

    # Indexed by LIDAR sensor id.
    MOUNTS = (
        (LIDAR_MOUNT_FRONT_X, LIDAR_MOUNT_FRONT_Y, LIDAR_MOUNT_FRONT_THETA),
        (LIDAR_MOUNT_LEFT_X, LIDAR_MOUNT_LEFT_Y, LIDAR_MOUNT_LEFT_THETA),
        (LIDAR_MOUNT_RIGHT_X, LIDAR_MOUNT_RIGHT_Y, LIDAR_MOUNT_RIGHT_THETA),
    )

    def __init__(self, lidar, map=None, prior=None):
        self.lidar = lidar
        self.map = map
        self.prior_func = prior
        self.pose = Pose(0.0, 0.0, 0.0)
        self.candidate_idx = []
        self.candidate_overflow = False
        self.beams_used = 0
        # Stands in for millis(): accumulated from the dt update() is handed.
        self._elapsed_ms = 0.0
        self._last_sample_ms = -float("inf")

    def setPrior(self, p):
        self.prior_func = p

    def set(self, p):
        self.pose = Pose(p.x, p.y, p.theta)

    def estimate(self):
        return self.pose

    def ready(self):
        """Always ready. SensorFusion tests ready() before calling update(), so
        an observer that reports itself unready is never updated again and can
        never come back; a cycle with nothing to say returns the prior instead,
        which fusePose() folds in as a zero correction."""
        return True

    def beams(self):
        """Beams that survived gating in the last solve. Zero means the
        estimate is the prior, unchanged."""
        return self.beams_used

    def update(self, dt):
        prior = self.prior_func() if self.prior_func is not None else self.pose
        prior = Pose(prior.x, prior.y, prior.theta)
        self.pose = Pose(prior.x, prior.y, prior.theta)
        self.beams_used = 0

        # The VL6180X is free-running at LIDAR_CONTINUOUS_PERIOD_MS, so
        # re-reading it at loop rate spends I2C on repeats of the same value
        # and re-solves against them.
        self._elapsed_ms += dt * 1000.0
        if self._elapsed_ms - self._last_sample_ms < LIDAR_CONTINUOUS_PERIOD_MS:
            return
        self._last_sample_ms = self._elapsed_ms
        self.lidar.update()

        if self.map is None or len(self.map) == 0:
            return

        # Broad phase once per solve, around the prior: the iterations below
        # move the pose by millimetres, far less than the margin in
        # LIDAR_OBSERVER_SEARCH_RADIUS_MM.
        #
        # A full list may have been truncated, and a dropped obstacle could be
        # the nearest one, so that case gives up the broad phase and casts
        # against everything. Slower, never wrong.
        self.candidate_idx = self.map.candidates(
            Vec2D(prior.x, prior.y),
            LIDAR_OBSERVER_SEARCH_RADIUS_MM,
            LIDAR_OBSERVER_MAX_CANDIDATES,
        )
        self.candidate_overflow = (
            len(self.candidate_idx) == LIDAR_OBSERVER_MAX_CANDIDATES
        )

        p = Pose(prior.x, prior.y, prior.theta)
        for _ in range(LIDAR_OBSERVER_ITERATIONS):
            # Normal equations, H = sum J J^T and g = sum J r, accumulated over
            # the beams that pass gating.
            H = [[0.0] * 3 for _ in range(3)]
            g = [0.0, 0.0, 0.0]
            self.beams_used = 0

            for s in range(LIDAR.COUNT):
                eq = self._beamEquation(p, s)
                if eq is None:
                    continue
                J, r = eq
                for i in range(3):
                    g[i] += J[i] * r
                    for j in range(3):
                        H[i][j] += J[i] * J[j]
                self.beams_used += 1

            if self.beams_used == 0:
                # Nothing to correct with. Hand back the prior so the fusion
                # sees a zero correction rather than a stale absolute pose.
                # Losing every beam on a later iteration means a step of a few
                # mm walked the solve off its own surfaces, so the whole solve
                # is abandoned rather than half-applied.
                self.pose = Pose(prior.x, prior.y, prior.theta)
                return

            # Fold in the prior. Each axis gets one more equation, weighted by
            # 1/sigma^2, saying it should stay where dead reckoning put it --
            # which both makes H invertible whatever the beams did and settles
            # how a correction is split across a direction they cannot see. The
            # right hand side is the prior's own residual, zero on the first
            # iteration and non-zero once the pose has moved.
            w = (
                1.0 / (LIDAR_OBSERVER_PRIOR_SIGMA_MM * LIDAR_OBSERVER_PRIOR_SIGMA_MM),
                1.0 / (LIDAR_OBSERVER_PRIOR_SIGMA_MM * LIDAR_OBSERVER_PRIOR_SIGMA_MM),
                1.0 / (LIDAR_OBSERVER_PRIOR_SIGMA_RAD * LIDAR_OBSERVER_PRIOR_SIGMA_RAD),
            )
            prior_residual = (
                prior.x - p.x,
                prior.y - p.y,
                wrapAngle(prior.theta - p.theta),
            )

            for i in range(3):
                H[i][i] = H[i][i] * (1.0 + LIDAR_OBSERVER_LAMBDA) + w[i]
                g[i] += w[i] * prior_residual[i]

            delta = self._solve3(H, g)
            if delta is None:
                break

            dx = _clampAbs(delta[0], LIDAR_OBSERVER_MAX_STEP_MM)
            dy = _clampAbs(delta[1], LIDAR_OBSERVER_MAX_STEP_MM)
            dth = _clampAbs(delta[2], LIDAR_OBSERVER_MAX_STEP_RAD)

            p.x += dx
            p.y += dy
            p.theta = wrapAngle(p.theta + dth)

            if (
                abs(dx) < LIDAR_OBSERVER_STEP_TOL_MM
                and abs(dy) < LIDAR_OBSERVER_STEP_TOL_MM
                and abs(dth) < LIDAR_OBSERVER_STEP_TOL_RAD
            ):
                break

        self.pose = p

    # --- internals ------------------------------------------------------

    def _beamEquation(self, p, sensor):
        """One beam's contribution as (J, r), or None if the reading carries
        nothing worth folding in."""
        measured = float(self.lidar.getReading(sensor))

        # Both clamps mean "no target", not "target at 0 mm" or "at 300 mm" --
        # see LidarSensor::read(). A saturated beam only says the wall is
        # somewhere beyond the ceiling, which is not an equation.
        if measured <= float(L.MIN_DIST) or measured >= float(L.MAX_DIST):
            return None

        mx, my, mtheta = self.MOUNTS[sensor]
        c = math.cos(p.theta)
        s = math.sin(p.theta)

        # Mount offset in world axes, kept separate because it doubles as
        # ds/dtheta once rotated a further quarter turn.
        offset = Vec2D(mx * c - my * s, mx * s + my * c)
        origin = Vec2D(p.x, p.y) + offset
        heading = p.theta + mtheta
        beam = Vec2D(math.cos(heading), math.sin(heading))

        # A little past the sensor's ceiling, so a prediction that lands just
        # beyond it is still available to be rejected by the residual gate
        # rather than silently becoming a miss.
        hit = self.map.cast(
            origin,
            beam,
            float(L.MAX_DIST) + LIDAR_OBSERVER_MAX_RESIDUAL_MM,
            None if self.candidate_overflow else self.candidate_idx,
        )
        if not hit.valid:
            return None

        nb = dot(hit.normal, beam)  # negative: the beam runs into the face
        if -nb < LIDAR_OBSERVER_MIN_INCIDENCE_COS:
            return None  # grazing

        # Association. A residual this large is a beam looking at something the
        # map does not hold, or at a different surface entirely, and folding it
        # in would drag the pose rather than fix it. Gating on
        # impliedHeadingError() instead looks sharper but is not: it asks how
        # far the beam would have to swing to explain the reading, and a beam
        # pointed square at a wall barely changes range when it swings, so it
        # throws away precisely the beams that say the most about position.
        r = measured - hit.distance
        if abs(r) > LIDAR_OBSERVER_MAX_RESIDUAL_MM:
            return None

        J = (
            -hit.normal.x / nb,
            -hit.normal.y / nb,
            -(dot(hit.normal, perp(offset)) + hit.distance * dot(hit.normal, perp(beam)))
            / nb,
        )
        return J, r

    @staticmethod
    def _solve3(H, g):
        """H x = g, by the adjugate. Damping has already put a floor under every
        diagonal, including the rows no beam constrains, so the determinant only
        vanishes here if H is degenerate for some other reason. None on
        failure."""
        adj = (
            (
                H[1][1] * H[2][2] - H[1][2] * H[2][1],
                H[0][2] * H[2][1] - H[0][1] * H[2][2],
                H[0][1] * H[1][2] - H[0][2] * H[1][1],
            ),
            (
                H[1][2] * H[2][0] - H[1][0] * H[2][2],
                H[0][0] * H[2][2] - H[0][2] * H[2][0],
                H[0][2] * H[1][0] - H[0][0] * H[1][2],
            ),
            (
                H[1][0] * H[2][1] - H[1][1] * H[2][0],
                H[0][1] * H[2][0] - H[0][0] * H[2][1],
                H[0][0] * H[1][1] - H[0][1] * H[1][0],
            ),
        )

        det = H[0][0] * adj[0][0] + H[0][1] * adj[1][0] + H[0][2] * adj[2][0]
        if abs(det) < STD_TOL:
            return None

        inv = 1.0 / det
        return tuple(
            inv * (adj[i][0] * g[0] + adj[i][1] * g[1] + adj[i][2] * g[2])
            for i in range(3)
        )


def _clampAbs(v, limit):
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v
