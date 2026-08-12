"""Mirrors micromouse/constants.h.

Values are transcribed verbatim. Derived constants stay derived so a change to
a base value propagates the same way it does in the header.
"""

import math

# MATH
PI = math.pi
TWO_PI = 2.0 * math.pi
PI_TWO = PI / 2.0
PI_FOUR = PI / 4.0

# Resolution of the atan/acos lookup tables in trig<> (see types.h). The sim
# does not use the LUTs (see sim/types.py) but keeps the constant for parity.
TRIG_LUT_SIZE = 256

# WHEEL & KINEMATICS
# Effective rolling diameter (mm). test implies ~31.4 effective
WHEEL_DIAMETER = 31.4
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
ENC_CPR = 700

# Per-wheel encoder calibration. Hand-measured angularDisplacement() for
# exactly one wheel revolution (rad), 2026-07-17. Readings are scaled by
# TWOPI / measured so a full revolution reads exactly 2 pi.
ENC_RAD_PER_REV_LEFT = 6.09
ENC_RAD_PER_REV_RIGHT = 6.24
ENC_SCALE_LEFT = TWO_PI / ENC_RAD_PER_REV_LEFT
ENC_SCALE_RIGHT = TWO_PI / ENC_RAD_PER_REV_RIGHT
AXLE_LEN = 92.5
AXLE_DIST_FROM_CENTRE = 20

MAXIMUM_WHEEL_PWM = 255
MAXIMUM_LATERAL_ACCELERATION = 4  # m/s^2
MAXIMUM_WHEEL_ANGULAR_VELOCITY = 25  # rad/s

# CONTROL LOOP
MIN_LOOP_DT_S = 0.000005  # 5 us

# MOTION
PATH_SEGMENTS_MAX_LEN = 256
STRAIGHT_TOLERANCE = 0.001
MAXIMUM_FORWARD_VELOCITY = WHEEL_RADIUS * MAXIMUM_WHEEL_ANGULAR_VELOCITY
MAXIMUM_ANGULAR_ACCELERATION = 20.0  # rad/s^2
STD_TOL = 1e-6
STD_DIST_TOL = 2.0
STD_ANG_TOL = 0.05
SEGMENT_ADVANCE_THRESHOLD = 0.995

# MAZE
# Physical size of one grid cell, mm. Full-size Micromouse uses 180mm,
# half-size 168mm. Shared by the path factory and the instruction runner.
MAZE_CELL_SIZE = 180.0

# Panel and post geometry, mirroring WALL_T_MM / POST_T_MM in
# path-planning/maze_map.py. The post radius is the circumscribed circle of
# the square section, so a post is never under-covered.
MAZE_WALL_THICKNESS = 12.0
MAZE_POST_SIZE = 12.0
MAZE_POST_RADIUS = MAZE_POST_SIZE * 0.7071068

# Longest instruction string ("frfllflr...") the InstructionRunner accepts,
# excluding the null terminator.
MAZE_INSTRUCTION_MAX_LEN = 256

# PID
PID_SATURATION_ABSOLUTE = 1000.0

# IMU
IMU_STARTUP_SETTLE_MS = 500
IMU_STARTUP_READING_COUNT = 500
IMU_STARTUP_READING_DELAY_MS = 5
IMU_ROLLING_AVG_SAMPLES = 8

# LIDAR
LIDAR_TIMEOUT_MS = 50
LIDAR_CONTINUOUS_PERIOD_MS = 10  # minimum: 10 ms
LIDAR_BRINGUP_SETTLE_MS = 1

# Sensor mount poses relative to the robot centre: offset AND bearing, in mm
# and rad, in the robot frame (x forward, y left, theta CCW). A sensor that is
# not aimed radially outward from where it is bolted is still described
# correctly.
#
# MEASURED:
LIDAR_MOUNT_FRONT_X = 57.0
LIDAR_MOUNT_FRONT_Y = 0.0
LIDAR_MOUNT_FRONT_THETA = 0.0
LIDAR_MOUNT_LEFT_X = 34.0
LIDAR_MOUNT_LEFT_Y = 35.0
LIDAR_MOUNT_LEFT_THETA = PI_TWO
LIDAR_MOUNT_RIGHT_X = 34.0
LIDAR_MOUNT_RIGHT_Y = -35.0
LIDAR_MOUNT_RIGHT_THETA = -PI_TWO

# LIDAR OBSERVER
# Levenberg-Marquardt steps per solve. Two is enough while the prior is within
# a few mm: the model is close to linear over that range and the fusion gain
# re-solves at the next sample anyway.
LIDAR_OBSERVER_ITERATIONS = 2

# Marquardt damping, as a fraction of each diagonal. Guards the step against
# the model's nonlinearity; the regularisation that matters is below.
LIDAR_OBSERVER_LAMBDA = 0.05

# How far the dead-reckoned prior is expected to be out by the time a fix
# arrives, per axis. These weight the solve: three beams routinely leave a
# direction of pose space unobservable, and something has to decide how the
# correction is split. Heading is held tighter than position because it is the
# axis the gyro-backed fusion is already good at. See constants.h for the full
# argument.
LIDAR_OBSERVER_PRIOR_SIGMA_MM = 20.0
LIDAR_OBSERVER_PRIOR_SIGMA_RAD = 0.05

# Beam rejection. A residual larger than this is a beam looking at something
# the map does not contain, or at the wrong surface entirely.
LIDAR_OBSERVER_MAX_RESIDUAL_MM = 40.0

# cos of the incidence angle below which a range is discarded (~70 deg off
# square), where quantisation and weak return signal both bite.
LIDAR_OBSERVER_MIN_INCIDENCE_COS = 0.34

# Largest beam heading error a reading may imply before it is treated as a
# misassociation rather than an error to correct, rad. Diagnostic only -- see
# the note in LidarObserver._beamEquation().
LIDAR_OBSERVER_MAX_IMPLIED_PHI = 0.35

# Per-solve step limits. A single sample should never move the estimate more
# than this, whatever the least squares asks for.
LIDAR_OBSERVER_MAX_STEP_MM = 20.0
LIDAR_OBSERVER_MAX_STEP_RAD = 0.15

# Below this the solve has converged and the remaining iterations are skipped.
LIDAR_OBSERVER_STEP_TOL_MM = 0.05
LIDAR_OBSERVER_STEP_TOL_RAD = 0.0005

# Broad phase. Obstacles whose bounding circle falls outside this radius of the
# robot centre cannot be reached by any beam this solve.
LIDAR_OBSERVER_SEARCH_RADIUS_MM = 400.0

# Ceiling on how many obstacles survive the broad phase; the cast falls back to
# the full map if it is ever exceeded.
LIDAR_OBSERVER_MAX_CANDIDATES = 24

# OLED
OLED_REFRESH_MS = 17  # ~58.8 Hz

# SENSOR FUSION
SENSOR_FUSION_MAX_VELOCITY_OBSERVERS = 4
SENSOR_FUSION_MAX_POSE_OBSERVERS = 2
