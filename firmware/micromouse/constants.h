// Micromouse Constants
//
// Zimmy Levi z5587840

#pragma once
#include <Arduino.h>

constexpr uint8_t MAZE_CORNER_CROP = 1;

// MATH
constexpr float PI_TWO  = PI / 2.0f;
constexpr float PI_FOUR = PI / 4.0f;

// Resolution of the atan/acos lookup tables in trig<> (see types.h).
constexpr size_t TRIG_LUT_SIZE = 256;

// WHEEL & KINEMATICS
// Effective rolling diameter (mm). test implies ~31.4 effective
constexpr float WHEEL_DIAMETER = 31.4f;
constexpr float WHEEL_RADIUS   = WHEEL_DIAMETER / 2.0f;
constexpr int ENC_CPR          = 700;

// Per-wheel encoder calibration. Hand-measured angularDisplacement() for
// exactly one wheel revolution (rad), 2026-07-17. Readings are scaled by
// TWOPI / measured so a full revolution reads exactly 2π.
constexpr float ENC_RAD_PER_REV_LEFT    = 6.04f;
constexpr float ENC_RAD_PER_REV_RIGHT   = 6.18f;
constexpr float ENC_SCALE_LEFT          = TWO_PI / ENC_RAD_PER_REV_LEFT;
constexpr float ENC_SCALE_RIGHT         = TWO_PI / ENC_RAD_PER_REV_RIGHT;
constexpr float AXLE_LEN                = 93.5;
constexpr uint8_t AXLE_DIST_FROM_CENTRE = 20;

constexpr uint8_t MAXIMUM_WHEEL_PWM            = 255;
constexpr float MAXIMUM_LATERAL_ACCELERATION   = 4;  // m/s²
constexpr float MAXIMUM_WHEEL_ANGULAR_VELOCITY = 25; // rad/s

// CONTROL LOOP
// Floor on a usable loop period. A guard against dividing by a dt of zero when
// loop() re-enters before micros() has moved, and nothing more -- it is not the
// period the loop runs at, and it is four orders of magnitude below it.
constexpr float MIN_LOOP_DT_S = 0.000005f; // 5 μs

// Rate loop() actually runs at, excluding the OLED frames. The IMU's sample
// rate is derived from this (see IMU_SAMPLE_RATE_DIVIDER), so it wants to be
// measured rather than hoped for -- the sketch prints observed samples per
// cycle for exactly that purpose, and a figure far from 1 means this is wrong.
//
// Being wrong costs granularity, not accuracy: summing the FIFO integrates
// exactly whatever it holds, whatever rate it was filled at.
constexpr unsigned long CONTROL_LOOP_NOMINAL_HZ = 300;

// MOTION
constexpr long PATH_SEGMENTS_MAX_LEN         = 256;
constexpr float STRAIGHT_TOLERANCE           = 0.001f;
constexpr float MAXIMUM_FORWARD_VELOCITY     = WHEEL_RADIUS * MAXIMUM_WHEEL_ANGULAR_VELOCITY;
constexpr float MAXIMUM_ANGULAR_ACCELERATION = 20.0f; // rad/s²
constexpr float STD_TOL                      = 1e-6f;
constexpr float STD_DIST_TOL                 = 2.0f;
// Position error at which PSPlanner stops translating and starts aligning
// with the target heading.  The larger deadband prevents lidar/odometry
// noise near a cell centre from delaying the turn handoff.
constexpr float PS_POSITION_TOL              = 8.0f;
// constexpr float STD_ANG_TOL                  = 0.02f;
constexpr float STD_ANG_TOL                  = 0.05f;

constexpr float SEGMENT_ADVANCE_THRESHOLD    = 0.995f;

// Fraction of the wheel speed limit an arc's feedforward is allowed to spend,
// leaving the rest for MotionPlanner's heading and lateral terms.
//
// On an arc the planner asks for omega = curvature * v, so the outer wheel runs
// at v (1 + curvature * AXLE_LEN / 2) / WHEEL_RADIUS. Left at cruise that is
// four or five times the wheel limit on a tight arc, and Kinematics::IK scales
// both wheels by one factor to fit -- which preserves the v : omega ratio, so
// the turn is still driven at its own radius, but the feedback terms are scaled
// down by the same factor and arrive with almost no authority. Capping v so the
// feedforward alone fits inside this fraction is what gives them room: the
// remaining 1 - margin is theirs, about 1.7 rad/s of correction on a 10 mm arc.
constexpr float TURN_ENVELOPE_MARGIN         = 0.8f;

// How far behind an arc's start a position still reads as "not started"
// (Segment::arcProgress), rather than as having come round past the end.
//
// A handover is a distance, not an angle: the straight feeding an arc lets go
// within a few mm of its start, and that slop subtends more angle the tighter
// the arc, so this is carried in mm and converted with the curvature. Kept
// small deliberately -- too large and a robot that has come round the back of
// the circle reads as "not started" and can never satisfy
// SEGMENT_ADVANCE_THRESHOLD, which strands it on that segment; too small and a
// slightly early handover reads as past the end and the arc is skipped. A cut
// corner and a robot that never leaves the corner are not equal costs.
constexpr float ARC_BEHIND_START_TOL_MM      = 5.0f;

// MAZE
// Cells per side. MAZE_SIZE_MAX is the template capacity every maze buffer is
// sized to, not the maze being run: the grid actually in use is chosen at boot
// and carried as a runtime value, so one binary handles every size in range.
//
// Cost grows as N^2. At the capacity, MazeMapper holds ~1.5 kB and its deepest
// breadth-first search -- the frontier pruning, which wants two distance fields
// and a queue at once -- borrows ~1.5 kB of stack. The searches run one at a
// time, so the borrow does not stack. That is affordable on the Nano R4's
// 32 kB, especially with task42.h's MotionPlanner instance gone.
constexpr uint8_t MAZE_SIZE_MIN = 2;
constexpr uint8_t MAZE_SIZE_MAX = 16;

// Physical size of one grid cell, mm. Full-size Micromouse uses 180mm,
// half-size 168mm. Shared by the path factory and the instruction runner.
constexpr float MAZE_CELL_SIZE = 180.0f;

// Panel and post geometry, mirroring WALL_T_MM / POST_T_MM in
// path-planning/maze_map.py. The post radius is the circumscribed circle of
// the square section, so a post is never under-covered.
constexpr float MAZE_WALL_THICKNESS = 12.0f;
constexpr float MAZE_POST_SIZE      = 12.0f;
constexpr float MAZE_POST_RADIUS    = MAZE_POST_SIZE * 0.7071068f;

// Longest instruction string ("frfllflr...") the InstructionRunner accepts,
// excluding the null terminator.
constexpr size_t MAZE_INSTRUCTION_MAX_LEN = 256;


// PID
constexpr float PID_SATURATION_ABSOLUTE = 1000.0f;

// I2C
constexpr uint32_t I2C_FREQ           = 400000;
constexpr uint8_t I2C_MAX_SLAVE_COUNT = 16;

// Bus supervisor / recovery clocking (see i2cRepairer.h). WIRE timeout must
// exceed the longest single transaction: one SSD1306 framebuffer chunk is
// 255 bytes ≈ 5.7ms at 400kHz, ≈ 23ms at 100kHz. A shorter timeout abandons
// transfers mid-flight and wedges the FSP driver.
constexpr unsigned int I2C_WIRE_TIMEOUT_US         = 50000;
constexpr unsigned long I2C_PROBE_INTERVAL_MS      = 250;
constexpr unsigned long I2C_RECOVERY_COOLDOWN_MS   = 1000;
constexpr uint8_t I2C_RECOVERY_FAILURE_THRESHOLD   = 3;
constexpr unsigned int I2C_RECOVERY_HALF_PERIOD_US = 5; // ~100kHz recovery clocking
constexpr uint8_t I2C_RECOVERY_MAX_PULSES          = 9;

// IMU
constexpr uint8_t IMU_ADDRESS       = 0x68;
constexpr int IMU_STARTUP_SETTLE_MS = 500;

// Rolling average behind the direct-read accessors (gyroX/Y, accelX/Y/Z).
// Heading does not go through them any more -- it comes off the FIFO, which
// needs no smoothing because it drops no samples to smooth over.
constexpr uint8_t IMU_ROLLING_AVG_SAMPLES = 8;

// Settling time after each PWR_MGMT_1 write, for the PLL to lock onto the gyro
// reference before anything is clocked off it.
constexpr uint8_t IMU_CLOCK_SETTLE_MS = 10;

// The rate SMPLRT_DIV divides. 1 kHz whenever the DLPF is doing anything at
// all; only bypassing it altogether raises this to 8 kHz. IMU::init derives the
// period it actually integrates with from the DLPF argument it is handed rather
// than from here, so a bypassed filter costs FIFO depth but not accuracy.
constexpr unsigned long IMU_GYRO_OUTPUT_RATE_HZ = 1000;

// SMPLRT_DIV, derived from the loop rate rather than picked.
//
// The target is one sample per control cycle, at minimum. Nothing about the
// integral needs that -- summing the FIFO is exact at any rate, which is the
// whole point of reading it rather than the data registers -- but a sample rate
// below the loop rate leaves some cycles with an empty batch and theta standing
// still until the next one, so the correction lands quantised to the sample
// period instead of to the loop.
//
// Integer floor before the minus one, so what comes out is at or above the loop
// rate rather than nearest to it. Clamped at zero because no divider can ask
// for more than the output rate: a loop faster than 1 kHz simply gets 1 kHz,
// and the batches it drains start coming back empty half the time.
constexpr uint8_t imuSampleRateDivider(unsigned long loop_hz) {
    return static_cast<uint8_t>(
        ((loop_hz == 0 || loop_hz > IMU_GYRO_OUTPUT_RATE_HZ)
                ? 1UL
                : IMU_GYRO_OUTPUT_RATE_HZ / loop_hz) -
        1UL
    );
}

constexpr uint8_t IMU_SAMPLE_RATE_DIVIDER = imuSampleRateDivider(CONTROL_LOOP_NOMINAL_HZ);
constexpr unsigned long IMU_SAMPLE_RATE_HZ =
    IMU_GYRO_OUTPUT_RATE_HZ / (1UL + IMU_SAMPLE_RATE_DIVIDER);

// Depth of the sensor's own buffer. Heading survives an OLED frame only because
// the sensor keeps filling this while nothing is reading it, so it is the
// budget the sample rate has to fit inside.
constexpr uint16_t IMU_FIFO_CAPACITY_BYTES = 1024;

// Longest stretch loop() is expected to go without draining. The framebuffer
// push is the one that matters, and OLED_REFRESH_MS is only how often it
// happens -- the transfer itself is ~23 ms of a 1 KiB write at 400 kHz. 40 ms
// leaves room for it to be worse than that.
constexpr unsigned long IMU_MAX_DRAIN_GAP_MS = 40;

// Z gyro only, so two bytes a sample. Asserted at a quarter of the buffer
// rather than at the brim: an overflow does not degrade the estimate, it drops
// a batch of heading outright, so the margin is the feature.
static_assert(
    IMU_SAMPLE_RATE_HZ * IMU_MAX_DRAIN_GAP_MS * 2UL / 1000UL < IMU_FIFO_CAPACITY_BYTES / 4UL,
    "IMU sample rate leaves under 4x FIFO headroom across the worst expected "
    "drain gap: lower CONTROL_LOOP_NOMINAL_HZ, or drain more often than "
    "IMU_MAX_DRAIN_GAP_MS."
);

// Bytes per FIFO read. Kept under the Wire library's own buffer so a chunk is
// never silently truncated, and even so a partial sample is never returned.
// Even, so a chunk boundary is always a sample boundary.
constexpr uint8_t IMU_FIFO_CHUNK_BYTES = 32;
static_assert(IMU_FIFO_CHUNK_BYTES % 2 == 0, "A FIFO chunk must be whole samples.");

// Zero-rate calibration at startup: how long the window is, and how often it
// is drained inside that window. The drain interval only has to stay well
// inside the FIFO's depth, so the window length is free to be whatever
// averages the noise down.
constexpr unsigned long IMU_CALIBRATION_MS       = 2500;
constexpr unsigned long IMU_CALIBRATION_DRAIN_MS = 200;

// LIDAR
constexpr uint8_t LIDAR_FRONT_ADDRESS        = 0x30;
constexpr uint8_t LIDAR_LEFT_ADDRESS         = 0x31;
constexpr uint8_t LIDAR_RIGHT_ADDRESS        = 0x32;
constexpr uint8_t LIDAR_TIMEOUT_MS           = 50;
constexpr uint8_t LIDAR_CONTINUOUS_PERIOD_MS = 10; // minimum: 10 ms
constexpr uint8_t LIDAR_BRINGUP_SETTLE_MS = 10;

constexpr uint16_t WALL_THRESHOLD_MM = 140;

// Sensor mount poses relative to the robot centre: offset AND bearing, in mm
// and rad, in the robot frame (x forward, y left, theta CCW). A sensor that is
// not aimed radially outward from where it is bolted is still described
// correctly.

// MEASURED:
constexpr float LIDAR_MOUNT_FRONT_X     = 57.0f;
constexpr float LIDAR_MOUNT_FRONT_Y     = 0.0f;
constexpr float LIDAR_MOUNT_FRONT_THETA = 0.0f;
constexpr float LIDAR_MOUNT_LEFT_X      = 34.0f;
constexpr float LIDAR_MOUNT_LEFT_Y      = 35.0f;
constexpr float LIDAR_MOUNT_LEFT_THETA  = PI_TWO;
constexpr float LIDAR_MOUNT_RIGHT_X     = 34.0f;
constexpr float LIDAR_MOUNT_RIGHT_Y     = -35.0f;
constexpr float LIDAR_MOUNT_RIGHT_THETA = -PI_TWO;

// LIDAR OBSERVER
// Levenberg-Marquardt steps per solve. Two is enough while the prior is within
// a few mm: the model is close to linear over that range and the fusion gain
// re-solves at the next sample anyway.
constexpr uint8_t LIDAR_OBSERVER_ITERATIONS = 2;

// Marquardt damping, as a fraction of each diagonal. Guards the step against
// the model's nonlinearity; the regularisation that matters is below.
constexpr float LIDAR_OBSERVER_LAMBDA = 0.05f;

// How far the dead-reckoned prior is expected to be out by the time a fix
// arrives, per axis. These weight the solve, and they are not a nicety: three
// beams routinely leave a direction of pose space unobservable -- two parallel
// walls seen by the symmetric side sensors constrain only one combination of y
// and theta, not both -- and something has to decide how the correction is
// split. Without a weighting that decision falls to the choice of units, which
// puts 30 mm of translation and one radian of rotation on the same footing and
// happily explains 6 mm of lateral offset as four degrees of heading error.
//
// Heading is held tighter than position because it is the axis the gyro-backed
// fusion is already good at, so a residual is attributed to position unless
// the geometry genuinely says otherwise. Tighter than about 0.02 rad and the
// observer stops correcting heading even where it can see it; looser than
// about 0.1 rad and a lateral offset in a corridor starts coming back as
// rotation. Both are the same knob: 20 mm against a ~30 mm sensor lever arm is
// roughly 0.7 rad of equivalent slack, so 0.05 rad still prefers translation
// by an order of magnitude.
constexpr float LIDAR_OBSERVER_PRIOR_SIGMA_MM  = 20.0f;
constexpr float LIDAR_OBSERVER_PRIOR_SIGMA_RAD = 0.05f;

// Beam rejection. A residual larger than this is a beam looking at something
// the map does not contain (a hand, a chair leg, the robot's own start box) or
// at the wrong surface entirely, and folding it in would drag the pose off.
constexpr float LIDAR_OBSERVER_MAX_RESIDUAL_MM = 10.0f;

// cos of the incidence angle below which a range is discarded. At 0.34 (~70
// degrees off square) a 1 mm quantisation step already moves the implied
// position by 3 mm, and the VL6180X's return signal is weak enough there that
// the reading itself is suspect.
constexpr float LIDAR_OBSERVER_MIN_INCIDENCE_COS = 0.34f;

// Largest beam heading error a reading may imply before it is treated as a
// misassociation rather than an error to correct, rad.
constexpr float LIDAR_OBSERVER_MAX_IMPLIED_PHI = 0.35f;

// Per-solve step limits. A single sample should never move the estimate more
// than this, whatever the least squares asks for.
constexpr float LIDAR_OBSERVER_MAX_STEP_MM  = 20.0f;
constexpr float LIDAR_OBSERVER_MAX_STEP_RAD = 0.15f;

// Below this the solve has converged and the remaining iterations are skipped.
constexpr float LIDAR_OBSERVER_STEP_TOL_MM  = 0.05f;
constexpr float LIDAR_OBSERVER_STEP_TOL_RAD = 0.0005f;

// Broad phase. Obstacles whose bounding circle falls outside this radius of
// the robot centre cannot be reached by any beam this solve, so they are
// culled once rather than tested on every cast. Covers the sensor mount
// offset, the sensor's own ceiling and the largest step the solve can take.
constexpr float LIDAR_OBSERVER_SEARCH_RADIUS_MM = 400.0f;

// Ceiling on how many obstacles survive the broad phase. The cast falls back
// to the full map if it is ever exceeded, which costs time but not accuracy.
//
// Measured, not guessed: a 400 mm search around a cell centre of maze_map.h
// reaches at most 40 obstacles, and reaches more than 24 at 50 of its 81 cell
// centres. At 24 the broad phase was therefore being abandoned most of the
// time and every beam re-cast the whole map -- exactly the cost it exists to
// avoid. MazeWallMap over the same maze, fully explored, peaks at 44. 48
// clears both with headroom, and costs 96 bytes of RAM in the observer.
constexpr size_t LIDAR_OBSERVER_MAX_CANDIDATES = 48;

// OLED
constexpr uint8_t OLED_WIDTH       = 128;
constexpr uint8_t OLED_HEIGHT      = 64;
constexpr uint8_t OLED_ADDRESS     = 0x3C;
constexpr int8_t OLED_NO_RESET_PIN = -1;
constexpr uint8_t OLED_TEXT_SIZE   = 1;
constexpr uint8_t OLED_TEXT_HEIGHT = 8;
constexpr uint8_t OLED_CHAR_WIDTH  = 6;

// ~24 Hz. Deliberately slower than the panel or the bus can go: OLEDScreen
// redraws the whole map every frame -- 85 lines and 57 circles for the exported
// maze -- and that work lands inside the control loop, between reading the pose
// and commanding the motors. Nobody can read a pose off a display faster than
// this, so the rate is set by what is legible rather than by what is possible,
// and the loop keeps the difference.
constexpr uint8_t OLED_REFRESH_MS = 100;

// Map pane. Square and as tall as the panel, so a maze drawn into it is never
// letterboxed on the axis that matters. Widening it past the panel height would
// buy nothing for a square maze -- OLEDView takes the tighter of the two fits,
// so the scale would stay pinned by the height and the extra columns would sit
// empty.
constexpr uint8_t OLED_MAP_PANE_X = 0;
constexpr uint8_t OLED_MAP_PANE_W = 64;

// Values pane. Starts two pixels clear of the map pane so a wall line drawn on
// the pane's right edge does not touch the first character column. That leaves
// 62 px, which is 10 characters at OLED_CHAR_WIDTH -- enough for "X  -1350",
// the widest readout a maze this size can produce.
constexpr uint8_t OLED_TEXT_PANE_X = 66;

// Column the values pane prints numbers in, as a character offset from
// OLED_TEXT_PANE_X. Two, so a one-character label and a space clear it and
// every row's digits line up whatever the label.
constexpr uint8_t OLED_VALUE_COLUMN = 2;

// SENSOR FUSION
constexpr size_t SENSOR_FUSION_MAX_VELOCITY_OBSERVERS = 4;
constexpr size_t SENSOR_FUSION_MAX_POSE_OBSERVERS     = 2;
