// Micromouse Constants
//
// Zimmy Levi z5587840

#pragma once
#include <Arduino.h>

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
constexpr float AXLE_LEN                = 92.5;
constexpr uint8_t AXLE_DIST_FROM_CENTRE = 20;

constexpr uint8_t MAXIMUM_WHEEL_PWM            = 255;
constexpr float MAXIMUM_LATERAL_ACCELERATION   = 4;  // m/s²
constexpr float MAXIMUM_WHEEL_ANGULAR_VELOCITY = 25; // rad/s

// CONTROL LOOP
constexpr float MIN_LOOP_DT_S = 0.000005f; // 5 μs

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
constexpr float PS_POSITION_TOL              = 5.0f;
constexpr float STD_ANG_TOL                  = 0.05f;
constexpr float SEGMENT_ADVANCE_THRESHOLD    = 0.995f;

// MAZE
// Cells per side, and so the size the mazeMapper<> alias instantiates
// MazeMapper<> at, the same way TRIG_LUT_SIZE sizes trig. The class is
// templated on it, so a test or a second instance can pick another size
// without touching the header.
//
// Cost grows as N^2: at 9 the mapper holds ~580 bytes and its shortest-path
// search borrows ~250 bytes of stack, at 16 that is ~1.8 kB and ~770 bytes.
//
// Nine, not ten, because maze_map.h describes a 10x10 post lattice, and ten
// post lines bound nine cells. Cell centres in it run -180 mm to 1260 mm on
// both axes.
constexpr uint8_t MAZE_SIZE = 2;

// The maze is not a full rectangle: every corner is chamfered, so the corner
// cell and its two orthogonal neighbours are not there. This is the Manhattan
// radius of that cut measured from the corner cell, so 0 would remove the
// corner alone and 1 removes the three cells the physical maze is missing --
// twelve in all.
//
// The sketch seals each cropped cell on all four sides as a prior before
// exploration starts. MazeMapper has no concept of a cell that does not
// exist, and does not need one: a cell walled on every side is one its search
// can neither enter nor plan through.
constexpr int8_t MAZE_CORNER_CROP = -1;

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
constexpr uint8_t IMU_ADDRESS                  = 0x68;
constexpr int IMU_STARTUP_SETTLE_MS            = 500;
constexpr int IMU_STARTUP_READING_COUNT        = 500;
constexpr uint8_t IMU_STARTUP_READING_DELAY_MS = 5;
constexpr uint8_t IMU_ROLLING_AVG_SAMPLES      = 8;

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
constexpr float LIDAR_OBSERVER_MAX_RESIDUAL_MM = 40.0f;

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
constexpr uint8_t OLED_WIDTH                  = 128;
constexpr uint8_t OLED_HEIGHT                 = 64;
constexpr uint8_t OLED_MAX_VALUES             = 8;
constexpr uint8_t OLED_ADDRESS                = 0x3C;
constexpr int8_t OLED_NO_RESET_PIN            = -1;
constexpr uint8_t OLED_REFRESH_MS             = 17; // ~58.8 Hz
constexpr uint8_t OLED_TEXT_SIZE              = 1;
constexpr uint8_t OLED_TEXT_HEIGHT            = 8;
constexpr uint8_t OLED_CHAR_WIDTH             = 6;
constexpr uint8_t OLED_ONE_COLUMN_LABEL_CHARS = 7;
constexpr uint8_t OLED_TWO_COLUMN_LABEL_CHARS = 3;
constexpr uint8_t OLED_ONE_COLUMN_DECIMALS    = 2;
constexpr uint8_t OLED_TWO_COLUMN_DECIMALS    = 1;

// Map pane, shared by OLEDMap and OLEDPath. Square and as tall as the panel,
// so a maze drawn into it is never letterboxed on the axis that matters.
// OLEDMap divides the width by N for its cell pitch, so a square pane is also
// what keeps a cell square.
constexpr uint8_t OLED_MAP_PANE_X = 0;
constexpr uint8_t OLED_MAP_PANE_W = 64;

// Text pane. Starts two pixels clear of the map pane so a wall line drawn on
// the pane's right edge does not touch the first character column. That leaves
// 62 px, which is 10 characters at OLED_CHAR_WIDTH.
constexpr uint8_t OLED_TEXT_PANE_X = 66;

// Progress bar, drawn along the bottom of the text pane. Seven pixels is one
// pixel of border either side of a five-pixel fill, which still reads as a
// bar at this pixel density; the margin keeps it off the panel edge.
constexpr uint8_t OLED_BAR_H      = 7;
constexpr uint8_t OLED_BAR_MARGIN = 2;

// SENSOR FUSION
constexpr size_t SENSOR_FUSION_MAX_VELOCITY_OBSERVERS = 4;
constexpr size_t SENSOR_FUSION_MAX_POSE_OBSERVERS     = 2;
