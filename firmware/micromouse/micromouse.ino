#include <Arduino.h>
#include <Wire.h>
#include <array>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

#include "constants.h"
#include "planners.h"
#include "control.h"
#include "pins.h"
#include "i2cRepairer.h"
#include "imu.h"
#include "lidar.h"
#include "mazeMapper.h"
#include "mazeRunner.h"
#include "mazeWallMap.h"
#include "observers.h"
#include "kinematics.h"
#include "motor.h"
#include "oled.h"
#include "oledDisplay.h"
#include "oledMap.h"
#include "oledPath.h"
#include "sensorFusion.h"
#include "types.h"

Motor<0> leftMotor(
    WHEEL_RADIUS,
    MOT_1_DIR,
    MOT_1_PWM,
    MOT_1_ENC_A,
    MOT_1_ENC_B,
    ENC_CPR,
    false,
    ENC_SCALE_LEFT
);

Motor<1> rightMotor(
    WHEEL_RADIUS,
    MOT_2_DIR,
    MOT_2_PWM,
    MOT_2_ENC_A,
    MOT_2_ENC_B,
    ENC_CPR,
    true,
    ENC_SCALE_RIGHT
);

Kinematics kinematics(WHEEL_RADIUS, AXLE_LEN);

// Add fixer for I2C channels which can break under high frequency interrupts and OLED buffer delays
I2CRepairer i2cRepairer(I2C_SDA, I2C_SCL, LIDAR_FRONT_ADDRESS);

WheelObserver wheel_obsv(leftMotor, rightMotor, kinematics);

IMU imu;
ImuObserver imu_obsv(imu);

LidarSensor frontLS(LIDAR_FRONT_ADDRESS, TOF_3_GPO);
LidarSensor leftLS(LIDAR_LEFT_ADDRESS, TOF_1_GPO);
LidarSensor rightLS(LIDAR_RIGHT_ADDRESS, TOF_2_GPO);
LIDAR lidar(std::array<LidarSensor*, 3>{&frontLS, &leftLS, &rightLS});
FrontLidarObserver fl_obsv(lidar);

// TASK 4.1 | 4.2
// const std::array<VelocitySource, 2> obs_v = {{
//     {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
//     {&imu_obsv, FusionWeights::OmegaVTrust}
// }};


// TASK 4.1 | 4.2
// The observer localises against the map export_map.py fitted by CV from a
// photograph of the maze. 4.3 has no photograph -- finding the maze is the
// exercise -- so the wiring below points the same observer at the walls the
// mapper has discovered instead. Either works: LidarObserver is templated on
// the map type, and MazeWallMap offers Map's cast()/candidates().
//
// #include "maze_map.h"
// LidarObserver lidar_obsv(lidar, MAZE_MAP);

// TASK 4.3
const std::array<VelocitySource, 2> obs_v = {{
    {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
    {&imu_obsv, FusionWeights::OmegaVTrust}
}};
PSPlanner psp(8.0f, 8.0f);
mazeMapper::Cell startCell = {0, 0};
Direction startHeading = North;
mazeMapper::Cell goalCell = {1, 0};

MazeRunner<MAZE_SIZE> runner(
    lidar,
    psp,
    startCell,
    startHeading,
    goalCell
);
MazeWallMap<MAZE_SIZE> wallMap(runner.map());
LidarObserver<MazeWallMap<MAZE_SIZE>> lidar_obsv(lidar, wallMap);

const std::array<PoseSource, 1> obs_p = {{
    {&lidar_obsv, FusionWeights::XYPTrust}
}};
SensorFusion sf(obs_v, obs_p, 0.1);
Pose fusedPose() { return sf.estimate.pose(); }

// TASK 4.1 | 4.2
// Drives the pre-computed maze_path.h route as blended arcs. 4.3 discovers its
// own route and drives it cell by cell through PSPlanner instead, so this and
// its std::array<Segment, 256> -- about 10 kB of the sketch's RAM -- come out.
// scripts/build_maze.sh still generates maze_path.h either way.
//
// MotionPlanner planner(10, 0.06f, 200.0f);

// Loop period, seconds. Defined here rather than beside the controller because
// the `values` readout below captures it.
float dt = 0;

OLEDDisplay display;

// TASK 4.3
// Delegates, so neither display depends on the runner's type.
float exploreProgress() {
    return runner.exploreProgress();
}

float raceProgress() {
    return runner.raceProgress();
}

OLEDMap<MAZE_SIZE> oledMap(display, runner.map(), etl::delegate<float()>::create<exploreProgress>());

// TASK 4.1 | 4.2
// The same display against the exported map, which is what OLEDPath was
// written for. Needs maze_map.h included above.
//
// OLEDPath<Map<MAZE_OBSTACLE_COUNT>> oledPath(
//     display,
//     MAZE_MAP,
//     etl::delegate<Pose()>::create<fusedPose>(),
//     etl::delegate<float()>::create<raceProgress>()
// );

// TASK 4.3
// OLEDPath<MazeWallMap<MAZE_SIZE>> oledPath(
OLEDPath<MazeWallMap<MAZE_SIZE>> oledPath(
    display,
    wallMap,
    etl::delegate<Pose()>::create<fusedPose>(),
    etl::delegate<float()>::create<raceProgress>()
);

// TASK 4.1 | 4.2
// The scalar readout as it was, reporting MotionPlanner. Uncommenting it also
// needs the MotionPlanner declaration above; the class itself is unchanged
// apart from its name (OLED -> OLEDValues) and taking the shared OLEDDisplay,
// since OLEDMap and OLEDPath draw to the same panel and only one thing can
// own it.
//
// const std::array values = {
//     OLEDValue{"x", []() { return sf.estimate.pose().x; }},
//     OLEDValue{"y", []() { return sf.estimate.pose().y; }},
//     OLEDValue{"th", []() { return sf.estimate.pose().theta; }},
//     OLEDValue{"dt", []() { return dt; }},
//     OLEDValue{"pgr", []() { return planner.progress(sf.estimate.pose()); }},
//     OLEDValue{"sta", []() { return static_cast<float>(planner.s()); }},
//     OLEDValue{"idx", []() { return static_cast<float>(planner.idx()); }},
// };

// TASK 4.3
// Kept constructed and available for bring-up, but not driven in loop():
// OLEDDisplay::due() is consuming, so only one renderer may draw per tick.
const std::array values = {
    OLEDValue{"x", []() { return sf.estimate.pose().x; }},
    OLEDValue{"y", []() { return sf.estimate.pose().y; }},
    OLEDValue{"th", []() { return sf.estimate.pose().theta; }},
    OLEDValue{"dt", []() { return dt; }},
    OLEDValue{"pgr", []() { return raceProgress(); }},
    OLEDValue{"sta",
              []() {
                return static_cast<float>(static_cast<uint8_t>(runner.state()));
              }},
    OLEDValue{"bms", []() { return static_cast<float>(lidar_obsv.beams()); }},
};

// kd injects noise since loop speed means minimum α = dω/dt is 9 rad/s
MotionController mc(leftMotor, rightMotor, kinematics, 10.0f, 3.0f, 0.0f);

OLEDValues oled(display, values);

unsigned long previous_time = 0;
unsigned long current_time = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Beginning setup:");

    Serial.print("Initialising I2C...");
    i2cRepairer.begin();
    Serial.println("\b\b\b [OKAY]");

    Serial.print("Initialising Motors...");
    leftMotor.init();
    rightMotor.init();
    Serial.println("\b\b\b [OKAY]");

    Serial.print("Initialising IMU Observer (V)...");
    if (!imu.init(IMU::GyroScale::DPS_1000, IMU::AccelScale::G_4, IMU::LowPassFrequency::HZ_44)) {
        Serial.println("\b\b\b [MPU6050 INIT FAILED]");
    } else {
        imu_obsv.init();
        if (!imu_obsv.ready()) {
            Serial.println("\b\b\b [IMU OBSERVER INIT FAILED]");
        } else {
            Serial.println("\b\b\b [OKAY]");
        }
    }

    Serial.print("Initialising Lidar Observer (P)...");
    if (!lidar.init()) {
        Serial.println("\b\b\b [VL6180X INIT FAILED]");
    } else {
        lidar_obsv.setPrior(decltype(lidar_obsv)::PoseFunc::create<fusedPose>());
        sf.set(Pose{0, 0, 0});
        Serial.println("\b\b\b [OKAY]");
    }

    Serial.print("Initialising OLED...");
    if (!display.init()) {
        Serial.println("\b\b\b [OLED INIT FAILED]");
    } else {
        display.clear();
        Serial.println("\b\b\b [OKAY]");
    }

    Serial.print("Loading goal...");

    // NOTE THAT X-AXIS IS FORWARDS: Y-AXIS IS LEFT!!!

    // TASK 4.1 | 4.2

    // TASK 4.1 | 4.2
    // Loads the pre-computed route. maze_path.h is a bare list of
    // planner.appendSegment(...) calls, so it is included here, inside a
    // function body, rather than at file scope.
    //
    // #include "maze_path.h"
    // if (planner.s() != MotionPlanner::State::Run) {
    //     Serial.println("\b\b\b [maze_path.h APPENDED NO SEGMENTS]");
    // } else {
    //     Serial.println("\b\b\b [OKAY]");
    // }

    // TASK 4.3
    // Start in the corner facing North, goal at the centre. The complete maze
    // configuration is supplied when MazeRunner is constructed above.
    if (!runner.begin()) {
        Serial.println("\b\b\b [MAZE RUNNER REJECTED START OR GOAL]");
    } else {
        Serial.print("\b\b\b [OKAY, ");
        Serial.print(runner.cropped());
        Serial.print(" cropped, ");
        Serial.print(runner.reachable());
        Serial.println(" reachable]");
    }

    // TASK 4.3
    // After runner.begin(), which seeds the perimeter -- the extent the map
    // pane is fitted to. Walls found later fall inside it, so one fit holds
    // for the whole run.
    Serial.print("Fitting map to display...");
    if (!oledPath.init()) {
        Serial.println("\b\b\b [MAP DID NOT FIT]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }

    previous_time = micros();
    Serial.println("Setup complete!");
}

void loop() {
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0f;

    if (dt <= MIN_LOOP_DT_S) return;

    i2cRepairer.update();

    // SensorFusion::update() steps its own velocity/pose observers
    sf.update(dt);
    Pose pose = sf.estimate.pose();
    Velocity current = sf.estimate.velocity();

    // TASK 4.1 | 4.2
    // Velocity desired = planner.update(pose, dt);

    // TASK 4.3
    // Explore, plan, race. Non-blocking, and it commands zero once done, so
    // nothing below has to special-case a stopped robot.
    Velocity desired = runner.update(pose, dt);

    mc.update(desired, current, dt);

    // TASK 4.1 | 4.2
    // The scalar readout. The OLED_REFRESH_MS gate that used to wrap this --
    // and a previous_oled_time to go with it -- is gone from loop(): it
    // duplicated the one inside the renderer, and both now live in
    // OLEDDisplay::due().
    //
    // oled.update();

    // TASK 4.3
    // One renderer per tick: OLEDDisplay::due() is consuming, so drawing two
    // would starve whichever asked second.
    if (runner.racing()) {
        oledPath.setRoute(runner.route());
        oledPath.update();
    } else {
        oledMap.update();
    }

    previous_time = current_time;
}
