#include <Arduino.h>
#include <Wire.h>
#include <array>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

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
#include "observers.h"
#include "kinematics.h"
#include "motor.h"
#include "oled.h"
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

LidarSensor front(LIDAR_FRONT_ADDRESS, TOF_3_GPO);
LidarSensor left(LIDAR_LEFT_ADDRESS, TOF_1_GPO);
LidarSensor right(LIDAR_RIGHT_ADDRESS, TOF_2_GPO);
LIDAR lidar(std::array<LidarSensor*, 3>{&front, &left, &right});
FrontLidarObserver fl_obsv(lidar);

// TASK 3.1
// const std::array<VelocitySource, 2> obs_v = {{
//     {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
//     {&imu_obsv, FusionWeights::OmegaVTrust}
// }};
// SensorFusion sf(obs_v);
// // KPHeading, KPLateral for the steering law. lateralError is in mm.
// // KPLateral is rad/s per mm off the line and thus should remain small. Damping of the
// // line-following loop is zeta = KPHeading / (2*sqrt(KPLateral*v))
// MotionPlanner planner(10, 0.06f);

// TASK 3.2
// const std::array<VelocitySource, 2> obs_v = {{
//     {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
//     {&imu_obsv, FusionWeights::OmegaVTrust}
// }};
// const std::array<PoseSource, 1> obs_p = {
//     {&fl_obsv, FusionWeights::XPTrust}
// };
// SensorFusion sf(obs_v, obs_p, 1);
// DistancePlanner planner(3, 0.06);

// TASK 3.3
// constexpr std::array obs_v = {
//     VelocitySource{&wheel_obsv, FusionWeights::VVTrust},
//     VelocitySource{&imu_obsv, FusionWeights::OmegaVTrust}
// };
// SensorFusion sf(obs_v);
// HeadingPlanner planner(5);

// TASK 3.4
// const std::array<VelocitySource, 2> obs_v = {{
//     {&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
//     {&imu_obsv, FusionWeights::OmegaVTrust}
// }};
// SensorFusion sf(obs_v);
// PSPlanner planner(10, 5);

// 4.1 | 4.2
const std::array<VelocitySource, 2> obs_v = {
    {{&wheel_obsv, ObserverVTrust{1.0f, 0.2f}},
     {&imu_obsv, FusionWeights::OmegaVTrust}}};

#include "maze_map.h"
LidarObserver lidar_obsv(lidar, MAZE_MAP);
const std::array<PoseSource, 1> obs_p = {{{&lidar_obsv, {0.2, 0.2, 0.1}}}};

// SensorFusion sf(obs_v, obs_p);
SensorFusion sf(obs_v);

// Pose fusedPose() {
//     return sf.estimate.pose();
// }

MotionPlanner planner(10, 0.06f, 200.0f);

float dt = 0;

// kd injects noise since loop speed means minimum α = dω/dt is 9 rad/s
MotionController mc(leftMotor, rightMotor, kinematics, 20.0f, 3.0f, 0.0f);

const std::array values = {
    // OLEDValue{"wov", []() { return wheel_obsv.estimate().v;}},
    // OLEDValue{"woo", []() { return wheel_obsv.estimate().omega; }},
    // OLEDValue{"ioo", []() { return imu_obsv.estimate().omega; }},
    OLEDValue{"x", []() { return sf.estimate.pose().x; }},
    OLEDValue{"y", []() { return sf.estimate.pose().y; }},
    OLEDValue{"th", []() { return sf.estimate.pose().theta; }},
    OLEDValue{"dt", []() { return dt; }},
    // OLEDValue{"pgr", []() { return planner.progress(sf.estimate.pose()); }},
    // OLEDValue{"sta", []() { return static_cast<float>(planner.s()); }},
    // OLEDValue{"idx", []() { return static_cast<float>(planner.idx()); }},
};

OLED oled(values);

unsigned long previous_time = 0;
unsigned long current_time = 0;

unsigned long previous_imu_time = 0;
unsigned long previous_print_time = 0;
unsigned long previous_oled_time = 0;
long previous_count_left = 0;
long previous_count_right = 0;

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

    Serial.print("Initialising Front Lidar Observer (P)...");
    if (!lidar.init()) {
        Serial.println("\b\b\b [VL6180X INIT FAILED]");
    } else {
        Serial.println("\b\b\b [OKAY]");
    }

    // TASK 4.1 | 4.2
    // lidar_obsv.setPrior(decltype(lidar_obsv)::PoseFunc::create<fusedPose>());
    sf.set(Pose{0, 0, 0});

    Serial.print("Initialising OLED...");
    if (!oled.init()) {
        Serial.println("\b\b\b [OLED INIT FAILED]");
    } else {
        oled.clear();
        Serial.println("\b\b\b [OKAY]");
    }

    Serial.print("Loading goal...");

    // TASK 3.1
    // NOTE THAT X-AXIS IS FORWARDS: Y-AXIS IS LEFT!!!
    // planner.appendSegment(Segment({0, 0}, {1000, 0}));
    // planner.appendSegment(Segment({1000, 0}, {1000, -50}, 1.0f / 25.0f, Segment::Direction::Right));
    // planner.appendSegment(Segment({1000, -50}, {0, 0}));

    // TASK 3.2
    // planner.setTarget(200.0f);

    // TASK 3.3
    // planner.setTarget(PI/2.0f);

    // TASK 3.4
    // planner.setStart({0, 0, PSPlanner::North});
    // planner.addInstructions("ffrfllfrlf");

    // TASK 4.1 | 4.2
    #include "maze_path.h"
    if (planner.s() != MotionPlanner::State::Run) {
        Serial.println("\b\b\b [maze_path.h APPENDED NO SEGMENTS]");
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

    // Serial.println(dt);

    i2cRepairer.update();

    // SensorFusion::update() steps its own velocity/pose observers
    sf.update(dt);
    Pose pose = sf.estimate.pose();
    Velocity current = sf.estimate.velocity();

    Velocity desired = planner.update(pose, dt);

    mc.update(desired, current, dt);

    // long new_count_left = leftMotor.count();
    // if (previous_count_left != new_count_left) {
    //     Serial.println(new_count_left);
    //     previous_count_left = new_count_left;
    // }

    // long new_count_right = rightMotor.count();
    // if (previous_count_right != new_count_right) {
    //     Serial.println(new_count_right);
    //     previous_count_right = new_count_right;
    // }

    if (current_time - previous_oled_time >= OLED_REFRESH_MS * 1000UL) {
        // Serial.print("x, y, t: ");
        // Serial.print(sf.estimate.pose().x);
        // Serial.print(", ");
        // Serial.print(sf.estimate.pose().y);
        // Serial.print(", ");
        // Serial.println(sf.estimate.pose().theta);
        // Serial.print("P: ");
        // Serial.println(planner.progress(pose));
        // Serial.print("S: ");
        // Serial.println(static_cast<int>(planner.s()));
        // Serial.print("V: ");
        // Serial.println(desired.v);
        oled.update();
        previous_oled_time = current_time;
    }
    previous_time = current_time;
}
