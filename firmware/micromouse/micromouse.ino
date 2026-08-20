#include <Arduino.h>
#include <Wire.h>
#include <array>

#include "constants.h"
#include "control.h"
#include "pins.h"
#include "i2cRepairer.h"
#include "imu.h"
#include "kinematics.h"
#include "lidar.h"
#include "motor.h"
#include "observers.h"
#include "oledDisplay.h"
#include "oledSplash.h"
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

// The velocity side of the fusion is the same either way. The pose side is not
// -- it is the map the observer localises against that differs -- so obs_p and
// the SensorFusion built from both live in the run header.
//
// The wheels alone. The gyro used to be blended in here as an omega, at four
// times the wheels' weight in heading, which is what made theta depend on the
// control loop's dt: ModelObserver integrates fused omega against whatever the
// loop period happened to be, and an OLED frame makes that period jump by an
// order of magnitude with no gyro sample taken across the gap. It is a pose
// source now -- see ImuObserver -- and integrates on the sensor's own clock.
//
// So current.omega, which is what MotionController closes its per-wheel PIDs
// on, is now wheel-derived end to end. That is what those PIDs were written
// for; it does also mean the loop no longer sees body rotation directly, so
// wheel slip is rejected by the pose estimate rather than by the controller.
const std::array<VelocitySource, 1> obs_v = {{
    {&wheel_obsv, FusionWeights::DefaultVTrust}
}};

// Loop period, seconds. At file scope rather than local to loop() so the run
// header or a trace can read the rate the control loop is actually running at.
float dt = 0;

OLEDDisplay display;

// The run, included here rather than with the headers above because it builds
// on lidar, obs_v, dt and display.
#include "unseenMaze.h"

// kd injects noise since loop speed means minimum α = dω/dt is 9 rad/s
MotionController mc(leftMotor, rightMotor, kinematics, 10.0f, 3.0f, 0.0f);

unsigned long previous_time = 0;
unsigned long current_time = 0;

void setup() {
    Serial.begin(115200); // DIAGNOSTIC: was 9600
    delay(1000);
    Serial.println("Beginning setup:");

    Serial.print("Initialising I2C...");
    i2cRepairer.begin();
    Serial.println("\b\b\b [OKAY]");

    // Straight after the bus and before everything slow, which is the whole
    // point: display.init() needs nothing but Wire, and the splash is only
    // on screen for as long as the bring-up *below* it takes. Down at the end
    // of setup(), where this used to sit, the only things left to outlast were
    // two Serial prints and runBegin() -- so the logo was overwritten by the
    // first loop() frame inside ~10 ms and all that showed was the clear.
    //
    // What makes it readable now is imu_obsv.init(), which is not a settle
    // delay but a 3 s measurement: IMU_STARTUP_SETTLE_MS then a
    // IMU_CALIBRATION_MS window averaging the gyro's zero-rate output. With the
    // lidar's ~90 ms on top, the logo holds for about 3.2 s, so it needs no
    // delay() of its own -- it is showing during time the robot was already
    // going to spend standing still.
    Serial.print("Initialising OLED...");
    if (!display.init()) {
        Serial.println("\b\b\b [OLED INIT FAILED]");
    } else {
        drawSplash(display);
        Serial.println("\b\b\b [OKAY]");
    }

    Serial.print("Initialising Motors...");
    leftMotor.init();
    rightMotor.init();
    Serial.println("\b\b\b [OKAY]");

    Serial.print("Initialising IMU Observer (P)...");
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
        Serial.println("\b\b\b [OKAY]");
    }

    // Seeds every observer that holds an absolute pose, ImuObserver's heading
    // included, so it has to run whatever the lidar did. It used to sit inside
    // the branch above, which was harmless only while nothing on the pose side
    // integrated anything.
    sf.set(Pose{0, 0, 0});

    Serial.print("Loading goal...");

    // NOTE THAT X-AXIS IS FORWARDS: Y-AXIS IS LEFT!!!

    runBegin();

    previous_time = micros();
    Serial.println("Setup complete!");
}

void loop() {
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0f;

    if (dt <= MIN_LOOP_DT_S) return;

    i2cRepairer.update();

    sf.update(dt);
    Pose pose = sf.estimate.pose();
    Velocity current = sf.estimate.velocity();
    Velocity desired = runUpdate(pose, dt);
    mc.update(desired, current, dt);

    runRender();

    // DIAGNOSTIC: gyro read failures and bus recoveries, rate limited so the
    // report cannot itself stall the loop it is measuring.
    {
        static unsigned long reported = 0;
        static unsigned long last_report_ms = 0;
        const unsigned long failures = imu.failures();
        if (failures != reported && millis() - last_report_ms >= 200) {
            Serial.print(F("imu dropped "));
            Serial.print(failures - reported);
            Serial.print(F(" reads (total "));
            Serial.print(failures);
            Serial.print(F("), fifo losses "));
            Serial.print(imu.fifoLosses());
            Serial.print(F(", i2c recoveries "));
            Serial.println(i2cRepairer.recoveries());
            reported       = failures;
            last_report_ms = millis();
        }
    }

    // DIAGNOSTIC: FIFO samples per control cycle, which is the check on
    // CONTROL_LOOP_NOMINAL_HZ. Capped at a few reports rather than left
    // running: the figure is a cumulative mean and settles within the first
    // couple, and a periodic Serial write is a few ms of exactly the loop
    // stall this whole path exists to stop mattering. Delete freely.
    {
        static unsigned long last_rate_report_ms = 0;
        static uint8_t rate_reports = 0;
        if (rate_reports < 5 && millis() - last_rate_report_ms >= 2000) {
            Serial.print(F("imu "));
            Serial.print(imu_obsv.samplesPerUpdate(), 2);
            Serial.print(F(" samples/cycle at "));
            Serial.print(IMU_SAMPLE_RATE_HZ);
            Serial.print(F(" Hz (CONTROL_LOOP_NOMINAL_HZ "));
            Serial.print(CONTROL_LOOP_NOMINAL_HZ);
            Serial.println(F(")"));
            last_rate_report_ms = millis();
            ++rate_reports;
        }
    }

    previous_time = current_time;
}
