#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "pins.h"
#include "motor.h"
#include "i2cRepairer.h"
#include "imu.h"
#include "lidar.h"
#include "mazeMapper.h"

Motor<0> leftMotor(
    WHEEL_RADIUS_MM,
    MOT_1_DIR,
    MOT_1_PWM,
    MOT_1_ENC_A,
    MOT_1_ENC_B,
    ENC_CPR,
    false,
    ENC_SCALE_LEFT
);

Motor<1> rightMotor(
    WHEEL_RADIUS_MM,
    MOT_2_DIR,
    MOT_2_PWM,
    MOT_2_ENC_A,
    MOT_2_ENC_B,
    ENC_CPR,
    true,
    ENC_SCALE_RIGHT
);

I2CRepairer i2cRepairer(I2C_SDA, I2C_SCL);
IMU imu;

// Pin mapping from your latest pins.h comments:
// TOF_3_GPO = centre/front, TOF_1_GPO = left, TOF_2_GPO = right.
LidarSensor frontLidar(LIDAR_FRONT_ADDRESS, TOF_3_GPO);
LidarSensor leftLidar(LIDAR_LEFT_ADDRESS, TOF_1_GPO);
LidarSensor rightLidar(LIDAR_RIGHT_ADDRESS, TOF_2_GPO);
LIDAR lidar(&frontLidar, &leftLidar, &rightLidar);

MazeMapper mapper;

float gyroOffset = 0.0f;
float yaw = 0.0f;
unsigned long lastYawTime = 0;
bool taskStarted = false;

void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
}

float wrapAngle(float a) {
    while (a > MM_PI) a -= MM_TWO_PI;
    while (a < -MM_PI) a += MM_TWO_PI;
    return a;
}

void updateYaw() {
    unsigned long now = micros();
    if (lastYawTime == 0) {
        lastYawTime = now;
        return;
    }
    float dt = (now - lastYawTime) / 1000000.0f;
    lastYawTime = now;
    yaw += GYRO_SIGN * (imu.gyroZ() - gyroOffset) * dt;
    yaw = wrapAngle(yaw);
}

void resetYaw() {
    yaw = 0.0f;
    lastYawTime = micros();
}

void calibrateGyro() {
    Serial.print("Calibrating gyro... keep robot still ");
    gyroOffset = 0.0f;
    delay(500);
    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) {
        gyroOffset += imu.gyroZ();
        delay(IMU_CALIBRATION_DELAY_MS);
    }
    gyroOffset /= (float)IMU_CALIBRATION_SAMPLES;
    Serial.println("OK");
}

void driveForwardPWM(int leftPwm, int rightPwm) {
    leftPwm = constrain(leftPwm, 0, 255);
    rightPwm = constrain(rightPwm, 0, 255);
    leftMotor.forward((uint8_t)leftPwm);
    rightMotor.forward((uint8_t)rightPwm);
}

bool driveDistance(float distanceMm) {
    leftMotor.setLinearDisplacement(0);
    rightMotor.setLinearDisplacement(0);
    resetYaw();

    unsigned long startMs = millis();

    while (true) {
        updateYaw();

        float leftDist = fabs(leftMotor.linearDisplacement());
        float rightDist = fabs(rightMotor.linearDisplacement());
        float distance = 0.5f * (leftDist + rightDist);
        float error = distanceMm - distance;

        if (error <= DRIVE_TOL_MM) break;
        if (millis() - startMs > DRIVE_TIMEOUT_MS) {
            Serial.println("Drive timeout");
            break;
        }

        int base = (int)(DRIVE_KP * error);
        base = constrain(base, DRIVE_MIN_PWM, DRIVE_MAX_PWM);

        int correction = (int)(DRIVE_YAW_KP * yaw);
        int leftPwm = base + correction;
        int rightPwm = base - correction;

        driveForwardPWM(leftPwm, rightPwm);
        delay(5);
    }

    stopMotors();
    delay(150);
    return true;
}

void rotateLeftPWM(int pwm) {
    pwm = constrain(pwm, 0, 255);
    leftMotor.backward((uint8_t)pwm);
    rightMotor.forward((uint8_t)pwm);
}

void rotateRightPWM(int pwm) {
    pwm = constrain(pwm, 0, 255);
    leftMotor.forward((uint8_t)pwm);
    rightMotor.backward((uint8_t)pwm);
}

bool turnAngle(float targetRad) {
    resetYaw();
    unsigned long startMs = millis();

    while (true) {
        updateYaw();
        float error = wrapAngle(targetRad - yaw);

        if (fabs(error) <= TURN_TOL_RAD) break;
        if (millis() - startMs > TURN_TIMEOUT_MS) {
            Serial.println("Turn timeout");
            break;
        }

        int pwm = (int)(TURN_KP * fabs(error));
        pwm = constrain(pwm, TURN_MIN_PWM, TURN_MAX_PWM);

        if (error > 0.0f) rotateLeftPWM(pwm);
        else rotateRightPWM(pwm);

        delay(5);
    }

    stopMotors();
    delay(150);
    return true;
}

uint8_t turnDiff(MazeMapper::Heading from, MazeMapper::Heading to) {
    return ((uint8_t)to + 4 - (uint8_t)from) & 3;
}

void faceDirection(MazeMapper::Heading target) {
    uint8_t diff = turnDiff(mapper.dir(), target);
    if (diff == 0) return;
    if (diff == 1) turnAngle(MM_HALF_PI);
    else if (diff == 3) turnAngle(-MM_HALF_PI);
    else turnAngle(MM_PI);
}

void moveToCell(MazeMapper::Cell next, MazeMapper::Heading moveDir) {
    Serial.print("Move to cell ");
    Serial.print(next.r);
    Serial.print(", ");
    Serial.println(next.c);

    faceDirection(moveDir);
    driveDistance(MAZE_CELL_MM);
    mapper.setCurrent(next, moveDir);
}

void readWalls(bool& frontWall, bool& leftWall, bool& rightWall) {
    uint8_t frontCount = 0;
    uint8_t leftCount = 0;
    uint8_t rightCount = 0;

    for (uint8_t i = 0; i < 3; ++i) {
        lidar.update();
        if (lidar.wall(LIDAR::Front)) ++frontCount;
        if (lidar.wall(LIDAR::Left)) ++leftCount;
        if (lidar.wall(LIDAR::Right)) ++rightCount;
        delay(25);
    }

    frontWall = frontCount >= 2;
    leftWall = leftCount >= 2;
    rightWall = rightCount >= 2;

    Serial.print("Walls F/L/R: ");
    Serial.print(frontWall);
    Serial.print(" ");
    Serial.print(leftWall);
    Serial.print(" ");
    Serial.print(rightWall);
    Serial.print("   Dist F/L/R: ");
    Serial.print(lidar.getReading(LIDAR::Front));
    Serial.print(" ");
    Serial.print(lidar.getReading(LIDAR::Left));
    Serial.print(" ");
    Serial.println(lidar.getReading(LIDAR::Right));
}

void exploreMaze() {
    Serial.println("Starting autonomous exploration");
    mapper.begin(START_ROW, START_COL, (MazeMapper::Heading)START_HEADING, GOAL_ROW, GOAL_COL);

    while (!mapper.doneExploring()) {
        Serial.print("Current cell: ");
        Serial.print(mapper.row());
        Serial.print(", ");
        Serial.print(mapper.col());
        Serial.print(" heading ");
        Serial.println((int)mapper.dir());

        bool frontWall, leftWall, rightWall;
        readWalls(frontWall, leftWall, rightWall);
        mapper.observe(frontWall, leftWall, rightWall);

        if (mapper.atGoal()) {
            Serial.println("Goal has been discovered. Continuing mapping so shortest path is based on explored cells.");
        }

        MazeMapper::Cell next;
        MazeMapper::Heading moveDir;
        bool backtracking;

        if (mapper.chooseNext(next, moveDir, backtracking)) {
            if (backtracking) Serial.println("Backtracking");
            moveToCell(next, moveDir);
        } else {
            Serial.println("Exploration complete");
        }
    }
}

void runShortestPath() {
    Serial.println("Building shortest path from start to goal");

    if (!mapper.buildShortestPathToGoal()) {
        Serial.println("No known path to goal. Check mapping/wall thresholds.");
        return;
    }

    uint8_t n = mapper.shortestPathLength();
    Serial.print("Shortest path cells: ");
    Serial.println(n);

    for (uint8_t i = 1; i < n; ++i) {
        MazeMapper::Cell next = mapper.shortestPathCell(i);
        MazeMapper::Heading d = mapper.directionTo({mapper.row(), mapper.col()}, next);
        moveToCell(next, d);
    }

    Serial.println("Final shortest-path run complete");
}

void runTask43() {
    exploreMaze();
    delay(1000);
    Serial.println("Returning to shortest path run");
    runShortestPath();
    stopMotors();
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Week 12 Task 4.3 Autonomous Mapping");

    i2cRepairer.begin();

    leftMotor.init();
    rightMotor.init();
    stopMotors();

    Serial.print("Initialising IMU... ");
    if (imu.init()) Serial.println("OK");
    else Serial.println("FAILED");

    calibrateGyro();

    Serial.print("Initialising LiDAR... ");
    if (lidar.begin()) Serial.println("OK");
    else Serial.println("FAILED - check VL6180X wiring/GPO order");

    delay(500);
}

void loop() {
    if (!taskStarted) {
        taskStarted = true;
        runTask43();
        Serial.println("Task 4.3 finished. Reset Arduino to run again.");
    }
}
