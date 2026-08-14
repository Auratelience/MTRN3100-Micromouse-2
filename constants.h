#pragma once
#include <Arduino.h>


constexpr float MM_PI      = 3.14159265358979323846f;
constexpr float MM_TWO_PI  = 6.28318530717958647692f;
constexpr float MM_HALF_PI = 1.57079632679489661923f;

// Robot geometry / encoders
constexpr float WHEEL_DIAMETER_MM = 31.4f;
constexpr float WHEEL_RADIUS_MM   = WHEEL_DIAMETER_MM / 2.0f;
constexpr int   ENC_CPR           = 700;
constexpr float ENC_SCALE_LEFT    = 1.0f;
constexpr float ENC_SCALE_RIGHT   = 1.0f;

// Maze
constexpr uint8_t MAZE_SIZE = 9;
constexpr float   MAZE_CELL_MM = 180.0f;

// Start and goal: edit these to match the demonstrator's task.
constexpr int START_ROW = 0;
constexpr int START_COL = 0;
constexpr int GOAL_ROW  = 4;
constexpr int GOAL_COL  = 4;

// 0=N, 1=E, 2=S, 3=W
constexpr uint8_t START_HEADING = 0;

// Drive tuning
constexpr int DRIVE_MIN_PWM = 48;
constexpr int DRIVE_MAX_PWM = 105;
constexpr float DRIVE_KP = 1.4f;
constexpr float DRIVE_YAW_KP = 90.0f;
constexpr float DRIVE_TOL_MM = 6.0f;
constexpr unsigned long DRIVE_TIMEOUT_MS = 5000;

// Turn tuning
constexpr int TURN_MIN_PWM = 48;
constexpr int TURN_MAX_PWM = 95;
constexpr float TURN_KP = 115.0f;
constexpr float TURN_TOL_RAD = 0.055f;   // about 3.2 degrees
constexpr unsigned long TURN_TIMEOUT_MS = 3500;

// Change to -1.0f if gyro heading increases in the wrong direction.
constexpr float GYRO_SIGN = 1.0f;

// LiDAR
constexpr uint8_t LIDAR_DEFAULT_ADDRESS = 0x29;
constexpr uint8_t LIDAR_FRONT_ADDRESS   = 0x30;
constexpr uint8_t LIDAR_LEFT_ADDRESS    = 0x31;
constexpr uint8_t LIDAR_RIGHT_ADDRESS   = 0x32;
constexpr uint16_t LIDAR_TIMEOUT_MS     = 50;
constexpr uint16_t LIDAR_NO_TARGET_MM   = 300;
constexpr uint16_t WALL_THRESHOLD_MM    = 125;

// I2C
constexpr uint32_t I2C_FREQ = 400000UL;
constexpr unsigned int I2C_WIRE_TIMEOUT_US = 50000;
constexpr unsigned int I2C_RECOVERY_HALF_PERIOD_US = 5;
constexpr uint8_t I2C_RECOVERY_MAX_PULSES = 9;

// IMU
constexpr uint8_t IMU_ADDRESS = 0x68;
constexpr int IMU_CALIBRATION_SAMPLES = 400;
constexpr int IMU_CALIBRATION_DELAY_MS = 3;
