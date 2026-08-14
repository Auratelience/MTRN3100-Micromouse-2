#pragma once
#include <Arduino.h>
using Pin = uint8_t;

// Motors
constexpr Pin MOT_1_ENC_A = 2;
constexpr Pin MOT_1_ENC_B = 7;
constexpr Pin MOT_1_PWM   = 11;
constexpr Pin MOT_1_DIR   = 12;

constexpr Pin MOT_2_ENC_A = 3;
constexpr Pin MOT_2_ENC_B = 8;
constexpr Pin MOT_2_PWM   = 9;
constexpr Pin MOT_2_DIR   = 10;

// LiDAR GPO pins. LOW = off, INPUT = on. Never drive HIGH.
constexpr Pin TOF_1_GPO = A0; // left
constexpr Pin TOF_2_GPO = A1; // right
constexpr Pin TOF_3_GPO = A2; // front/centre

// I2C
constexpr Pin I2C_SCL = A5;
constexpr Pin I2C_SDA = A4;
