// Pin definitions for Nano
//
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>
using Pin = uint8_t;

// Motors
// PWM outputs to xEN, and DIR goes to xPH on H-bridge
// ENC inputs from the motors directly
// DIR / phase := 0 is forwards
constexpr Pin MOT_1_ENC_A = 2;
constexpr Pin MOT_1_ENC_B = 7;
constexpr Pin MOT_1_PWM   = 11;
constexpr Pin MOT_1_DIR   = 12;

constexpr Pin MOT_2_ENC_A = 3;
constexpr Pin MOT_2_ENC_B = 8;
constexpr Pin MOT_2_PWM   = 9;
constexpr Pin MOT_2_DIR   = 10;

// Lidar
// GPO should be set to low to turn off a lidar
// GPO should be set to input to turn on lidar
// The lidar will pull itself to 2.8V
// DO NOT drive high as this will destroy the lidar
constexpr Pin TOF_1_GPO = A0; // Left lidar
constexpr Pin TOF_2_GPO = A1; // right lidar
constexpr Pin TOF_3_GPO = A2; // centre lidar

// I2C
constexpr Pin I2C_SCL = A5;
constexpr Pin I2C_SDA = A4;
