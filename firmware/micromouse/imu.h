// IMU
// Raw I2C driver for MPU6050-compatible sensors
//
// Artificially-generated code in this file

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"

// Handles the bootleg-ish IMU6080 we have been given
class IMU {
    public:

    // degrees per second
    enum class GyroScale : uint8_t {
        DPS_250  = 0x00, // 131.0 LSB per °/s
        DPS_500  = 0x08, // 65.5  LSB per °/s
        DPS_1000 = 0x10, // 32.8  LSB per °/s
        DPS_2000 = 0x18  // 16.4  LSB per °/s
    };

    // 9.8m/s²
    enum class AccelScale : uint8_t {
        G_2  = 0x00, // 16384 LSB/g
        G_4  = 0x08, // 8192  LSB/g
        G_8  = 0x10, // 4096  LSB/g
        G_16 = 0x18  // 2048  LSB/g
    };

    enum class LowPassFrequency : uint8_t {
        HZ_260 = 0,
        HZ_184 = 1,
        HZ_94  = 2,
        HZ_44  = 3,
        HZ_21  = 4,
        HZ_10  = 5,
        HZ_5   = 6
    };

    IMU(uint8_t address = IMU_ADDRESS) : address(address) {}

    // Sane defaults
    bool init(
        GyroScale gyro        = GyroScale::DPS_1000,
        AccelScale accel      = AccelScale::G_4,
        LowPassFrequency dlpf = LowPassFrequency::HZ_44
    ) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0) return false;

        // Set up IMU
        writeReg(PWR_MGMT_1, 0x00);
        writeReg(GYRO_CONFIG, static_cast<uint8_t>(gyro));
        writeReg(ACCEL_CONFIG, static_cast<uint8_t>(accel));
        writeReg(CONFIG, static_cast<uint8_t>(dlpf));

        gyroscopicSensitivity    = gyroSensitivity(gyro);
        accelerometerSensitivity = accelSensitivity(accel);

        return true;
    }

    float gyroX() {
        return toRadPerSec(gyroXAvg.push(readWord(GYRO_XOUT_H)));
    }

    float gyroY() {
        return toRadPerSec(gyroYAvg.push(readWord(GYRO_YOUT_H)));
    }

    float gyroZ() {
        return toRadPerSec(gyroZAvg.push(readWord(GYRO_ZOUT_H)));
    }

    float accelX() {
        return toMPerSec2(accelXAvg.push(readWord(ACCEL_XOUT_H)));
    }

    float accelY() {
        return toMPerSec2(accelYAvg.push(readWord(ACCEL_YOUT_H)));
    }

    float accelZ() {
        return toMPerSec2(accelZAvg.push(readWord(ACCEL_ZOUT_H)));
    }

    private:

    static constexpr uint8_t PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t CONFIG       = 0x1A;
    static constexpr uint8_t GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
    static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
    static constexpr uint8_t GYRO_XOUT_H  = 0x43;
    static constexpr uint8_t GYRO_YOUT_H  = 0x45;
    static constexpr uint8_t GYRO_ZOUT_H  = 0x47;

    static constexpr float GRAVITY = 9.80665f;

    // Fixed-size rolling average over the last IMU_ROLLING_AVG_SAMPLES raw readings.
    class RollingAverage {
        public:

        int16_t push(int16_t sample) {
            sum -= samples[index];
            samples[index] = sample;
            sum += sample;
            index = (index + 1) % IMU_ROLLING_AVG_SAMPLES;
            if (count < IMU_ROLLING_AVG_SAMPLES) count++;
            return static_cast<int16_t>(sum / count);
        }

        private:

        int16_t samples[IMU_ROLLING_AVG_SAMPLES] = {0};
        int32_t sum                              = 0;
        uint8_t index                            = 0;
        uint8_t count                            = 0;
    };

    uint8_t address;
    float gyroscopicSensitivity;
    float accelerometerSensitivity;

    RollingAverage gyroXAvg, gyroYAvg, gyroZAvg;
    RollingAverage accelXAvg, accelYAvg, accelZAvg;

    float toRadPerSec(int16_t raw) const {
        return (raw / gyroscopicSensitivity) * DEG_TO_RAD;
    }

    float toMPerSec2(int16_t raw) const {
        return (raw / accelerometerSensitivity) * GRAVITY;
    }

    static float gyroSensitivity(GyroScale scale) {
        switch (scale) {
            case GyroScale::DPS_250:  return 131.0f;
            case GyroScale::DPS_500:  return 65.5f;
            case GyroScale::DPS_1000: return 32.8f;
            case GyroScale::DPS_2000: return 16.4f;
            default:                  return 32.8f;
        }
    }

    static float accelSensitivity(AccelScale scale) {
        switch (scale) {
            case AccelScale::G_2:  return 16384.0f;
            case AccelScale::G_4:  return 8192.0f;
            case AccelScale::G_8:  return 4096.0f;
            case AccelScale::G_16: return 2048.0f;
            default:               return 8192.0f;
        }
    }

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    int16_t readWord(uint8_t reg) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return 0;
        Wire.requestFrom(address, (uint8_t)2);
        if (Wire.available() < 2) return 0;
        uint8_t hi = (uint8_t)Wire.read();
        uint8_t lo = (uint8_t)Wire.read();
        return (int16_t)((uint16_t)(hi << 8) | lo);
    }
};
