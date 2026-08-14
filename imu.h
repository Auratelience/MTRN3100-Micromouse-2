#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "constants.h"

class IMU {
public:
    bool init(uint8_t address = IMU_ADDRESS) {
        this->address = address;
        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0) return false;
        writeReg(0x6B, 0x00); // wake
        writeReg(0x1B, 0x10); // gyro +/-1000 dps
        writeReg(0x1C, 0x08); // accel +/-4g
        writeReg(0x1A, 0x03); // DLPF 44Hz
        return true;
    }

    float gyroZ() {
        int16_t raw = readWord(0x47);
        return ((float)raw / 32.8f) * DEG_TO_RAD;
    }

private:
    uint8_t address = IMU_ADDRESS;

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
        Wire.requestFrom((int)address, 2);
        if (Wire.available() < 2) return 0;
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        return (int16_t)((hi << 8) | lo);
    }
};
