// Lidar.h
//
// Zimmy Levi z5587840
// VL6180X

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
#include <array>
#include <cstdint>

#include "pins.h"
#include "constants.h"

namespace L {
    enum Register : uint16_t {
        HIGH_THRESHOLD              = 0x019,
        LOW_THRESHOLD               = 0x01A,
        INTERMEASUREMENT_PERIOD     = 0x01B,
        CONVERGENCE_TIME            = 0x01C,
        ALS_INTERMEASUREMENT_PERIOD = 0x03E,
        AVERAGING_SAMPLE_PERIOD     = 0x040,
        RES_RANGE_STATUS            = 0x04D,
        RES_RANGE_VAL               = 0x062
    };

    enum Convergence : uint8_t {
        ms11 = 0x06, // max range approx 50mm
        ms18 = 0x12, // max range approx 80mm
        ms31 = 0x1A,
        ms50 = 0x31, // default
        ms63 = 0x3C  // long range
    };

    enum AveragingPeriod : uint8_t {
        SAMPLES_32  = 0x20,
        SAMPLES_48  = 0x30,
        SAMPLES_64  = 0x40, // Default
        SAMPLES_128 = 0x80
    };

    enum Scale : uint8_t {
        X1 = 1, // 1mm res, 255mm max
        X2 = 2, // 2mm res, 400mm max
        X3 = 3  // 3mm res, 600mm max
    };

    // RESULT__RANGE_STATUS[7:4] error codes, per VL6180X datasheet
    enum Status : uint8_t {
        OK                         = 0x00,
        VCSEL_CONTINUITY_TEST_FAIL = 0x01, // Laser hw fault
        VCSEL_WATCHDOG_TEST_FAIL   = 0x02, // as above
        VCSEL_WATCHDOG_TIMEOUT     = 0x03, // as above
        PLL1_LOCK_FAIL             = 0x04,
        PLL2_LOCK_FAIL             = 0x05,
        EARLY_CONVERGENCE_ERROR    = 0x06, // Ambient infrared light saturation error
        MAX_CONVERGENCE_TIMEOUT    = 0x07, // Convergence time limit reached (no target in range)
        NO_TARGET_IGNORE_THRESHOLD = 0x08, // Target reflectivity too low
        MAX_SIGNAL_TO_NOISE_ERROR  = 0x0B, // Excessive optical noise
        RAW_RANGING_ALGO_UNDERFLOW = 0x0C, // Object too close
        RAW_RANGING_ALGO_OVERFLOW  = 0x0D, // Object too far
        RANGING_ALGO_UNDERFLOW     = 0x0E, // Scaled range < 0 (too close)
        RANGING_ALGO_OVERFLOW      = 0x0F  // Scaled range out of range (too far)
    };

    enum ReadingConstants : uint16_t {
        MIN_DIST = 0,
        MAX_DIST = 300,

        TIMEOUT = 500,
        GENERIC_ERR
    };
}

class LidarSensor {
    public:

    LidarSensor(const uint8_t address, const Pin pin) :
        addr(address),
        pin(pin),
        sensor(VL6180X()) {}

    void init(uint16_t timeout = LIDAR_TIMEOUT_MS) {
        sensor.init();
        sensor.setAddress(addr);
        sensor.setTimeout(timeout);
        sensor.configureDefault();

        // speed optimisations
        sensor.writeReg(L::CONVERGENCE_TIME, L::ms18);
        sensor.writeReg(L::AVERAGING_SAMPLE_PERIOD, L::SAMPLES_48);
        sensor.setScaling(scale);
        sensor.startRangeContinuous(LIDAR_CONTINUOUS_PERIOD_MS); // 10 ms
    }

    void on() {
        pinMode(pin, INPUT);
    }

    void off() {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    bool exists() {
        Wire.beginTransmission(addr);
        return Wire.endTransmission() == 0;
    }

    uint16_t read() {
        // Out-of-range measurements never raise the sample-ready
        // interrupt, so the library read below would busy-poll I2C for
        // the full timeout. Check the status of the last measurement
        // first and bail out without blocking. No serial prints here:
        // these are normal conditions (open maze = too far) and occur
        // at loop rate.
        L::Status s = status();

        if (s == L::RAW_RANGING_ALGO_UNDERFLOW || s == L::RANGING_ALGO_UNDERFLOW) {
            return L::MIN_DIST;
        }

        if (s == L::MAX_CONVERGENCE_TIMEOUT || s == L::NO_TARGET_IGNORE_THRESHOLD ||
            s == L::MAX_SIGNAL_TO_NOISE_ERROR || s == L::RAW_RANGING_ALGO_OVERFLOW ||
            s == L::RANGING_ALGO_OVERFLOW) {
            return L::MAX_DIST;
        }

        // Already scaled: readRangeContinuousMillimeters() multiplies
        // the raw register value by the scaling factor internally
        uint16_t dist = sensor.readRangeContinuousMillimeters();

        if (sensor.timeoutOccurred()) {
            Serial.print("Lidar error: addr, err: ");
            Serial.print(addr);
            Serial.print(", ");
            Serial.println(s);
            return L::TIMEOUT;
        }

        return dist;
    }

    void recover() {
        sensor.startRangeContinuous(LIDAR_CONTINUOUS_PERIOD_MS);
    }

    private:

    L::Status status() {
        // Bitshift selects only status bits, not other flags
        return static_cast<L::Status>(sensor.readReg(L::RES_RANGE_STATUS) >> 4);
    }

    const uint8_t addr;
    const Pin pin;
    const uint8_t scale = L::X2;
    VL6180X sensor;
};

class LIDAR {
    public:

    enum Sensors : uint8_t { Front, Left, Right, COUNT };

    LIDAR(std::array<LidarSensor*, COUNT> sensors) : sensors(sensors) {}

    bool init() {
        for (LidarSensor* s : sensors)
            s->off();
        delay(LIDAR_BRINGUP_SETTLE_MS);
        for (LidarSensor* s : sensors) {
            s->on();
            delay(LIDAR_BRINGUP_SETTLE_MS);
            s->init();
            delay(LIDAR_BRINGUP_SETTLE_MS);
            if (!s->exists()) return false;
        }
        return true;
    }

    void update() {
        for (int i = 0; i < COUNT; ++i) {
            uint16_t val = sensors[i]->read();
            if (val == L::GENERIC_ERR) continue;
            if (val == L::TIMEOUT) {
                sensors[i]->recover();
                continue;
            }
            readings[i] = val;
        }
    }

    uint16_t getReading(Sensors s) const {
        return readings[s];
    }

    private:

    std::array<uint16_t, COUNT> readings = {0, 0, 0};
    std::array<LidarSensor*, COUNT> sensors;
};
