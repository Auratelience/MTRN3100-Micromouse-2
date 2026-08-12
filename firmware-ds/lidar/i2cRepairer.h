// I2C Bus supervisor
//
// Artifically-generated code in this file

#pragma once

#include "constants.h"
#include <Arduino.h>
#include <Wire.h>

// Owns I2C bring-up and runtime recovery.
//
// The renesas_uno core's Wire never recovers on its own after a wedged
// transaction: a timeout does not abort the in-flight FSP transfer, so a
// slave left holding SDA low kills every device on the bus until the bus
// is manually cleared and the driver reopened. This class replaces the
// one-shot unstick routine in setup() with one that can also run at any
// point after boot.
//
// Tunables (timeouts, thresholds, recovery clocking) live in constants.h
// under the I2C section.
//
// Usage:
//   I2CBus i2c(I2C_SDA, I2C_SCL);
//   i2c.begin();   // in setup(), before any device init
//   i2c.update();  // every loop(); probes periodically, recovers if stuck
class I2CRepairer {
    public:

    I2CRepairer(int sda, int scl, uint8_t probeAddress, uint32_t clockHz = I2C_FREQ) :
        sda(sda),
        scl(scl),
        clockHz(clockHz),
        probeAddress(probeAddress),
        consecutiveFailures(0),
        recoveryCount(0),
        lastProbeMs(0),
        lastRecoveryMs(0) {}

    // Clears the bus and starts Wire. Call once in setup() before
    // initialising any I2C device.
    void begin() {
        clearBus();
        Wire.begin();
        Wire.setClock(clockHz);
        Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US);
    }

    // Call every loop(). Pings the probe device every I2C_PROBE_INTERVAL_MS
    // and rebuilds the bus after I2C_RECOVERY_FAILURE_THRESHOLD consecutive
    // failures. Costs one 1-byte transaction per probe; nothing between probes.
    void update() {
        unsigned long now = millis();
        if (now - lastProbeMs < I2C_PROBE_INTERVAL_MS) return;
        lastProbeMs = now;

        Wire.beginTransmission(probeAddress);
        if (Wire.endTransmission() == 0) {
            consecutiveFailures = 0;
            return;
        }

        if (consecutiveFailures < I2C_RECOVERY_FAILURE_THRESHOLD) ++consecutiveFailures;
        if (consecutiveFailures < I2C_RECOVERY_FAILURE_THRESHOLD) return;
        if (now - lastRecoveryMs < I2C_RECOVERY_COOLDOWN_MS) return;

        recover();
        lastRecoveryMs = millis();
    }

    // Tears down Wire, manually clears the bus, and brings Wire back up.
    // Device registers (MPU6050 config, SSD1306 state) survive as they
    // are only lost on power cycle.
    void recover() {
        Wire.end();
        clearBus();
        Wire.begin();
        Wire.setClock(clockHz);
        Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US);
        ++recoveryCount;
        Serial.print("I2C bus recovery: (");
        Serial.print(recoveryCount);
        Serial.println(")");
    }

    // Number of recoveries since boot; useful on the OLED / serial dump
    unsigned int recoveries() const {
        return recoveryCount;
    }

    private:

    // Clock out up to I2C_RECOVERY_MAX_PULSES pulses so a slave stuck mid-byte
    // can finish and release SDA, then issue a STOP. Pins are only ever driven
    // low or released to the pull-up (open-drain emulation) — driving them high
    // against a slave holding the line low would short the driver.
    void clearBus() {
        releasePin(sda);
        releasePin(scl);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);

        for (int i = 0; i < I2C_RECOVERY_MAX_PULSES && digitalRead(sda) == LOW; ++i) {
            drivePinLow(scl);
            delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
            releasePin(scl);
            delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        }

        // STOP condition: SDA rises while SCL is high
        drivePinLow(sda);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        releasePin(scl);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        releasePin(sda);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
    }

    void drivePinLow(int pin) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }

    void releasePin(int pin) {
        pinMode(pin, INPUT_PULLUP);
    }

    const int sda;
    const int scl;
    const uint32_t clockHz;
    const uint8_t probeAddress;

    uint8_t consecutiveFailures;
    unsigned int recoveryCount;
    unsigned long lastProbeMs;
    unsigned long lastRecoveryMs;
};
