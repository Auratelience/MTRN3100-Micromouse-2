#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "constants.h"

class I2CRepairer {
public:
    I2CRepairer(Pin sda, Pin scl, uint32_t clockHz = I2C_FREQ) :
        sda(sda), scl(scl), clockHz(clockHz) {}

    void begin() {
        clearBus();
        Wire.begin();
        Wire.setClock(clockHz);
        #ifdef WIRE_HAS_TIMEOUT
        Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US);
        #endif
    }

    void recover() {
        Wire.end();
        clearBus();
        Wire.begin();
        Wire.setClock(clockHz);
        #ifdef WIRE_HAS_TIMEOUT
        Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US);
        #endif
    }

private:
    Pin sda;
    Pin scl;
    uint32_t clockHz;

    void driveLow(Pin p) {
        digitalWrite(p, LOW);
        pinMode(p, OUTPUT);
    }

    void release(Pin p) {
        pinMode(p, INPUT_PULLUP);
    }

    void clearBus() {
        release(sda);
        release(scl);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);

        for (uint8_t i = 0; i < I2C_RECOVERY_MAX_PULSES && digitalRead(sda) == LOW; ++i) {
            driveLow(scl);
            delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
            release(scl);
            delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        }

        driveLow(sda);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        release(scl);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
        release(sda);
        delayMicroseconds(I2C_RECOVERY_HALF_PERIOD_US);
    }
};
