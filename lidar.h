#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
#include "constants.h"
#include "pins.h"

class LidarSensor {
public:
    LidarSensor(uint8_t newAddress, Pin gpoPin) : address(newAddress), pin(gpoPin) {}

    void off() {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }

    void on() {
        pinMode(pin, INPUT);
    }

    bool begin() {
        on();
        delay(80);
        if (!exists(LIDAR_DEFAULT_ADDRESS)) return false;
        sensor.setTimeout(LIDAR_TIMEOUT_MS);
        sensor.init();
        sensor.configureDefault();
        sensor.setAddress(address);
        delay(20);
        ready = exists(address);
        return ready;
    }

    uint16_t read() {
        if (!ready) return LIDAR_NO_TARGET_MM;
        uint16_t d = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) return LIDAR_NO_TARGET_MM;
        if (d == 0) return 0;
        if (d > LIDAR_NO_TARGET_MM) return LIDAR_NO_TARGET_MM;
        return d;
    }

    bool isReady() const { return ready; }

private:
    uint8_t address;
    Pin pin;
    bool ready = false;
    VL6180X sensor;

    bool exists(uint8_t addr) {
        Wire.beginTransmission(addr);
        return Wire.endTransmission() == 0;
    }
};

class LIDAR {
public:
    enum Sensor : uint8_t { Front = 0, Left = 1, Right = 2 };

    LIDAR(LidarSensor* front, LidarSensor* left, LidarSensor* right) {
        sensors[Front] = front;
        sensors[Left] = left;
        sensors[Right] = right;
    }

    bool begin() {
        for (uint8_t i = 0; i < 3; ++i) sensors[i]->off();
        delay(120);

        bool ok = true;
        for (uint8_t i = 0; i < 3; ++i) {
            if (!sensors[i]->begin()) ok = false;
        }
        return ok;
    }

    void update() {
        for (uint8_t i = 0; i < 3; ++i) readings[i] = sensors[i]->read();
    }

    uint16_t getReading(Sensor s) const {
        return readings[s];
    }

    bool wall(Sensor s) const {
        uint16_t d = readings[s];
        return d > 0 && d < WALL_THRESHOLD_MM;
    }

private:
    LidarSensor* sensors[3];
    uint16_t readings[3] = {LIDAR_NO_TARGET_MM, LIDAR_NO_TARGET_MM, LIDAR_NO_TARGET_MM};
};
