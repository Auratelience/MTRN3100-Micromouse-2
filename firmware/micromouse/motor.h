// Motor.hpp
// Header-only library due to use of templates
//
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>

#include "pins.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

class BaseMotor {
    public:

    virtual void init()                          = 0;
    virtual void forward(uint8_t speed)          = 0;
    virtual void backward(uint8_t speed)         = 0;
    virtual void move(int16_t speed)             = 0;
    virtual void stop()                          = 0;
    virtual long count()                         = 0;
    virtual float linearDisplacement()           = 0;
    virtual float angularDisplacement()          = 0;
    virtual void setAngularDisplacement(float a) = 0;
    virtual void setLinearDisplacement(float d)  = 0;
};

// Implements controls for a simple DC motor over a
// DRV8835 motor driver.
//
// Usage: Motor<0> leftMotor(...);
//        Motor<1> rightMotor(...);
//
// Variables intending to use Motor type should use BaseMotor instead
template <unsigned char ID>
class Motor : public BaseMotor {
    public:

    Motor(
        float wheelRadius,
        Pin direction,
        Pin pwm,
        Pin encoderA,
        Pin encoderB,
        int countsPerRevolution,
        bool reverse,
        float encoderScale = 1.0f
    ) :
        wheelRadius(wheelRadius),
        phase(direction),
        enable(pwm),
        countsPerRevolution(countsPerRevolution),
        encoderA(encoderA),
        encoderB(encoderB),
        encoderCount(0),
        reverse(reverse),
        encoderScale(encoderScale),
        encoderB_port(nullptr),
        encoderB_mask(0){}

    // Sets the pinMode for the two motor pins
    //
    // Side effects: modifies pinModes for pins at this.phase and .enable
    void init() override {
        pinMode(phase, OUTPUT);
        pinMode(enable, OUTPUT);
        pinMode(encoderA, INPUT_PULLUP);
        pinMode(encoderB, INPUT_PULLUP);

        encoderB_port = portInputRegister(digitalPinToPort(encoderB));
        encoderB_mask = digitalPinToBitMask(encoderB);

        instance = this;

        // Automatically bind the hardware interrupt
        attachInterrupt(digitalPinToInterrupt(encoderA), Motor<ID>::readEncoderISR, RISING);
    }

    // Turns the motor forwards
    //
    // forward(0) is equivalent to this.stop()
    // speed should be in range [0, 255]
    void forward(uint8_t speed) override {
        if (reverse) {
            digitalWrite(phase, HIGH);
        } else {
            digitalWrite(phase, LOW);
        }
        analogWrite(enable, speed);
    }

    // Turns the motor backwards
    //
    // backward(0) is equivalent to this.stop()
    // speed should be in range [0, 255]
    void backward(uint8_t speed) override {
        if (reverse) {
            digitalWrite(phase, LOW);
        } else {
            digitalWrite(phase, HIGH);
        }
        analogWrite(enable, speed);
    }

    // Combines forward and backwards into a single -255 to 255 range.
    void move(int16_t speed) override {
        if (speed < 0) {
            backward(static_cast<uint8_t>(abs(speed)));
        } else {
            forward(static_cast<uint8_t>(speed));
        }
    }

    // Stops the motor
    void stop() override {
        analogWrite(enable, 0);
    }

    // mm
    float linearDisplacement() override {
        return angularDisplacement() * wheelRadius;
    }

    // rad
    float angularDisplacement() override {
        long countCopy;
        noInterrupts();
        countCopy = encoderCount;
        interrupts();

        // Returns angular displacement of the wheel (in rad), corrected by
        // the per-wheel calibration scale
        return encoderScale * TWO_PI * static_cast<float>(countCopy) /
               static_cast<float>(countsPerRevolution);
    }

    long count() override {
        long countCopy;
        noInterrupts();
        countCopy = encoderCount;
        interrupts();
        return countCopy;
    }

    // rad
    void setAngularDisplacement(float a) override {
        long targetCount = static_cast<long>(
            (a / (encoderScale * TWO_PI)) * static_cast<float>(countsPerRevolution)
        );

        noInterrupts();
        encoderCount = targetCount;
        interrupts();
    }

    // mm
    void setLinearDisplacement(float d) override {
        float circumference     = TWO_PI * wheelRadius;
        float targetRevolutions = d / circumference;
        long targetCount        = static_cast<long>(
            (targetRevolutions / encoderScale) * static_cast<float>(countsPerRevolution)
        );

        noInterrupts();
        encoderCount = targetCount;
        interrupts();
    }

    private:

    static inline Motor<ID>* instance = nullptr;

    // direction pin
    const Pin phase;

    // pwm pin
    const Pin enable;

    const Pin encoderA;
    const Pin encoderB;

    const int countsPerRevolution;

    volatile long encoderCount;

    // mm
    const float wheelRadius;

    const bool reverse;

    // Per-wheel calibration: multiplies raw encoder-derived angle so a full
    // revolution reads exactly 2π.
    const float encoderScale;

    const volatile uint16_t* encoderB_port;
    uint16_t encoderB_mask;

    static void readEncoderISR() {
        if (instance != nullptr) instance->readEncoder();
    }

    void readEncoder() {
        if (!(*encoderB_port & encoderB_mask)) {
            if (reverse) {
                ++encoderCount;
            } else {
                --encoderCount;
            }
        } else {
            if (reverse) {
                --encoderCount;
            } else {
                ++encoderCount;
            }
        }
    }
};

#pragma GCC pop_options
