#pragma once
#include <Arduino.h>
#include "constants.h"
#include "pins.h"

class BaseMotor {
public:
    virtual void init() = 0;
    virtual void forward(uint8_t speed) = 0;
    virtual void backward(uint8_t speed) = 0;
    virtual void move(int16_t speed) = 0;
    virtual void stop() = 0;
    virtual long count() = 0;
    virtual float linearDisplacement() = 0;
    virtual float angularDisplacement() = 0;
    virtual void setAngularDisplacement(float a) = 0;
    virtual void setLinearDisplacement(float d) = 0;
};

template <unsigned char ID>
class Motor : public BaseMotor {
public:
    Motor(float wheelRadius, Pin direction, Pin pwm, Pin encoderA, Pin encoderB,
          int countsPerRevolution, bool reverse, float encoderScale = 1.0f) :
        wheelRadius(wheelRadius),
        phase(direction),
        enable(pwm),
        encoderA(encoderA),
        encoderB(encoderB),
        countsPerRevolution(countsPerRevolution),
        reverse(reverse),
        encoderScale(encoderScale),
        encoderCount(0)
    {}

    void init() override {
        pinMode(phase, OUTPUT);
        pinMode(enable, OUTPUT);
        pinMode(encoderA, INPUT_PULLUP);
        pinMode(encoderB, INPUT_PULLUP);
        instance = this;
        attachInterrupt(digitalPinToInterrupt(encoderA), Motor<ID>::readEncoderISR, RISING);
        stop();
    }

    void forward(uint8_t speed) override {
        digitalWrite(phase, reverse ? HIGH : LOW);
        analogWrite(enable, speed);
    }

    void backward(uint8_t speed) override {
        digitalWrite(phase, reverse ? LOW : HIGH);
        analogWrite(enable, speed);
    }

    void move(int16_t speed) override {
        if (speed > 255) speed = 255;
        if (speed < -255) speed = -255;
        if (speed > 0) forward((uint8_t)speed);
        else if (speed < 0) backward((uint8_t)(-speed));
        else stop();
    }

    void stop() override {
        analogWrite(enable, 0);
    }

    long count() override {
        long copy;
        noInterrupts();
        copy = encoderCount;
        interrupts();
        return copy;
    }

    float angularDisplacement() override {
        return encoderScale * MM_TWO_PI * (float)count() / (float)countsPerRevolution;
    }

    float linearDisplacement() override {
        return angularDisplacement() * wheelRadius;
    }

    void setAngularDisplacement(float a) override {
        long target = (long)((a / (encoderScale * MM_TWO_PI)) * (float)countsPerRevolution);
        noInterrupts();
        encoderCount = target;
        interrupts();
    }

    void setLinearDisplacement(float d) override {
        setAngularDisplacement(d / wheelRadius);
    }

private:
    const float wheelRadius;
    const Pin phase;
    const Pin enable;
    const Pin encoderA;
    const Pin encoderB;
    const int countsPerRevolution;
    const bool reverse;
    const float encoderScale;
    volatile long encoderCount;

    static Motor<ID>* instance;

    static void readEncoderISR() {
        if (instance) instance->readEncoder();
    }

    void readEncoder() {
        bool bHigh = digitalRead(encoderB) == HIGH;
        if (!bHigh) {
            if (reverse) ++encoderCount;
            else --encoderCount;
        } else {
            if (reverse) --encoderCount;
            else ++encoderCount;
        }
    }
};

template <unsigned char ID>
Motor<ID>* Motor<ID>::instance = nullptr;
