// Control
//
// Zimmy Levi z5587840

#pragma once

#include "motor.h"
#include "constants.h"
#include "types.h"
#include "kinematics.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

class PID {
    public:

    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    float step(float setpoint, float measurement, float dt) {
        // Bail on any non-finite input so a stray NaN/Inf can't permanently
        // poison the integrator or flow through roundf() into an int cast.
        if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(dt)) return 0.0f;
        float error = setpoint - measurement;
        if (justReset) {
            prevError = error;
            justReset = false;
        }
        if (ki != 0) integral += error * dt;
        if (integral > PID_SATURATION_ABSOLUTE) integral = PID_SATURATION_ABSOLUTE;
        if (integral < -PID_SATURATION_ABSOLUTE) integral = -PID_SATURATION_ABSOLUTE;
        float derivative = 0;
        if (kd != 0) derivative = (error - prevError) / dt;
        prevError    = error;
        float result = kp * error + ki * integral + kd * derivative;
        if (result >= PID_SATURATION_ABSOLUTE) return PID_SATURATION_ABSOLUTE;
        if (result <= -PID_SATURATION_ABSOLUTE) return -PID_SATURATION_ABSOLUTE;
        else return result;
    }

    void reset() {
        integral  = 0;
        prevError = 0;
        justReset = true;
    }

    private:

    float kp, ki, kd;
    float integral  = 0;
    float prevError = 0;
    bool justReset  = true;
};

class MotionController {
    public:

    MotionController(
        BaseMotor& leftMotor,
        BaseMotor& rightMotor,
        Kinematics& kinematics,
        float kp,
        float ki,
        float kd
    ) :
        leftMotor(leftMotor),
        rightMotor(rightMotor),
        kinematics(kinematics),
        leftPID(PID(kp, ki, kd)),
        rightPID(PID(kp, ki, kd)) {}

    void update(Velocity desired, Velocity current, float dt) {
        // A zero desired velocity means "stop", not "regulate to zero with a
        // wound-up integrator still driving the motors". Cut the motors and
        // clear PID state so a finished plan (planner Wait -> {0, 0}) brings
        // the robot to rest instead of coasting on the stale integral term.
        if (desired.v == 0.0f && desired.omega == 0.0f) {
            reset();
            leftMotor.stop();
            rightMotor.stop();
            return;
        }

        WheelVelocities targetWV  = kinematics.ik.velocity(desired);
        WheelVelocities currentWV = kinematics.ik.velocityRaw(current);

        int left  = roundf(leftPID.step(targetWV.left, currentWV.left, dt));
        int right = roundf(rightPID.step(targetWV.right, currentWV.right, dt));

        leftMotor.move(clamp(left));
        rightMotor.move(clamp(right));
    }

    void reset() {
        leftPID.reset();
        rightPID.reset();
    }

    private:

    static int16_t clamp(int x) {
        if (x > MAXIMUM_WHEEL_PWM) return MAXIMUM_WHEEL_PWM;
        if (x < -MAXIMUM_WHEEL_PWM) return -MAXIMUM_WHEEL_PWM;
        return x;
    }

    BaseMotor& leftMotor;
    BaseMotor& rightMotor;
    Kinematics& kinematics;
    PID leftPID;
    PID rightPID;
};

#pragma GCC pop_options
