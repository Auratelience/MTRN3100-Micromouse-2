// The boot wizard: maze size, start cell, start heading and goal
//
// Zimmy Levi z5587840
//
// Blocking, and called from setup() before the IMU calibrates. The control loop
// does not exist yet and there is nothing to service but I2C, so a blocking
// wizard is far simpler than a loop() state machine -- and it keeps loop() the
// wiring diagram it is.
//
// The motors are never driven from here, so the wheels turn freely under hand.

#pragma once

#include <math.h>

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "constants.h"
#include "i2cRepairer.h"
#include "lidar.h"
#include "mazeMapper.h"  // Cell and RunConfig are declared here, not in types.h
#include "motor.h"
#include "oledDisplay.h"
#include "types.h"

// One wheel as a detented dial.
//
// Incremental, not absolute. Mapping raw position straight onto the value would
// wind up against a clamp: spin ten detents past the maximum and you would need
// ten back before the number moved. take() returns a delta and advances its own
// mark, so a clamped dial responds to the first detent in the other direction.
class UIDial {
    public:

    void begin(long count) { mark = count; origin = count; }

    // Whole detents since the last call, signed, advancing the mark by exactly
    // what it returns so no fraction is lost across calls.
    int take(long count) {
        const long d = (count - mark) / UI_ENCODER_DETENT_COUNTS;
        mark += d * UI_ENCODER_DETENT_COUNTS;
        return static_cast<int>(d);
    }

    // Wheel angle since begin(), for the chrome spoke. Off the raw count rather
    // than the detented value, so the spoke tracks the wheel 1:1 and moves
    // smoothly instead of stepping -- which is what makes it read as live.
    float angle(long count) const {
        return TWO_PI * static_cast<float>(count - origin) / static_cast<float>(ENC_CPR);
    }

    private:

    long mark   = 0;
    long origin = 0;
};

// One side lidar as a momentary button, against an adapting baseline.
class UIButton {
    public:

    void begin(uint16_t reading, unsigned long nowMs) {
        baseline   = reading;
        armed      = false;  // a clean release arms it; see update()
        below      = 0;
        driftSince = nowMs;
    }

    // True exactly once per press, on the edge.
    bool update(uint16_t reading, unsigned long nowMs) {
        const bool near = (reading + UI_BUTTON_PRESS_DELTA_MM) < baseline;

        if (!near) {
            below = 0;
            // Back at the baseline is a release, and a release is what arms the
            // button. Starting unarmed is what stops a hand already in front of
            // a sensor at boot from firing the first screen.
            if ((reading + UI_BUTTON_RELEASE_DELTA_MM) >= baseline) {
                armed      = true;
                driftSince = nowMs;
            }
            return false;
        }

        if (below < UI_BUTTON_DEBOUNCE_SAMPLES) ++below;

        if (armed && below >= UI_BUTTON_DEBOUNCE_SAMPLES) {
            armed      = false;  // needs a release before the next press
            driftSince = nowMs;
            return true;
        }

        // Near, but not consumed as a press -- either unarmed, or still
        // debouncing. If it has been like this past the adopt window it is not
        // a hand, it is the world: take it as the new baseline.
        if (nowMs - driftSince >= UI_BUTTON_BASELINE_ADOPT_MS) {
            baseline   = reading;
            below      = 0;
            armed      = true;
            driftSince = nowMs;
        }
        return false;
    }

    private:

    uint16_t baseline        = 0;
    unsigned long driftSince = 0;
    uint8_t below            = 0;
    bool armed               = false;
};
