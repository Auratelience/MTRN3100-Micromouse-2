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

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

// Which inputs are live on the current screen.
//
// Drawn every frame, so the panel always answers "what can I do here?" without
// the operator having to remember the sequence. Screen 1 sets leftButton false
// because back is a no-op there: the chrome and the behaviour agree, which is
// the whole point of drawing it.
struct UIChrome {
    bool leftButton;
    bool rightButton;
    bool leftDial;
    bool rightDial;
};

// A wheel, as a circle with one spoke.
inline void drawDial(Adafruit_SSD1306& g, int16_t cx, float angle) {
    g.drawCircle(cx, UI_DIAL_Y, UI_DIAL_RADIUS, SSD1306_WHITE);

    // World angle to screen: the spoke points up at zero and turns the way the
    // wheel does. Screen y grows downward, hence the negated cosine.
    const int16_t ex = cx + static_cast<int16_t>(lroundf(UI_DIAL_RADIUS * sinf(angle)));
    const int16_t ey =
        UI_DIAL_Y - static_cast<int16_t>(lroundf(UI_DIAL_RADIUS * cosf(angle)));
    g.drawLine(cx, UI_DIAL_Y, ex, ey, SSD1306_WHITE);
}

// A lidar button, as a semicircle on its edge.
//
// fillCircle centred on the edge pixel: GFX clips the off-panel half, so this
// is a semicircle bulging inward at no cost over a normal circle. A press
// flashes it to an outline for UI_BLINK_MS.
inline void drawButton(Adafruit_SSD1306& g, int16_t x, bool blink) {
    if (blink) {
        g.drawCircle(x, OLED_HEIGHT / 2, UI_BUTTON_RADIUS, SSD1306_WHITE);
    } else {
        g.fillCircle(x, OLED_HEIGHT / 2, UI_BUTTON_RADIUS, SSD1306_WHITE);
    }
}

// Chrome occupies x < 8, x > 119 and y > 46, leaving a content region of 18
// characters by 5 lines at OLED_CHAR_WIDTH / OLED_TEXT_HEIGHT.
inline void drawChrome(
    OLEDDisplay& display, const UIChrome& chrome, float leftAngle, float rightAngle,
    bool leftBlink, bool rightBlink
) {
    Adafruit_SSD1306& g = display.gfx();

    if (chrome.leftButton) drawButton(g, 0, leftBlink);
    if (chrome.rightButton) drawButton(g, OLED_WIDTH - 1, rightBlink);
    if (chrome.leftDial) drawDial(g, UI_DIAL_LEFT_X, leftAngle);
    if (chrome.rightDial) drawDial(g, UI_DIAL_RIGHT_X, rightAngle);
}

// NOLINTEND(misc-definitions-in-headers)
