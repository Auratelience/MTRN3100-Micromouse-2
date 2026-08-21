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
//
// A press is near-side only -- a hand in front of a sensor can shorten the
// range but never lengthen it -- while drift is two-sided, because the world
// moving away is exactly as much a change of baseline as the world moving
// closer. A one-sided adopt rule would be a ratchet: the baseline could follow
// a hand down to whatever it was resting at and never climb back once the hand
// left, and at MIN_DIST it could never move again, killing the button until
// the next power cycle.
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
        // Ordered subtraction: the distance is unsigned, so taking it the
        // wrong way round wraps to ~65000 rather than coming out negative.
        const uint16_t away =
            (reading > baseline) ? (reading - baseline) : (baseline - reading);

        const bool far  = away > UI_BUTTON_PRESS_DELTA_MM;
        const bool near = far && reading < baseline;

        if (!far) {
            below = 0;

            // Within PRESS_DELTA of the baseline is the world as the button
            // knows it, so the adopt window starts again from here.
            driftSince = nowMs;

            // Back at the baseline is a release, and a release is what arms the
            // button. Starting unarmed is what stops a hand already in front of
            // a sensor at boot from firing the first screen.
            if (away <= UI_BUTTON_RELEASE_DELTA_MM) armed = true;
            return false;
        }

        if (!near) {
            below = 0;  // the far side is never a press, only ever drift
        } else if (below < UI_BUTTON_DEBOUNCE_SAMPLES) {
            ++below;
        }

        if (armed && below >= UI_BUTTON_DEBOUNCE_SAMPLES) {
            armed      = false;  // needs a release before the next press
            driftSince = nowMs;
            return true;
        }

        // Past PRESS_DELTA and not consumed as a press -- unarmed, still
        // debouncing, or on the far side, where there is no press to consume.
        // If it has been like this past the adopt window it is not a hand, it
        // is the world: take it as the new baseline.
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

// Direction is not an integer dial: the enum is {North = 0, West = 1,
// South = 2, East = -1}, so incrementing it does not walk the compass. This is
// the clockwise order the heading screen indexes.
constexpr Direction UI_HEADING_ORDER[4] = {North, East, South, West};

inline const char* headingName(Direction d) {
    switch (d) {
        case North: return "N";
        case East:  return "E";
        case South: return "S";
        default:    return d == West ? "W" : "?";
    }
}

// MazeMapper::sameCell is a private static member, so it is not reachable from
// here and this is not worth widening the mapper's interface for.
inline bool sameCellUI(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }

inline int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// The wizard's screens, in order.
enum class UIStep : uint8_t { Size, Start, Heading, Goal, Countdown, Done };

inline RunConfig runStartupUI(
    OLEDDisplay& display, LIDAR& lidar, BaseMotor& left, BaseMotor& right,
    I2CRepairer& i2c
) {
    RunConfig cfg{MAZE_SIZE_MAX, Cell{0, 0}, North, Cell{0, 0}};
    uint8_t headingIndex = 0;

    UIDial leftDial;
    UIDial rightDial;
    UIButton backButton;
    UIButton nextButton;

    lidar.update();
    const unsigned long t0 = millis();
    backButton.begin(lidar.getReading(LIDAR::Left), t0);
    nextButton.begin(lidar.getReading(LIDAR::Right), t0);
    leftDial.begin(left.count());
    rightDial.begin(right.count());

    UIStep step                 = UIStep::Size;
    UIStep shownStep             = UIStep::Size;
    unsigned long stepEnteredMs = t0;
    unsigned long leftBlinkMs   = 0;
    unsigned long rightBlinkMs  = 0;

    while (step != UIStep::Done) {
        i2c.update();
        lidar.update();

        const unsigned long now = millis();
        const long lc           = left.count();
        const long rc           = right.count();

        // Chrome for this screen, which is also the gate on the inputs: a
        // button that is not drawn is not read, so the two cannot disagree.
        UIChrome chrome{false, false, false, false};
        switch (step) {
            case UIStep::Size:
                chrome = UIChrome{false, true, false, true};
                break;
            case UIStep::Start:
            case UIStep::Goal:
                chrome = UIChrome{true, true, true, true};
                break;
            case UIStep::Heading:
                chrome = UIChrome{true, true, false, true};
                break;
            case UIStep::Countdown:
                chrome = UIChrome{
                    UI_COUNTDOWN_BACK_ENABLED, UI_COUNTDOWN_SKIP_ENABLED, false, false
                };
                break;
            default: break;
        }

        // Read the dials every frame regardless, so the marks track the wheels
        // and a screen that does not use a dial cannot bank detents for the
        // next one that does.
        const int dl = leftDial.take(lc);
        const int dr = rightDial.take(rc);

        const bool back =
            backButton.update(lidar.getReading(LIDAR::Left), now) && chrome.leftButton;
        const bool next =
            nextButton.update(lidar.getReading(LIDAR::Right), now) && chrome.rightButton;

        if (back) leftBlinkMs = now;
        if (next) rightBlinkMs = now;

        const int last = static_cast<int>(cfg.size) - 1;

        switch (step) {
            case UIStep::Size:
                if (dr != 0) {
                    cfg.size = static_cast<uint8_t>(
                        clampInt(static_cast<int>(cfg.size) + dr, MAZE_SIZE_MIN, MAZE_SIZE_MAX)
                    );
                    // A smaller maze can strand a cell chosen earlier.
                    cfg.start.x = static_cast<int8_t>(clampInt(cfg.start.x, 0, cfg.size - 1));
                    cfg.start.y = static_cast<int8_t>(clampInt(cfg.start.y, 0, cfg.size - 1));
                    cfg.goal.x  = static_cast<int8_t>(clampInt(cfg.goal.x, 0, cfg.size - 1));
                    cfg.goal.y  = static_cast<int8_t>(clampInt(cfg.goal.y, 0, cfg.size - 1));
                }
                if (next) step = UIStep::Start;
                break;

            case UIStep::Start:
                if (dl != 0) cfg.start.x = static_cast<int8_t>(clampInt(cfg.start.x + dl, 0, last));
                if (dr != 0) cfg.start.y = static_cast<int8_t>(clampInt(cfg.start.y + dr, 0, last));
                if (next) step = UIStep::Heading;
                if (back) step = UIStep::Size;
                break;

            case UIStep::Heading:
                if (dr != 0) {
                    headingIndex = static_cast<uint8_t>(((headingIndex + dr) % 4 + 4) % 4);
                    cfg.heading  = UI_HEADING_ORDER[headingIndex];
                }
                if (next) step = UIStep::Goal;
                if (back) step = UIStep::Start;
                break;

            case UIStep::Goal:
                if (dl != 0) cfg.goal.x = static_cast<int8_t>(clampInt(cfg.goal.x + dl, 0, last));
                if (dr != 0) cfg.goal.y = static_cast<int8_t>(clampInt(cfg.goal.y + dr, 0, last));
                if (back) step = UIStep::Heading;
                if (next && !sameCellUI(cfg.start, cfg.goal)) step = UIStep::Countdown;
                break;

            case UIStep::Countdown:
                if (back) step = UIStep::Goal;
                if (next || now - stepEnteredMs >= UI_COUNTDOWN_MS) step = UIStep::Done;
                break;

            default: break;
        }

        if (step == UIStep::Done) break;

        // Entering a screen restarts its clock, which is what the countdown
        // measures against. A plain local, not a function-local static: a
        // static would survive into a second call and start the countdown
        // already expired.
        if (step != shownStep) {
            shownStep     = step;
            stepEnteredMs = now;
        }

        if (!display.due()) continue;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        switch (step) {
            case UIStep::Size:
                g.setCursor(12, 4);
                g.print(F("MAZE SIZE"));
                g.setCursor(12, 20);
                g.print(cfg.size);
                g.print(F(" x "));
                g.print(cfg.size);
                break;
            case UIStep::Start:
                g.setCursor(12, 4);
                g.print(F("START CELL"));
                g.setCursor(12, 20);
                g.print(F("X:"));
                g.print(cfg.start.x);
                g.print(F("  Y:"));
                g.print(cfg.start.y);
                break;
            case UIStep::Heading:
                g.setCursor(12, 4);
                g.print(F("START HEADING"));
                g.setCursor(12, 20);
                g.print(headingName(cfg.heading));
                break;
            case UIStep::Goal:
                g.setCursor(12, 4);
                g.print(F("GOAL CELL"));
                g.setCursor(12, 20);
                g.print(F("X:"));
                g.print(cfg.goal.x);
                g.print(F("  Y:"));
                g.print(cfg.goal.y);
                if (sameCellUI(cfg.start, cfg.goal)) {
                    g.setCursor(12, 32);
                    g.print(F("= START"));
                }
                break;
            case UIStep::Countdown:
                g.setCursor(12, 4);
                g.print(F("STARTING IN"));
                g.setCursor(12, 20);
                g.print((UI_COUNTDOWN_MS - (now - stepEnteredMs) + 999) / 1000);
                break;
            default: break;
        }

        drawChrome(
            display, chrome, leftDial.angle(lc), rightDial.angle(rc),
            now - leftBlinkMs < UI_BLINK_MS, now - rightBlinkMs < UI_BLINK_MS
        );
        g.display();
    }

    return cfg;
}

// NOLINTEND(misc-definitions-in-headers)
