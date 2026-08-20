// The bring-up splash: the logo, blitted once before the run display exists
//
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "constants.h"
#include "oledDisplay.h"
#include "splash_screen.h"

// splash_screen.h is generated from a PNG that is only as big as whoever
// exported it made it, and drawBitmap clips silently rather than complaining --
// so a resized source would show up as art quietly cropped on the panel rather
// than as anything a build would catch. Caught here instead.
static_assert(
    SPLASH_WIDTH == OLED_WIDTH, "splash bitmap is not the width of the panel; re-export the PNG"
);
static_assert(
    SPLASH_HEIGHT == OLED_HEIGHT, "splash bitmap is not the height of the panel; re-export the PNG"
);

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

// The logo, pushed to the panel immediately.
//
// For setup() only, in place of a display.clear(): it holds through the rest of
// bring-up and the first OLEDScreen frame in loop() overwrites it. So there is
// no splash state to leave, nothing in loop() to gate, and the run display is
// reached by the run simply starting. Hold it longer with a delay() after the
// call if bring-up turns out to be quicker than the logo deserves.
//
// Which makes *where* it is called the only thing keeping it on screen, and the
// OLED block in micromouse.ino sits directly under the I2C bring-up for that
// reason alone. It was written at the end of setup() first, and the logo lasted
// under ~10 ms there: the IMU and the lidar are where bring-up actually spends
// its time, and both had already run, leaving only two Serial prints and
// runBegin() before loop() drew over it. Nothing here can fix that from the
// inside -- a splash lasts exactly as long as the work below it.
//
// Deliberately not gated on display.due(): that throttle is consuming and only
// opens once per OLED_REFRESH_MS, so a single frame asking through it would be
// swallowed outright whenever bring-up had already drawn inside the window. The
// throttle exists to keep the *control loop* from spending milliseconds on I2C,
// which is not a cost setup() has to care about.
//
// The same goes for the blit itself: Adafruit_GFX walks the bitmap a pixel at a
// time, so this is 8192 drawPixel calls. That is a few milliseconds, invisible
// here, and the reason this is a bring-up screen rather than something loop()
// could render per frame.
inline void drawSplash(OLEDDisplay& display) {
    if (!display.ready()) return;

    Adafruit_SSD1306& g = display.gfx();

    // drawBitmap paints set bits only and leaves clear ones untouched, so the
    // black field around the logo is this clear rather than anything in the
    // bitmap.
    g.clearDisplay();
    g.drawBitmap(
        0, 0, SPLASH_BITMAP, static_cast<int16_t>(SPLASH_WIDTH),
        static_cast<int16_t>(SPLASH_HEIGHT), SSD1306_WHITE
    );
    g.display();
}

// NOLINTEND(misc-definitions-in-headers)
