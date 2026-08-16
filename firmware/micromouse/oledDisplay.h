// SSD1306 ownership, and the mm -> px projection its map renderers share
//
// Zimmy Levi z5587840

#pragma once

#include <math.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "constants.h"

// The one owner of the panel.
//
// There is exactly one SSD1306 on the bus, so there is exactly one of these.
// Renderers (OLEDValues, OLEDMap, OLEDPath) borrow it by reference and draw
// through gfx(); none of them owns a framebuffer, calls begin(), or holds the
// refresh throttle. Three owners would mean three 1 kB framebuffers malloc'd
// for the same panel and three begin() calls on address 0x3C.
class OLEDDisplay {
    public:

    explicit OLEDDisplay(
        uint8_t width   = OLED_WIDTH,
        uint8_t height  = OLED_HEIGHT,
        uint8_t address = OLED_ADDRESS,
        int8_t resetPin = OLED_NO_RESET_PIN
    ) :
        display(width, height, &Wire, resetPin),
        w(width),
        h(height),
        address(address) {}

    bool init() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, address)) {
            initialized = false;
            return false;
        }

        display.clearDisplay();
        display.setTextSize(OLED_TEXT_SIZE);
        display.setTextColor(SSD1306_WHITE);
        display.cp437(true);
        display.display();
        initialized = true;
        return true;
    }

    bool ready() const {
        return initialized;
    }

    void clear() {
        if (!initialized) return;
        display.clearDisplay();
        display.display();
    }

    // True at most once per OLED_REFRESH_MS, and false if the panel never came
    // up. Pushing a frame is ~1 kB over I2C, which is milliseconds the control
    // loop does not have every tick.
    //
    // Consuming: the first caller in a refresh window takes it. That is only
    // correct because exactly one renderer draws at a time -- the sketch
    // selects one per state. Drawing two in the same tick would silently starve
    // whichever asked second.
    bool due() {
        if (!initialized) return false;
        const unsigned long now = millis();
        if (now - lastRefreshMs < OLED_REFRESH_MS) return false;
        lastRefreshMs = now;
        return true;
    }

    Adafruit_SSD1306& gfx() {
        return display;
    }

    uint8_t width() const {
        return w;
    }

    uint8_t height() const {
        return h;
    }

    private:

    Adafruit_SSD1306 display;
    uint8_t w;
    uint8_t h;
    uint8_t address;
    bool initialized            = false;
    unsigned long lastRefreshMs = 0;
};

// World millimetres to panel pixels, for both map renderers.
//
// The robot frame is x forward and y left (see types.h); the screen is x right
// and y down. So world x runs up the screen and world y runs left across it:
//
//     screenX = ox - k * (worldY - cy)
//     screenY = oy - k * (worldX - cx)
//
// One scale k for both axes, taken as the tighter of the two fits, so a maze
// is letterboxed rather than stretched and a cell stays square. Both renderers
// project through this, so a cell is the same size on screen in either.
class OLEDView {
    public:

    // Fits the world box into the pixel rect, centred. False -- and every
    // projection after it meaningless -- if either span or either pixel extent
    // is non-positive, since k would divide by zero. Callers gate on this
    // rather than drawing garbage.
    bool fit(
        float minX, float maxX, float minY, float maxY, int16_t px, int16_t py, int16_t pw,
        int16_t ph
    ) {
        k                 = 0.0f;
        const float spanX = maxX - minX;
        const float spanY = maxY - minY;
        if (!(spanX > 0.0f) || !(spanY > 0.0f) || pw <= 0 || ph <= 0) return false;

        // Screen width carries world y, screen height carries world x.
        const float kx = static_cast<float>(pw) / spanY;
        const float ky = static_cast<float>(ph) / spanX;

        k  = (kx < ky) ? kx : ky;
        cx = 0.5f * (minX + maxX);
        cy = 0.5f * (minY + maxY);
        ox = static_cast<float>(px) + 0.5f * static_cast<float>(pw);
        oy = static_cast<float>(py) + 0.5f * static_cast<float>(ph);
        return true;
    }

    bool valid() const {
        return k > 0.0f;
    }

    int16_t sx(float worldX, float worldY) const {
        (void)worldX; // screen x depends on world y alone; taken for symmetry
        return static_cast<int16_t>(lroundf(ox - k * (worldY - cy)));
    }

    int16_t sy(float worldX, float worldY) const {
        (void)worldY;
        return static_cast<int16_t>(lroundf(oy - k * (worldX - cx)));
    }

    // Pixels per millimetre. Renderers use it to size a circle or a tick.
    float scale() const {
        return k;
    }

    private:

    float k  = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float ox = 0.0f;
    float oy = 0.0f;
};

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

// A progress delegate onto [0, 1].
//
// Never trust the delegate: it divides by a count that can be zero or can be
// exceeded, so it can hand back a NaN or a value off either end. Both the bar
// and the percentage readout run it, and a bar that clamped while the number
// beside it did not would disagree on screen.
inline float clampFraction(float fraction) {
    if (!isfinite(fraction)) return 0.0f;
    if (fraction < 0.0f) return 0.0f;
    if (fraction > 1.0f) return 1.0f;
    return fraction;
}

// The completion meter both map renderers carry, along the bottom of the text
// pane: a one-pixel outline with a fill proportional to `fraction`. An
// out-of-range fill would run the rectangle off the panel, hence the clamp.
inline void drawProgressBar(OLEDDisplay& display, float fraction) {
    fraction = clampFraction(fraction);

    const int16_t x = OLED_TEXT_PANE_X;
    const int16_t y = static_cast<int16_t>(display.height() - OLED_BAR_H - OLED_BAR_MARGIN);
    const int16_t w = static_cast<int16_t>(display.width() - OLED_TEXT_PANE_X - OLED_BAR_MARGIN);
    if (w <= 2) return;

    Adafruit_SSD1306& g = display.gfx();
    g.drawRect(x, y, w, OLED_BAR_H, SSD1306_WHITE);

    const int16_t inner = static_cast<int16_t>(w - 2);
    const int16_t fill  = static_cast<int16_t>(lroundf(fraction * static_cast<float>(inner)));
    if (fill > 0) g.fillRect(x + 1, y + 1, fill, OLED_BAR_H - 2, SSD1306_WHITE);
}

// NOLINTEND(misc-definitions-in-headers)
