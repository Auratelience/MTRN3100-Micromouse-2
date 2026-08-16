// Live maze-exploration display
//
// Zimmy Levi z5587840

#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>

#include "constants.h"
#include "mazeMapper.h"
#include "oledDisplay.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// What the mapper currently believes, drawn every refresh: walls, visited
// cells, where the robot is and which way it faces, the goal, and a progress
// meter.
//
// Read-only against MazeMapper -- it holds a const reference and touches
// nothing but accessors, so nothing here can perturb an exploration run.
//
// The grid is drawn on a cell lattice rather than through OLEDView, because
// that is what the mapper's state is: cells and boundary bits, with no
// millimetres anywhere in the class. OLEDPath is the geometric one.
template <size_t N>
class OLEDMap {
    // One pixel of the pitch is the far boundary line, hence the -1. Below
    // three the robot marker has no interior left to fill and the heading
    // pixel would land on a wall line, so fail the build rather than draw
    // something unreadable.
    static constexpr int16_t PITCH = (OLED_MAP_PANE_W - 1) / static_cast<int16_t>(N);
    static_assert(PITCH >= 3, "OLED_MAP_PANE_W too small to draw N cells");

    static constexpr int16_t GRID = PITCH * static_cast<int16_t>(N) + 1;

    public:

    OLEDMap(
        OLEDDisplay& display,
        const MazeMapper<N>& mapper,
        etl::delegate<float()> progress = etl::delegate<float()>()
    ) :
        display(display),
        mapper(mapper),
        progress(progress) {}

    void setProgress(etl::delegate<float()> p) {
        progress = p;
    }

    void update() {
        if (!display.ready()) return;
        if (!display.due()) return;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        drawGrid();
        drawRobot();
        drawGoal();
        drawText();

        if (progress.is_valid()) drawProgressBar(display, progress());

        g.display();
    }

    private:

    using Cell = typename MazeMapper<N>::Cell;

    OLEDDisplay& display;
    const MazeMapper<N>& mapper;
    etl::delegate<float()> progress;

    // Grid origin, centring the lattice in the map pane. The pitch is integer,
    // so N cells rarely fill the pane exactly (61 px of 64 at N = 10) and the
    // remainder is split either side.
    static constexpr int16_t originX() {
        return OLED_MAP_PANE_X + (OLED_MAP_PANE_W - GRID) / 2;
    }

    static constexpr int16_t originY() {
        return (static_cast<int16_t>(OLED_HEIGHT) - GRID) / 2;
    }

    // Top-left pixel of cell (x, y). y increases left across the screen and x
    // increases up it, matching the frame in types.h, so both are counted back
    // from N - 1.
    static int16_t cellLeft(int8_t y) {
        return originX() + (static_cast<int16_t>(N) - 1 - y) * PITCH;
    }

    static int16_t cellTop(int8_t x) {
        return originY() + (static_cast<int16_t>(N) - 1 - x) * PITCH;
    }

    // Every set wall bit as its boundary line, plus a centre pixel per visited
    // cell.
    //
    // All four sides of every cell are drawn, so a shared boundary is drawn
    // from both cells. On a 1 bpp buffer that is idempotent, and cheaper than
    // the bookkeeping needed to visit each boundary once.
    void drawGrid() {
        Adafruit_SSD1306& g = display.gfx();

        for (int8_t x = 0; x < static_cast<int8_t>(N); ++x) {
            for (int8_t y = 0; y < static_cast<int8_t>(N); ++y) {
                const Cell c          = Cell{x, y};
                const int16_t left    = cellLeft(y);
                const int16_t top     = cellTop(x);
                const int16_t right   = left + PITCH;
                const int16_t bottom  = top + PITCH;
                const int16_t span    = PITCH + 1;

                if (mapper.hasWall(c, North)) g.drawFastHLine(left, top, span, SSD1306_WHITE);
                if (mapper.hasWall(c, South)) g.drawFastHLine(left, bottom, span, SSD1306_WHITE);
                if (mapper.hasWall(c, West)) g.drawFastVLine(left, top, span, SSD1306_WHITE);
                if (mapper.hasWall(c, East)) g.drawFastVLine(right, top, span, SSD1306_WHITE);

                if (mapper.visited(c)) {
                    g.drawPixel(left + PITCH / 2, top + PITCH / 2, SSD1306_WHITE);
                } else if (mapper.sealedCell(c)) {
                    // Crossed out: a cell the search can never enter. Before
                    // the run that is the chamfered corners, which the sketch
                    // seals as priors; during it, anywhere the discovered
                    // walls have closed off. Four wall lines alone already
                    // draw a box, but a box is what an ordinary cell in a
                    // tight corridor looks like too.
                    g.drawLine(left + 1, top + 1, right - 1, bottom - 1, SSD1306_WHITE);
                    g.drawLine(left + 1, bottom - 1, right - 1, top + 1, SSD1306_WHITE);
                }
            }
        }
    }

    // The current cell filled solid, with one pixel knocked back out at the
    // inside edge of the heading direction. Inside the fill rather than on the
    // boundary, so the marker never erases a wall line.
    void drawRobot() {
        if (!mapper.ready()) return;

        Adafruit_SSD1306& g = display.gfx();
        const Cell c        = mapper.position();
        const int16_t left  = cellLeft(c.y);
        const int16_t top   = cellTop(c.x);

        g.fillRect(left + 1, top + 1, PITCH - 1, PITCH - 1, SSD1306_WHITE);

        const int16_t mid  = PITCH / 2;
        const int16_t last = PITCH - 1;
        int16_t hx         = left + mid;
        int16_t hy         = top + mid;
        switch (mapper.heading()) {
            case North: hy = top + 1; break;
            case South: hy = top + last; break;
            case West:  hx = left + 1; break;
            case East:  hx = left + last; break;
        }
        g.drawPixel(hx, hy, SSD1306_BLACK);
    }

    // Four corner pixels, so the goal does not read as a wall. Drawn last and
    // inverted, so it stays visible when the robot is standing on it.
    void drawGoal() {
        if (!mapper.ready()) return;

        Adafruit_SSD1306& g = display.gfx();
        const Cell c        = mapper.goalPosition();
        const int16_t left  = cellLeft(c.y);
        const int16_t top   = cellTop(c.x);
        const int16_t last  = PITCH - 1;

        g.drawPixel(left + 1, top + 1, SSD1306_INVERSE);
        g.drawPixel(left + last, top + 1, SSD1306_INVERSE);
        g.drawPixel(left + 1, top + last, SSD1306_INVERSE);
        g.drawPixel(left + last, top + last, SSD1306_INVERSE);
    }

    void drawText() {
        Adafruit_SSD1306& g = display.gfx();
        const Cell p        = mapper.position();
        const Cell goal     = mapper.goalPosition();

        // FAULT is kept distinct from DONE deliberately. mazeMapper.h raises
        // doneExploring() when it faults too, so a display that folded them
        // together would show a run that gave up as a finished sweep -- which
        // is exactly the failure this screen exists to catch.
        g.setCursor(OLED_TEXT_PANE_X, 0);
        if (!mapper.ready()) g.print(F("IDLE"));
        else if (mapper.faulted()) g.print(F("FAULT"));
        else if (mapper.doneExploring()) g.print(F("DONE"));
        else g.print(F("EXPL"));

        g.setCursor(OLED_TEXT_PANE_X, OLED_TEXT_HEIGHT);
        g.print(F("P "));
        g.print(p.x);
        g.print(',');
        g.print(p.y);

        g.setCursor(OLED_TEXT_PANE_X, 2 * OLED_TEXT_HEIGHT);
        g.print(F("H "));
        g.print(directionChar(mapper.heading()));

        g.setCursor(OLED_TEXT_PANE_X, 3 * OLED_TEXT_HEIGHT);
        g.print(F("G "));
        g.print(goal.x);
        g.print(',');
        g.print(goal.y);

        g.setCursor(OLED_TEXT_PANE_X, 4 * OLED_TEXT_HEIGHT);
        g.print(F("V "));
        g.print(mapper.visitedCount());
    }
};

#pragma GCC pop_options
