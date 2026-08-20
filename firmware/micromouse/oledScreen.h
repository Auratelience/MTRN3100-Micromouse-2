// The run display: a map on the left, the numbers describing the run on the right
//
// Zimmy Levi z5587840
// Stephen Gottlieb z5481352

#pragma once

#include <math.h>

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include <Embedded_Template_Library.h>
#include <etl/delegate.h>
#include <etl/span.h>

#include "constants.h"
#include "oledDisplay.h"
#include "types.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

// The one readout in the values pane whose meaning changes with the phase of
// the run: cells explored during a sweep, distance along a route while driving
// one. The label travels with the number precisely so that it can change --
// a bare percentage that silently switched from one to the other would be
// unreadable, and two rows, one of them always blank, would waste a third of
// the pane.
//
// A '\0' label leaves the row empty, which is what a phase with nothing
// meaningful to report should say rather than printing a stale figure.
struct OLEDMetric {
    char label     = '\0';
    float fraction = 0.0f;
};

// One screen for the run, in two panes:
//
//     +---------- 64 px ----------+--- 62 px ---+
//     | walls as lines, obstacles | EXPL        |  mode
//     | as filled circles, the    | X    340    |  mm
//     | route as a polyline, the  | Y    128    |  mm
//     | robot as a dot and a tick | T    -87    |  deg
//     |                           | E    42%    |  metric
//     +---------------------------+-------------+
//
// Templated on the map type rather than on an obstacle count, so it draws
// either the exported Map<S> -- constexpr, in flash, costing no RAM -- or
// MazeWallMap, the walls the robot has discovered for itself. It needs only
// size(), present() and operator[]; whether a slot is stored or derived is the
// map's business, and a discovered map simply grows walls as the run goes on.
//
// The map is drawn as geometry and not as a cell lattice. That is deliberate
// for both map types, though for different reasons: maze_map.h is fitted from a
// photograph, so it carries five free-standing cylinders and panels a degree or
// two off the lattice, none of which survives being snapped to a grid; and
// MazeWallMap is what the lidar localises against, so drawing it is drawing the
// map the fix was taken from rather than a second rendering of the same belief
// that could disagree with it.
//
// Everything the screen shows arrives through a delegate. It therefore knows
// nothing about MotionPlanner, MazeRunner or MazeMapper, which is what lets one
// class serve a run that races a precomputed route and a run that discovers
// its own.
template <typename MapT>
class OLEDScreen {
    public:

    OLEDScreen(
        OLEDDisplay& display,
        const MapT& map,
        etl::delegate<Pose()> pose,
        etl::delegate<const char*()> mode,
        etl::delegate<OLEDMetric()> metric = etl::delegate<OLEDMetric()>()
    ) :
        display(display),
        map(map),
        pose(pose),
        mode(mode),
        metric(metric) {}

    // Fits the map into the map pane, through mapBounds -- every present slot
    // as its bounding circle, so a panel is bounded by the circle containing it
    // and nothing is clipped.
    //
    // False if the map holds nothing or the fitted box is flat on either axis
    // (a single obstacle, or a row of them), because OLEDView would then divide
    // by a zero span. The values pane still draws in that case; see update().
    //
    // Call it whenever the map's extent can have changed. For the exported map
    // that is once, in setup(); for a discovered one, once the perimeter is
    // seeded -- MazeMapper::begin() does that, and every wall found afterwards
    // falls inside it, so one fit holds for the whole run there too.
    bool init() {
        const MapBounds bounds = mapBounds(map);
        if (!bounds.valid()) return false;

        return view.fit(
            bounds.minX, bounds.maxX, bounds.minY, bounds.maxY, OLED_MAP_PANE_X, 0,
            OLED_MAP_PANE_W, static_cast<int16_t>(OLED_HEIGHT)
        );
    }

    // The planned route, in millimetres, drawn as a polyline. Optional: the map
    // and the robot draw without it, and a route shorter than two points draws
    // nothing, so a caller may hand over an empty span every frame until the
    // route exists rather than tracking whether it does.
    //
    // Millimetres and not cells, so this class stays independent of MazeMapper
    // and of the maze size. The caller materialises the cells.
    void setRoute(etl::span<const Vec2D> points) {
        route = points;
    }

    void update() {
        if (!display.ready()) return;
        if (!display.due()) return;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        // The values draw whether or not the map fitted. A failed fit is a
        // map that could not be scaled, not a robot that has stopped having a
        // pose, and the pose readout is the only one the machine has -- so a
        // half-drawn screen beats the blank one that gating the whole frame on
        // view.valid() would give.
        if (view.valid()) {
            drawMap();
            drawRoute();
            drawRobot();
        }
        drawValues();

        g.display();
    }

    private:

    OLEDDisplay& display;
    const MapT& map;
    etl::delegate<Pose()> pose;
    etl::delegate<const char*()> mode;
    etl::delegate<OLEDMetric()> metric;
    etl::span<const Vec2D> route;
    OLEDView view;

    // Panels as lines between their two ends, everything else as a filled disc.
    //
    // The shape question goes through Obstacle rather than through the variant
    // directly: panelEnds() answers "is this a wall, and if so where does it
    // run" in one call, which is the whole of what drawing one needs.
    void drawMap() {
        Adafruit_SSD1306& g = display.gfx();

        for (size_t i = 0; i < map.size(); ++i) {
            if (!map.present(i)) continue;
            const Obstacle o = map[i];

            Vec2D a;
            Vec2D b;
            if (o.panelEnds(a, b)) {
                g.drawLine(
                    view.sx(a.x, a.y), view.sy(a.x, a.y), view.sx(b.x, b.y), view.sy(b.x, b.y),
                    SSD1306_WHITE
                );
                continue;
            }

            const int16_t px = view.sx(o.centre.x, o.centre.y);
            const int16_t py = view.sy(o.centre.x, o.centre.y);
            const int16_t r  = static_cast<int16_t>(lroundf(view.scale() * o.radius()));

            // A 12 mm post is a third of a pixel at this scale, and fillCircle
            // with r = 0 draws nothing, so fall back to the pixel itself. The
            // cylinders are the ones this rounds up to a visible disc, which is
            // the distinction worth keeping: a post sits on the lattice with a
            // panel either side, a cylinder stands in open floor.
            if (r < 1) g.drawPixel(px, py, SSD1306_WHITE);
            else g.fillCircle(px, py, r, SSD1306_WHITE);
        }
    }

    void drawRoute() {
        if (route.size() < 2) return;

        Adafruit_SSD1306& g = display.gfx();
        for (size_t i = 1; i < route.size(); ++i) {
            const Vec2D& a = route[i - 1];
            const Vec2D& b = route[i];
            g.drawLine(
                view.sx(a.x, a.y), view.sy(a.x, a.y), view.sx(b.x, b.y), view.sy(b.x, b.y),
                SSD1306_WHITE
            );
        }
    }

    // A dot at the pose plus a tick along the heading. Both are skipped if they
    // project outside the map pane: the estimate can wander off the map, and
    // Adafruit_GFX clips to the panel, not to our pane, so an off-map dot would
    // be drawn over the values.
    void drawRobot() {
        if (!pose.is_valid()) return;

        const Pose p     = pose();
        const int16_t px = view.sx(p.x, p.y);
        const int16_t py = view.sy(p.x, p.y);
        if (!inPane(px, py)) return;

        // Most of a cell long. The dot alone is three pixels across and the
        // obstacles are filled discs now too, so the tick is what identifies
        // the robot as well as which way it points -- at ~10 px to the cell on
        // a discovered 6x6 map, and ~6 on the whole exported deck, a half-cell
        // tick was too short to read as either.
        const float tick = 0.8f * MAZE_CELL_SIZE;
        const float tx   = p.x + tick * trig::xcos(p.theta);
        const float ty   = p.y + tick * trig::xsin(p.theta);

        Adafruit_SSD1306& g = display.gfx();
        const int16_t ex    = view.sx(tx, ty);
        const int16_t ey    = view.sy(tx, ty);
        if (inPane(ex, ey)) g.drawLine(px, py, ex, ey, SSD1306_WHITE);
        g.fillCircle(px, py, 1, SSD1306_WHITE);
    }

    static bool inPane(int16_t px, int16_t py) {
        return px >= OLED_MAP_PANE_X && px < OLED_MAP_PANE_X + OLED_MAP_PANE_W && py >= 0 &&
               py < static_cast<int16_t>(OLED_HEIGHT);
    }

    // Where a row's number starts, so the digits line up down the pane whatever
    // the labels are.
    static int16_t valueX() {
        return static_cast<int16_t>(OLED_TEXT_PANE_X + (OLED_VALUE_COLUMN * OLED_CHAR_WIDTH));
    }

    // Five of the eight rows: the phase, the pose, and the one metric that
    // phase makes sense of. Rows five to seven are left clear.
    void drawValues() {
        Adafruit_SSD1306& g = display.gfx();

        g.setCursor(OLED_TEXT_PANE_X, 0);
        const char* label = mode.is_valid() ? mode() : nullptr;
        g.print(label == nullptr ? "?" : label);

        // Whole millimetres and whole degrees. A tenth of a millimetre is two
        // orders of magnitude below what the fix is good for, and the decimal
        // point would cost a character of a ten-character pane.
        if (pose.is_valid()) {
            const Pose p = pose();
            printRow(1, 'X', p.x);
            printRow(2, 'Y', p.y);
            printRow(3, 'T', wrapAngle(p.theta) * 180.0f / PI);
        }

        if (!metric.is_valid()) return;
        const OLEDMetric m = metric();
        if (m.label == '\0') return;

        const int16_t y = 4 * OLED_TEXT_HEIGHT;
        g.setCursor(OLED_TEXT_PANE_X, y);
        g.print(m.label);
        g.setCursor(valueX(), y);
        g.print(static_cast<int>(lroundf(clampFraction(m.fraction) * 100.0f)));
        g.print('%');
    }

    void printRow(uint8_t row, char label, float value) {
        Adafruit_SSD1306& g = display.gfx();
        const int16_t y     = static_cast<int16_t>(row * OLED_TEXT_HEIGHT);

        g.setCursor(OLED_TEXT_PANE_X, y);
        g.print(label);
        g.setCursor(valueX(), y);
        g.print(value, 0);
    }
};

#pragma GCC pop_options
