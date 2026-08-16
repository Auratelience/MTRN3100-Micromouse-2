// Path-execution display: a known map, the robot on it, and the route
//
// Zimmy Levi z5587840

#pragma once

#include <math.h>
#include <variant>

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

// A map drawn as geometry rather than as a grid: every panel as a line at its
// own angle, every post and cylinder as a circle. That is deliberate --
// maze_map.h is fitted from a photograph, so it carries five free-standing
// cylinders and panels that are a degree or two off the lattice, none of which
// survives being snapped to a cell grid.
//
// Templated on the map type rather than on an obstacle count, so it draws
// either the exported Map<S> -- constexpr, in flash, costing no RAM -- or
// MazeWallMap, the walls the robot has discovered for itself. It needs only
// size(), present() and operator[]; whether a slot is stored or derived is the
// map's business.
template <typename MapT>
class OLEDPath {
    public:

    OLEDPath(
        OLEDDisplay& display,
        const MapT& map,
        etl::delegate<Pose()> pose,
        etl::delegate<float()> progress = etl::delegate<float()>()
    ) :
        display(display),
        map(map),
        pose(pose),
        progress(progress) {}

    // Fits the map into the map pane. Walks every obstacle once, taking
    // centre +/- boundingRadius, so a panel is bounded by the circle that
    // contains it and nothing is clipped.
    //
    // False if the map holds nothing or the fitted box is flat on either axis
    // -- a single obstacle, or a row of them -- because OLEDView would then
    // divide by a zero span. update() draws nothing until this succeeds.
    //
    // Call it whenever the map's extent can have changed. For an exported map
    // that is once, in setup(); for a discovered one it is worth re-fitting as
    // walls appear, or the first few cells fill the whole panel.
    bool init() {
        bool any   = false;
        float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;

        for (size_t i = 0; i < map.size(); ++i) {
            if (!map.present(i)) continue;
            const Obstacle o = map[i];
            const float r    = o.boundingRadius();
            const float x    = o.centre.x;
            const float y    = o.centre.y;
            if (!any) {
                minX = x - r; maxX = x + r; minY = y - r; maxY = y + r;
                any  = true;
                continue;
            }
            if (x - r < minX) minX = x - r;
            if (x + r > maxX) maxX = x + r;
            if (y - r < minY) minY = y - r;
            if (y + r > maxY) maxY = y + r;
        }
        if (!any) return false;

        return view.fit(
            minX, maxX, minY, maxY, OLED_MAP_PANE_X, 0, OLED_MAP_PANE_W,
            static_cast<int16_t>(OLED_HEIGHT)
        );
    }

    // The planned route, in millimetres, drawn as a polyline. Optional: the
    // map and the robot draw without it.
    //
    // Millimetres and not cells, so this class stays independent of MazeMapper
    // and of the maze size. The caller materialises the cells.
    void setRoute(etl::span<const Vec2D> points) {
        route = points;
    }

    void setProgress(etl::delegate<float()> p) {
        progress = p;
    }

    void update() {
        if (!display.ready()) return;
        if (!view.valid()) return;
        if (!display.due()) return;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        drawMap();
        drawRoute();
        drawRobot();
        drawText();

        if (progress.is_valid()) drawProgressBar(display, progress());

        g.display();
    }

    private:

    OLEDDisplay& display;
    const MapT& map;
    etl::delegate<Pose()> pose;
    etl::delegate<float()> progress;
    etl::span<const Vec2D> route;
    OLEDView view;

    // std::get_if rather than std::visit, matching Obstacle: visit is not
    // constexpr in the GCC 7.2 libstdc++ the Renesas core ships.
    void drawMap() {
        Adafruit_SSD1306& g = display.gfx();

        for (size_t i = 0; i < map.size(); ++i) {
            if (!map.present(i)) continue;
            const Obstacle o = map[i];

            if (const WallObstacle* w = std::get_if<WallObstacle>(&o.form)) {
                const float half = 0.5f * w->panelLength();
                const float ax   = trig::xcos(w->panelAlpha());
                const float ay   = trig::xsin(w->panelAlpha());
                const float x0   = o.centre.x - half * ax;
                const float y0   = o.centre.y - half * ay;
                const float x1   = o.centre.x + half * ax;
                const float y1   = o.centre.y + half * ay;
                g.drawLine(
                    view.sx(x0, y0), view.sy(x0, y0), view.sx(x1, y1), view.sy(x1, y1),
                    SSD1306_WHITE
                );
                continue;
            }

            const CircularObstacle* c = std::get_if<CircularObstacle>(&o.form);
            const int16_t px          = view.sx(o.centre.x, o.centre.y);
            const int16_t py          = view.sy(o.centre.x, o.centre.y);
            const int16_t r =
                static_cast<int16_t>(lroundf(view.scale() * c->boundingRadius()));

            // A 6 mm post is well under a pixel at this scale; drawCircle with
            // r = 0 draws nothing, so fall back to the pixel itself.
            if (r < 1) g.drawPixel(px, py, SSD1306_WHITE);
            else g.drawCircle(px, py, r, SSD1306_WHITE);
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

    // A dot at the pose plus a tick along the heading. Both are skipped if the
    // pose projects outside the map pane: the estimate can wander off the map,
    // and Adafruit_GFX clips to the panel, not to our pane, so an off-map dot
    // would be drawn over the text.
    void drawRobot() {
        if (!pose.is_valid()) return;

        const Pose p     = pose();
        const int16_t px = view.sx(p.x, p.y);
        const int16_t py = view.sy(p.x, p.y);
        if (!inPane(px, py)) return;

        // Long enough to read as a direction at ~0.04 px/mm, short enough to
        // stay inside one cell.
        const float tick = 0.5f * MAZE_CELL_SIZE;
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

    void drawText() {
        Adafruit_SSD1306& g = display.gfx();

        if (pose.is_valid()) {
            const Pose p = pose();
            g.setCursor(OLED_TEXT_PANE_X, 0);
            g.print(F("X "));
            g.print(p.x, 0);

            g.setCursor(OLED_TEXT_PANE_X, OLED_TEXT_HEIGHT);
            g.print(F("Y "));
            g.print(p.y, 0);

            g.setCursor(OLED_TEXT_PANE_X, 2 * OLED_TEXT_HEIGHT);
            g.print(F("T "));
            g.print(wrapAngle(p.theta) * 180.0f / PI, 0);
        }

        if (!progress.is_valid()) return;

        const float fraction = clampFraction(progress());

        g.setCursor(OLED_TEXT_PANE_X, 3 * OLED_TEXT_HEIGHT);
        g.print(static_cast<int>(lroundf(fraction * 100.0f)));
        g.print('%');
    }
};

#pragma GCC pop_options
