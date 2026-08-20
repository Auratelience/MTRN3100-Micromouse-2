// Boot self-check for the runtime maze grid
//
// Zimmy Levi z5587840
//
// Built only under ./compile.sh --debug. The risk in making the grid size a
// runtime value is a place that still reads the template capacity N where it
// should read the selected n: the result is a mapper that explores cells the
// maze does not have, which the panel cannot show you.
//
// Runs on the live mapper rather than a private one. A second MazeMapper at the
// capacity would be ~1.5 kB of RAM to test with, and it is not needed --
// runStartupUI() reconfigures the mapper afterwards, so whatever this leaves
// behind is overwritten before the run begins.

#pragma once

#ifdef MICROMOUSE_DEBUG

#include <Arduino.h>

#include "constants.h"
#include "mazeMapper.h"
#include "types.h"

// Linter hidden as this is a header-only library
// NOLINTBEGIN(misc-definitions-in-headers)

inline bool checkOne(const char* what, bool ok) {
    if (!ok) {
        Serial.print(F("  FAIL: "));
        Serial.println(what);
    }
    return ok;
}

// One grid size, end to end, on the live mapper.
template <size_t N>
bool selfCheckGrid(MazeMapper<N>& mapper, uint8_t n) {
    Serial.print(F("self-check n="));
    Serial.println(n);

    const int8_t last = static_cast<int8_t>(n - 1);
    bool ok           = true;

    ok = checkOne("configure rejected an in-range size",
                  mapper.configure(n, Cell{0, 0}, North, Cell{last, last})) && ok;
    ok = checkOne("begin() failed", mapper.begin()) && ok;
    ok = checkOne("gridSize() disagrees", mapper.gridSize() == n) && ok;
    ok = checkOne("cellCount() disagrees",
                  mapper.cellCount() == static_cast<uint16_t>(n) * n) && ok;

    // The perimeter is sealed exactly at the n boundary.
    ok = checkOne("no wall on the north edge", mapper.hasWall(Cell{last, 0}, North)) && ok;
    ok = checkOne("no wall on the south edge", mapper.hasWall(Cell{0, 0}, South)) && ok;
    ok = checkOne("no wall on the east edge", mapper.hasWall(Cell{0, 0}, East)) && ok;
    ok = checkOne("no wall on the west edge", mapper.hasWall(Cell{0, last}, West)) && ok;

    // ...and nowhere else. Only meaningful once there is an interior.
    if (n >= 3) {
        ok = checkOne("phantom interior wall", !mapper.hasWall(Cell{1, 1}, North)) && ok;
    }

    // A cell past the edge is outside the maze, whatever the capacity is.
    ok = checkOne("markWall accepted a cell outside n",
                  !mapper.markWall(Cell{static_cast<int8_t>(n), 0}, North)) && ok;

    return ok;
}

template <size_t N>
bool runSelfChecks(MazeMapper<N>& mapper) {
    // A plain array, not an initializer_list: nothing else in this tree pulls
    // <initializer_list> in and a debug-only header is a poor place to start.
    const uint8_t sizes[] = {2, 5, 9, MAZE_SIZE_MAX};

    bool ok = true;
    for (uint8_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        ok = selfCheckGrid(mapper, sizes[i]) && ok;
    }

    ok = checkOne("configure accepted a size below MAZE_SIZE_MIN",
                  !mapper.configure(MAZE_SIZE_MIN - 1, Cell{0, 0}, North, Cell{1, 1})) && ok;
    ok = checkOne("configure accepted a size above the capacity",
                  !mapper.configure(MAZE_SIZE_MAX + 1, Cell{0, 0}, North, Cell{1, 1})) && ok;

    Serial.println(ok ? F("self-check PASSED") : F("self-check FAILED"));
    return ok;
}

// NOLINTEND(misc-definitions-in-headers)

#endif  // MICROMOUSE_DEBUG
