#pragma once
#include <Arduino.h>
#include "constants.h"

class MazeMapper {
public:
    enum Heading : uint8_t { North = 0, East = 1, South = 2, West = 3 };
    enum WallBit : uint8_t { WallN = 1, WallE = 2, WallS = 4, WallW = 8 };

    struct Cell {
        int r;
        int c;
    };

    void begin(int startR, int startC, Heading startH, int goalR, int goalC) {
        start.r = startR;
        start.c = startC;
        current = start;
        heading = startH;
        goal.r = goalR;
        goal.c = goalC;
        stackTop = 0;
        finalPathLen = 0;
        exploringDone = false;

        for (uint8_t r = 0; r < MAZE_SIZE; ++r) {
            for (uint8_t c = 0; c < MAZE_SIZE; ++c) {
                walls[r][c] = 0;
                visited[r][c] = false;
                if (r == 0) addWall(r, c, South);
                if (r == MAZE_SIZE - 1) addWall(r, c, North);
                if (c == 0) addWall(r, c, West);
                if (c == MAZE_SIZE - 1) addWall(r, c, East);
            }
        }
    }

    int row() const { return current.r; }
    int col() const { return current.c; }
    Heading dir() const { return heading; }
    bool doneExploring() const { return exploringDone; }
    bool atGoal() const { return current.r == goal.r && current.c == goal.c; }

    void observe(bool frontWall, bool leftWall, bool rightWall) {
        visited[current.r][current.c] = true;
        if (frontWall) addWall(current.r, current.c, heading);
        if (leftWall)  addWall(current.r, current.c, leftOf(heading));
        if (rightWall) addWall(current.r, current.c, rightOf(heading));
    }

    bool chooseNext(Cell& next, Heading& moveDir, bool& backtracking) {
        Heading order[4] = { leftOf(heading), heading, rightOf(heading), backOf(heading) };

        for (uint8_t i = 0; i < 4; ++i) {
            Heading d = order[i];
            int nr = current.r + dr(d);
            int nc = current.c + dc(d);
            if (!inside(nr, nc)) continue;
            if (hasWall(current.r, current.c, d)) continue;
            if (!visited[nr][nc]) {
                push(current);
                next = {nr, nc};
                moveDir = d;
                backtracking = false;
                return true;
            }
        }

        if (stackTop > 0) {
            next = pop();
            moveDir = directionTo(current, next);
            backtracking = true;
            return true;
        }

        exploringDone = true;
        return false;
    }

    void setCurrent(Cell cell, Heading newHeading) {
        clearWallBetween(current, cell);
        current = cell;
        heading = newHeading;
    }

    bool buildShortestPathToGoal() {
        return buildPath(start, goal);
    }

    uint8_t shortestPathLength() const { return finalPathLen; }
    Cell shortestPathCell(uint8_t index) const { return finalPath[index]; }

    Heading directionTo(const Cell& from, const Cell& to) const {
        if (to.r > from.r) return North;
        if (to.c > from.c) return East;
        if (to.r < from.r) return South;
        return West;
    }

    bool hasWall(int r, int c, Heading d) const {
        return (walls[r][c] & bit(d)) != 0;
    }

    static Heading leftOf(Heading h)  { return (Heading)((h + 3) & 3); }
    static Heading rightOf(Heading h) { return (Heading)((h + 1) & 3); }
    static Heading backOf(Heading h)  { return (Heading)((h + 2) & 3); }

    static int dr(Heading h) {
        if (h == North) return 1;
        if (h == South) return -1;
        return 0;
    }

    static int dc(Heading h) {
        if (h == East) return 1;
        if (h == West) return -1;
        return 0;
    }

    static bool inside(int r, int c) {
        return r >= 0 && r < MAZE_SIZE && c >= 0 && c < MAZE_SIZE;
    }

private:
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];
    bool visited[MAZE_SIZE][MAZE_SIZE];

    Cell start;
    Cell goal;
    Cell current;
    Heading heading;

    Cell stack[MAZE_SIZE * MAZE_SIZE];
    uint8_t stackTop = 0;
    bool exploringDone = false;

    Cell finalPath[MAZE_SIZE * MAZE_SIZE];
    uint8_t finalPathLen = 0;

    static uint8_t bit(Heading d) {
        if (d == North) return WallN;
        if (d == East) return WallE;
        if (d == South) return WallS;
        return WallW;
    }

    void push(Cell cell) {
        if (stackTop < MAZE_SIZE * MAZE_SIZE) stack[stackTop++] = cell;
    }

    Cell pop() {
        if (stackTop == 0) return current;
        return stack[--stackTop];
    }

    void addWall(int r, int c, Heading d) {
        if (!inside(r, c)) return;
        walls[r][c] |= bit(d);
        int nr = r + dr(d);
        int nc = c + dc(d);
        if (inside(nr, nc)) walls[nr][nc] |= bit(backOf(d));
    }

    void clearWallBetween(Cell a, Cell b) {
        Heading d = directionTo(a, b);
        walls[a.r][a.c] &= ~bit(d);
        walls[b.r][b.c] &= ~bit(backOf(d));
    }

    bool buildPath(Cell from, Cell to) {
        bool seen[MAZE_SIZE][MAZE_SIZE];
        Cell parent[MAZE_SIZE][MAZE_SIZE];
        Cell queue[MAZE_SIZE * MAZE_SIZE];
        uint8_t head = 0;
        uint8_t tail = 0;

        for (uint8_t r = 0; r < MAZE_SIZE; ++r) {
            for (uint8_t c = 0; c < MAZE_SIZE; ++c) {
                seen[r][c] = false;
                parent[r][c] = {-1, -1};
            }
        }

        seen[from.r][from.c] = true;
        queue[tail++] = from;

        while (head < tail) {
            Cell u = queue[head++];
            if (u.r == to.r && u.c == to.c) break;

            for (uint8_t d = 0; d < 4; ++d) {
                Heading hd = (Heading)d;
                int nr = u.r + dr(hd);
                int nc = u.c + dc(hd);
                if (!inside(nr, nc)) continue;
                if (hasWall(u.r, u.c, hd)) continue;
                if (!visited[nr][nc]) continue;
                if (seen[nr][nc]) continue;
                seen[nr][nc] = true;
                parent[nr][nc] = u;
                queue[tail++] = {nr, nc};
            }
        }

        if (!seen[to.r][to.c]) {
            finalPathLen = 0;
            return false;
        }

        Cell reversePath[MAZE_SIZE * MAZE_SIZE];
        uint8_t n = 0;
        Cell p = to;
        while (!(p.r == from.r && p.c == from.c)) {
            reversePath[n++] = p;
            p = parent[p.r][p.c];
        }
        reversePath[n++] = from;

        finalPathLen = n;
        for (uint8_t i = 0; i < n; ++i) {
            finalPath[i] = reversePath[n - 1 - i];
        }
        return true;
    }
};
