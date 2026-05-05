#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include "config.h"

struct GridPoint { int x, y; };

#define CELL_FREE     0
#define CELL_OBSTACLE 1
#define CELL_UNKNOWN  2
#define CELL_VISITED  3

class Pathfinder {
public:
    uint8_t  grid[GRID_SIZE][GRID_SIZE];
    uint32_t obstacleAge[GRID_SIZE][GRID_SIZE];
    GridPoint robotPos;
    GridPoint targetGrid;

    float worldOriginX = 0;
    float worldOriginY = 0;
    float targetWorldX = 0;
    float targetWorldY = 0;
    bool  hasTarget = false;
    bool  targetReached = false;
    bool  gridDirty = false;

    float crumbX[MAX_CRUMBS];
    float crumbY[MAX_CRUMBS];
    int   crumbCount = 0;
    int   backtrackIdx = -1;

    void begin() {
        clearMap();
        robotPos = {GRID_SIZE / 2, GRID_SIZE / 2};
        targetGrid = {GRID_SIZE / 2, 0};
        hasTarget = false;
        targetReached = false;
        crumbCount = 0;
        backtrackIdx = -1;
        gridDirty = true;
        worldOriginX = -(GRID_SIZE / 2) * CELL_SIZE_CM;
        worldOriginY = -(GRID_SIZE / 2) * CELL_SIZE_CM;
    }

    void clearMap() {
        memset(grid, CELL_UNKNOWN, sizeof(grid));
        memset(obstacleAge, 0, sizeof(obstacleAge));
    }

    void setTargetWorld(float wx, float wy) {
        targetWorldX = wx;
        targetWorldY = wy;
        hasTarget = true;
        targetReached = false;
        updateTargetGrid();
    }

    void updateRobotWorld(float worldX, float worldY) {
        int gx = worldToGridX(worldX);
        int gy = worldToGridY(worldY);

        const int MARGIN = 4;
        if (gx < MARGIN)                  shiftGrid(-(GRID_SIZE / 2 - MARGIN), 0);
        else if (gx >= GRID_SIZE - MARGIN) shiftGrid(GRID_SIZE / 2 - MARGIN, 0);
        if (gy < MARGIN)                  shiftGrid(0, -(GRID_SIZE / 2 - MARGIN));
        else if (gy >= GRID_SIZE - MARGIN) shiftGrid(0, GRID_SIZE / 2 - MARGIN);

        gx = worldToGridX(worldX);
        gy = worldToGridY(worldY);

        if (inBounds(gx, gy)) {
            robotPos = {gx, gy};
            grid[gy][gx] = CELL_VISITED;
            gridDirty = true;
        }

        if (hasTarget) updateTargetGrid();

        if (crumbCount == 0 ||
            dist2D(worldX, worldY, crumbX[crumbCount - 1], crumbY[crumbCount - 1]) > 20.0) {
            dropCrumb(worldX, worldY);
        }

        if (hasTarget && !targetReached &&
            dist2D(worldX, worldY, targetWorldX, targetWorldY) < NAV_REACHED_CM) {
            targetReached = true;
        }
    }

    void markObstacle(float distCm, float angleRad) {
        if (distCm > NAV_OBSTACLE_MARK_CM) return;

        int cellDist = (int)(distCm / CELL_SIZE_CM);
        int ox = robotPos.x + (int)(cellDist * sin(angleRad));
        int oy = robotPos.y + (int)(cellDist * cos(angleRad));

        if (inBounds(ox, oy)) {
            grid[oy][ox] = CELL_OBSTACLE;
            obstacleAge[oy][ox] = millis();
            gridDirty = true;
        }
        markLineFree(robotPos.x, robotPos.y, ox, oy);
    }

    void decayObstacles() {
        unsigned long now = millis();
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                if (grid[y][x] == CELL_OBSTACLE && obstacleAge[y][x] > 0) {
                    if (now - obstacleAge[y][x] > OBSTACLE_DECAY_MS) {
                        grid[y][x] = CELL_UNKNOWN;
                        obstacleAge[y][x] = 0;
                        gridDirty = true;
                    }
                }
            }
        }
    }

    int gridSidePreference(float headingRad) {
        int leftFree = 0, rightFree = 0;
        float leftAngle = headingRad - PI / 2;
        float rightAngle = headingRad + PI / 2;

        for (int r = 1; r <= 3; r++) {
            int lx = robotPos.x + (int)(r * sin(leftAngle));
            int ly = robotPos.y + (int)(r * cos(leftAngle));
            int rx = robotPos.x + (int)(r * sin(rightAngle));
            int ry = robotPos.y + (int)(r * cos(rightAngle));

            if (inBounds(lx, ly) && (grid[ly][lx] == CELL_FREE || grid[ly][lx] == CELL_VISITED))
                leftFree++;
            if (inBounds(rx, ry) && (grid[ry][rx] == CELL_FREE || grid[ry][rx] == CELL_VISITED))
                rightFree++;
        }

        return rightFree - leftFree;
    }

    void startBacktrack() {
        if (crumbCount == 0) return;
        backtrackIdx = crumbCount - 1;
        hasTarget = true;
        targetReached = false;
        targetWorldX = crumbX[backtrackIdx];
        targetWorldY = crumbY[backtrackIdx];
        updateTargetGrid();
    }

    bool updateBacktrack(float rx, float ry) {
        if (backtrackIdx < 0) return false;

        if (dist2D(rx, ry, crumbX[backtrackIdx], crumbY[backtrackIdx]) < NAV_REACHED_CM) {
            backtrackIdx--;
            if (backtrackIdx < 0) {
                hasTarget = false;
                targetReached = true;
                return false;
            }
            targetWorldX = crumbX[backtrackIdx];
            targetWorldY = crumbY[backtrackIdx];
            updateTargetGrid();
        }
        return true;
    }

    bool isBacktracking() { return backtrackIdx >= 0; }

    float distToTarget(float wx, float wy) {
        return dist2D(wx, wy, targetWorldX, targetWorldY);
    }

    int encodeGridRLE(char* buf, int maxLen) {
        int pos = 0;
        uint8_t cur = grid[0][0];
        int count = 1;
        for (int i = 1; i < GRID_SIZE * GRID_SIZE; i++) {
            uint8_t cell = grid[i / GRID_SIZE][i % GRID_SIZE];
            if (cell == cur) {
                count++;
            } else {
                int w = snprintf(buf + pos, maxLen - pos, "%d,%d,", cur, count);
                if (w < 0 || pos + w >= maxLen - 12) return pos;
                pos += w;
                cur = cell;
                count = 1;
            }
        }
        int w = snprintf(buf + pos, maxLen - pos, "%d,%d", cur, count);
        if (w > 0) pos += w;
        return pos;
    }

private:
    bool inBounds(int x, int y) {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }

    int worldToGridX(float wx) { return (int)floor((wx - worldOriginX) / CELL_SIZE_CM); }
    int worldToGridY(float wy) { return (int)floor((wy - worldOriginY) / CELL_SIZE_CM); }

    float dist2D(float x1, float y1, float x2, float y2) {
        float dx = x1 - x2, dy = y1 - y2;
        return sqrt(dx * dx + dy * dy);
    }

    void updateTargetGrid() {
        targetGrid.x = worldToGridX(targetWorldX);
        targetGrid.y = worldToGridY(targetWorldY);
    }

    void markLineFree(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (x0 != x1 || y0 != y1) {
            if (inBounds(x0, y0) && grid[y0][x0] != CELL_OBSTACLE && grid[y0][x0] != CELL_VISITED) {
                grid[y0][x0] = CELL_FREE;
                gridDirty = true;
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    }

    void shiftGrid(int sx, int sy) {
        static uint8_t  tempGrid[GRID_SIZE][GRID_SIZE];
        static uint32_t tempAge[GRID_SIZE][GRID_SIZE];
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                int srcX = x + sx, srcY = y + sy;
                if (srcX >= 0 && srcX < GRID_SIZE && srcY >= 0 && srcY < GRID_SIZE) {
                    tempGrid[y][x] = grid[srcY][srcX];
                    tempAge[y][x]  = obstacleAge[srcY][srcX];
                } else {
                    tempGrid[y][x] = CELL_UNKNOWN;
                    tempAge[y][x]  = 0;
                }
            }
        }
        memcpy(grid, tempGrid, sizeof(grid));
        memcpy(obstacleAge, tempAge, sizeof(obstacleAge));
        worldOriginX += sx * CELL_SIZE_CM;
        worldOriginY += sy * CELL_SIZE_CM;
        gridDirty = true;
    }

    void dropCrumb(float wx, float wy) {
        if (crumbCount < MAX_CRUMBS) {
            crumbX[crumbCount] = wx;
            crumbY[crumbCount] = wy;
            crumbCount++;
        } else {
            memmove(crumbX, crumbX + 1, (MAX_CRUMBS - 1) * sizeof(float));
            memmove(crumbY, crumbY + 1, (MAX_CRUMBS - 1) * sizeof(float));
            crumbX[MAX_CRUMBS - 1] = wx;
            crumbY[MAX_CRUMBS - 1] = wy;
        }
    }
};

#endif
