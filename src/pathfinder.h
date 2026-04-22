#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include "config.h"

// ============================================
// A* Pathfinding on sliding 40x40 grid
//
// Improvements over v1:
//   - 8-directional movement (diagonals)
//   - Octile distance heuristic (correct for 8-dir)
//   - Obstacle inflation (safety margin around walls)
//   - Diagonal blocked if adjacent cardinal is obstacle (no corner cutting)
//   - Unknown cells cost more (prefer explored areas)
//   - Path smoothing (remove unnecessary waypoints)
//   - Larger grid (40x40 = 4m x 4m window)
//   - More breadcrumbs (100 = 20m backtrack range)
//   - Max iterations guard (prevents freezing)
// ============================================

struct GridPoint { int x, y; };

// Cell types
#define CELL_FREE     0
#define CELL_OBSTACLE 1
#define CELL_UNKNOWN  2

// Movement costs (x10 for integer math)
#define COST_CARDINAL   10    // N/S/E/W
#define COST_DIAGONAL   14    // NE/NW/SE/SW (sqrt(2) * 10)
#define COST_UNKNOWN    20    // penalty for exploring unknown cells
#define COST_INFLATION  40    // penalty for cells adjacent to obstacles
#define MAX_ITERATIONS  800   // prevent infinite loops on ESP32

// 8-directional offsets: N, NE, E, SE, S, SW, W, NW
// Grid Y increases = world Y increases = NORTH
static const int DX8[] = { 0,  1,  1,  1,  0, -1, -1, -1};
static const int DY8[] = { 1,  1,  0, -1, -1, -1,  0,  1};

class Pathfinder {
public:
    uint8_t grid[GRID_SIZE][GRID_SIZE];
    GridPoint robotPos;
    GridPoint targetGrid;
    GridPoint path[200];      // computed path (max 200 waypoints)
    int pathLength;

    // World coordinates (cm)
    float worldOriginX = 0;
    float worldOriginY = 0;
    float targetWorldX = 0;
    float targetWorldY = 0;
    bool  hasTarget = false;
    bool  targetReached = false;

    // Breadcrumbs
    float crumbX[MAX_CRUMBS];
    float crumbY[MAX_CRUMBS];
    int   crumbCount = 0;
    int   backtrackIdx = -1;

    void begin() {
        clearMap();
        robotPos = {GRID_SIZE / 2, GRID_SIZE / 2};
        targetGrid = {GRID_SIZE / 2, 0};
        pathLength = 0;
        hasTarget = false;
        targetReached = false;
        crumbCount = 0;
        backtrackIdx = -1;
        worldOriginX = -(GRID_SIZE / 2) * CELL_SIZE_CM;
        worldOriginY = -(GRID_SIZE / 2) * CELL_SIZE_CM;
    }

    void clearMap() {
        memset(grid, CELL_UNKNOWN, sizeof(grid));
    }

    // --- Target ---
    void setTargetWorld(float wx, float wy) {
        targetWorldX = wx;
        targetWorldY = wy;
        hasTarget = true;
        targetReached = false;
        updateTargetGrid();
    }

    void setTarget(int gx, int gy) {
        setTargetWorld(worldOriginX + gx * CELL_SIZE_CM,
                       worldOriginY + gy * CELL_SIZE_CM);
    }

    // --- Update robot position ---
    void updateRobotWorld(float worldX, float worldY) {
        int gx = worldToGridX(worldX);
        int gy = worldToGridY(worldY);

        // Slide grid if near edge
        const int MARGIN = 6;
        if (gx < MARGIN)              shiftGrid(-(GRID_SIZE / 2 - MARGIN), 0);
        else if (gx >= GRID_SIZE - MARGIN) shiftGrid(GRID_SIZE / 2 - MARGIN, 0);
        if (gy < MARGIN)              shiftGrid(0, -(GRID_SIZE / 2 - MARGIN));
        else if (gy >= GRID_SIZE - MARGIN) shiftGrid(0, GRID_SIZE / 2 - MARGIN);

        // Recalculate after shift
        gx = worldToGridX(worldX);
        gy = worldToGridY(worldY);

        if (inBounds(gx, gy)) {
            robotPos = {gx, gy};
            grid[gy][gx] = CELL_FREE;
        }

        if (hasTarget) updateTargetGrid();

        // Drop breadcrumb every 20cm
        if (crumbCount == 0 ||
            dist2D(worldX, worldY, crumbX[crumbCount - 1], crumbY[crumbCount - 1]) > 20.0) {
            dropCrumb(worldX, worldY);
        }

        // Check target reached (within 15cm)
        if (hasTarget && !targetReached &&
            dist2D(worldX, worldY, targetWorldX, targetWorldY) < 15.0) {
            targetReached = true;
        }
    }

    // --- Mark obstacle from sensor reading ---
    void markObstacle(float distCm, float angleRad) {
        int cellDist = (int)(distCm / CELL_SIZE_CM);
        int ox = robotPos.x + (int)(cellDist * sin(angleRad));
        int oy = robotPos.y + (int)(cellDist * cos(angleRad));  // +cos: north = +gridY

        if (inBounds(ox, oy)) {
            grid[oy][ox] = CELL_OBSTACLE;
        }
        markLineFree(robotPos.x, robotPos.y, ox, oy);
    }

    // --- A* Pathfinding (8-directional, weighted, smoothed) ---
    bool findPath() {
        pathLength = 0;
        if (!hasTarget || targetReached) return false;

        // Clamp target to grid bounds
        GridPoint goal = targetGrid;
        goal.x = constrain(goal.x, 0, GRID_SIZE - 1);
        goal.y = constrain(goal.y, 0, GRID_SIZE - 1);

        // If goal is on an obstacle, find nearest free cell
        if (grid[goal.y][goal.x] == CELL_OBSTACLE) {
            if (!findNearestFree(goal)) return false;
        }

        // Build inflation cost map
        static uint8_t costMap[GRID_SIZE][GRID_SIZE];
        buildCostMap(costMap);

        // A* data structures.
        // g/f are int32 — with COST_INFLATION=40 + COST_UNKNOWN=20 + COST_DIAGONAL=14
        // per move, an 800-iter search can exceed int16_t range (32767).
        struct Node {
            int16_t x, y;
            int32_t g, f;
            int16_t parentIdx;
        };

        static Node openList[300];
        static Node closedList[500];
        static int16_t cameFrom[GRID_SIZE][GRID_SIZE]; // stores closedList index
        int openCount = 0, closedCount = 0;

        // Init visited
        memset(cameFrom, -1, sizeof(cameFrom));

        // Start node
        openList[openCount++] = {
            (int16_t)robotPos.x, (int16_t)robotPos.y,
            0, octileHeuristic(robotPos.x, robotPos.y, goal),
            -1
        };

        int iterations = 0;

        while (openCount > 0 && iterations < MAX_ITERATIONS) {
            iterations++;

            // Find lowest f in open list
            int bestIdx = 0;
            for (int i = 1; i < openCount; i++) {
                if (openList[i].f < openList[bestIdx].f) bestIdx = i;
            }

            Node current = openList[bestIdx];
            openList[bestIdx] = openList[--openCount];

            // Skip if already visited
            if (cameFrom[current.y][current.x] >= 0) continue;

            // Add to closed list
            int ci = closedCount;
            if (closedCount >= 500) return false;
            closedList[closedCount++] = current;
            cameFrom[current.y][current.x] = ci;

            // Goal reached?
            if (current.x == goal.x && current.y == goal.y) {
                reconstructPath(closedList, cameFrom, goal);
                smoothPath();
                return true;
            }

            // Explore 8 neighbors
            for (int d = 0; d < 8; d++) {
                int nx = current.x + DX8[d];
                int ny = current.y + DY8[d];

                if (!inBounds(nx, ny)) continue;
                if (cameFrom[ny][nx] >= 0) continue;       // already visited
                if (grid[ny][nx] == CELL_OBSTACLE) continue; // blocked

                // Diagonal: block if adjacent cardinals are obstacles (no corner cutting)
                bool isDiag = (d % 2 == 1);
                if (isDiag) {
                    int cx1 = current.x + DX8[d];
                    int cy1 = current.y;
                    int cx2 = current.x;
                    int cy2 = current.y + DY8[d];
                    if ((inBounds(cx1, cy1) && grid[cy1][cx1] == CELL_OBSTACLE) ||
                        (inBounds(cx2, cy2) && grid[cy2][cx2] == CELL_OBSTACLE)) {
                        continue; // blocked diagonal
                    }
                }

                int moveCost = isDiag ? COST_DIAGONAL : COST_CARDINAL;

                // Add cell type penalty
                if (grid[ny][nx] == CELL_UNKNOWN) moveCost += COST_UNKNOWN;

                // Add inflation penalty
                moveCost += costMap[ny][nx];

                int32_t g = current.g + moveCost;
                int32_t f = g + octileHeuristic(nx, ny, goal);

                if (openCount < 300) {
                    openList[openCount++] = {
                        (int16_t)nx, (int16_t)ny,
                        g, f,
                        (int16_t)ci
                    };
                }
            }
        }

        return false; // no path found
    }

    // --- Navigation direction ---
    int getNextDirection(float headingRad) {
        if (targetReached) return -2;
        if (pathLength < 2) return -2;

        GridPoint next = path[1];
        float dx = next.x - robotPos.x;
        float dy = next.y - robotPos.y;  // grid Y increases = north (not inverted)

        float targetAngle = atan2(dx, dy);
        float diff = targetAngle - headingRad;

        while (diff > PI)  diff -= 2 * PI;
        while (diff < -PI) diff += 2 * PI;

        if (abs(diff) < 0.3) return 0;   // forward
        return (diff > 0) ? 1 : -1;       // right : left
    }

    // --- Backtracking ---
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

        if (dist2D(rx, ry, crumbX[backtrackIdx], crumbY[backtrackIdx]) < 15.0) {
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

    // --- Octile distance heuristic (correct for 8-directional) ---
    int octileHeuristic(int x, int y, GridPoint goal) {
        int dx = abs(x - goal.x);
        int dy = abs(y - goal.y);
        // D=10, D2=14, so: 10*max + (14-10)*min = 10*max + 4*min
        return (dx > dy) ? (10 * dx + 4 * dy) : (10 * dy + 4 * dx);
    }

    // --- Build obstacle inflation cost map ---
    void buildCostMap(uint8_t costMap[GRID_SIZE][GRID_SIZE]) {
        memset(costMap, 0, GRID_SIZE * GRID_SIZE);

        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                if (grid[y][x] == CELL_OBSTACLE) {
                    // Inflate: add cost to all 8 neighbors
                    for (int d = 0; d < 8; d++) {
                        int nx = x + DX8[d];
                        int ny = y + DY8[d];
                        if (inBounds(nx, ny) && grid[ny][nx] != CELL_OBSTACLE) {
                            costMap[ny][nx] = COST_INFLATION;
                        }
                    }
                }
            }
        }
    }

    // --- Find nearest free cell to a blocked goal ---
    bool findNearestFree(GridPoint &goal) {
        for (int r = 1; r < GRID_SIZE / 2; r++) {
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    if (abs(dx) != r && abs(dy) != r) continue; // only perimeter
                    int nx = goal.x + dx;
                    int ny = goal.y + dy;
                    if (inBounds(nx, ny) && grid[ny][nx] != CELL_OBSTACLE) {
                        goal = {nx, ny};
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // --- Reconstruct path from A* result ---
    // NOTE: layout MUST match the Node struct in findPath().
    void reconstructPath(void* closedList, int16_t cameFrom[GRID_SIZE][GRID_SIZE], GridPoint goal) {
        struct Node { int16_t x, y; int32_t g, f; int16_t parentIdx; };
        Node* closed = (Node*)closedList;

        // Trace back from goal
        int tempPath[200];
        int tempLen = 0;
        int idx = cameFrom[goal.y][goal.x];

        while (idx >= 0 && tempLen < 200) {
            tempPath[tempLen++] = idx;
            idx = closed[idx].parentIdx;
        }

        // Reverse into path
        pathLength = 0;
        for (int i = tempLen - 1; i >= 0 && pathLength < 200; i--) {
            path[pathLength++] = {closed[tempPath[i]].x, closed[tempPath[i]].y};
        }
    }

    // --- Path smoothing: remove collinear waypoints ---
    void smoothPath() {
        if (pathLength < 3) return;

        GridPoint smoothed[200];
        int sLen = 0;

        smoothed[sLen++] = path[0];

        for (int i = 1; i < pathLength - 1; i++) {
            // Check if path[i] is collinear with prev and next
            int dx1 = path[i].x - smoothed[sLen - 1].x;
            int dy1 = path[i].y - smoothed[sLen - 1].y;
            int dx2 = path[i + 1].x - path[i].x;
            int dy2 = path[i + 1].y - path[i].y;

            // If direction changes, keep this waypoint
            if (dx1 != dx2 || dy1 != dy2) {
                // Also verify line-of-sight (no obstacles between prev and next)
                if (!lineOfSight(smoothed[sLen - 1], path[i + 1])) {
                    smoothed[sLen++] = path[i]; // must keep, obstacle in the way
                } else {
                    // Can skip this point (straight line is clear)
                }
            }
        }

        smoothed[sLen++] = path[pathLength - 1]; // always keep goal

        // Copy back
        pathLength = sLen;
        memcpy(path, smoothed, sLen * sizeof(GridPoint));
    }

    // --- Line-of-sight check (Bresenham) ---
    bool lineOfSight(GridPoint a, GridPoint b) {
        int x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (x0 != x1 || y0 != y1) {
            if (inBounds(x0, y0) && grid[y0][x0] == CELL_OBSTACLE) return false;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
        return true;
    }

    // --- Mark cells free along a ray ---
    void markLineFree(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (x0 != x1 || y0 != y1) {
            if (inBounds(x0, y0) && grid[y0][x0] != CELL_OBSTACLE) {
                grid[y0][x0] = CELL_FREE;
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    }

    // --- Slide grid ---
    void shiftGrid(int sx, int sy) {
        static uint8_t temp[GRID_SIZE][GRID_SIZE];
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                int srcX = x + sx, srcY = y + sy;
                temp[y][x] = (srcX >= 0 && srcX < GRID_SIZE && srcY >= 0 && srcY < GRID_SIZE)
                    ? grid[srcY][srcX] : CELL_UNKNOWN;
            }
        }
        memcpy(grid, temp, sizeof(grid));
        worldOriginX += sx * CELL_SIZE_CM;
        worldOriginY += sy * CELL_SIZE_CM;
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
