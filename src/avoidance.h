#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"

// NON-BLOCKING obstacle avoidance state machine
// FIX BUG 2: Does NOT call sweepTick() - main loop handles that.
// FIX BUG 1: Uses readFrontNow() which checks servo is at center.

enum AvoidState {
    AVOID_IDLE,       // No obstacle - pathfinder has control
    AVOID_SLOWDOWN,   // Obstacle 15-30cm - scanning + curving
    AVOID_BRAKE,      // Obstacle <15cm - emergency brake
    AVOID_REVERSING,  // Backing up
    AVOID_TURNING     // Turning away from obstacle
};

class ObstacleAvoidance {
public:
    AvoidState state = AVOID_IDLE;

    void begin() {
        state = AVOID_IDLE;
        actionTimer = 0;
    }

    // Call every ~100ms. NEVER blocks. Does NOT call sweepTick().
    bool update(Sensors &sensors, Motors &motors) {
        unsigned long now = millis();

        // Read front only when servo is pointing forward
        // (returns false if servo is mid-sweep, keeps stale value)
        sensors.readFrontNow();

        switch (state) {

            case AVOID_IDLE:
                if (sensors.distFront < OBSTACLE_CLOSE) {
                    motors.brake();
                    sensors.startSweep();
                    state = AVOID_BRAKE;
                    return true;
                }
                if (sensors.distFront < OBSTACLE_SLOW) {
                    motors.forward(SPEED_SLOW);
                    sensors.startSweep();
                    state = AVOID_SLOWDOWN;
                    return true;
                }
                return false;

            case AVOID_BRAKE:
                motors.brake();

                if (sensors.distFront > OBSTACLE_CLEAR && sensors.servoAtCenter()) {
                    state = AVOID_IDLE;
                    return false;
                }

                if (sensors.isSweepDone()) {
                    sensors.clearSweepDone();
                    motors.backward(SPEED_SLOW);
                    actionTimer = now;
                    state = AVOID_REVERSING;
                }
                return true;

            case AVOID_SLOWDOWN:
                if (sensors.distFront < OBSTACLE_CLOSE && sensors.servoAtCenter()) {
                    motors.brake();
                    if (!sensors.isSweeping()) sensors.startSweep();
                    state = AVOID_BRAKE;
                    return true;
                }

                if (sensors.distFront > OBSTACLE_CLEAR && sensors.servoAtCenter()) {
                    state = AVOID_IDLE;
                    return false;
                }

                if (sensors.isSweepDone()) {
                    sensors.clearSweepDone();
                    int dir = sensors.bestDirection();
                    if (dir < 0) {
                        motors.curveLeft(SPEED_SLOW);
                    } else if (dir > 0) {
                        motors.curveRight(SPEED_SLOW);
                    } else {
                        motors.forward(SPEED_SLOW);
                    }
                    actionTimer = now;
                }

                if (!sensors.isSweeping() && (now - actionTimer > 500)) {
                    sensors.startSweep();
                }
                return true;

            case AVOID_REVERSING:
                if (now - actionTimer >= 300) {
                    motors.brake();
                    int dir2 = sensors.bestDirection();
                    if (dir2 <= 0) {
                        motors.turnLeft(SPEED_TURN);
                    } else {
                        motors.turnRight(SPEED_TURN);
                    }
                    actionTimer = now;
                    state = AVOID_TURNING;
                }
                return true;

            case AVOID_TURNING:
                // Only trust front reading when servo is forward
                if (sensors.distFront > OBSTACLE_CLEAR && sensors.servoAtCenter()) {
                    motors.brake();
                    state = AVOID_IDLE;
                    return false;
                }

                if (now - actionTimer >= 450) {
                    motors.brake();
                    if (sensors.distFront > OBSTACLE_SLOW && sensors.servoAtCenter()) {
                        state = AVOID_IDLE;
                        return false;
                    }
                    sensors.startSweep();
                    state = AVOID_BRAKE;
                }
                return true;

        }

        return false;
    }

private:
    unsigned long actionTimer;
};

#endif // AVOIDANCE_H
