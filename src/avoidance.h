#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"

// Non-blocking obstacle avoidance state machine.
// Distances come from continuous sweep (always fresh, no wait).

enum AvoidState {
    AVOID_IDLE,
    AVOID_SLOWDOWN,
    AVOID_BRAKE,
    AVOID_REVERSING,
    AVOID_TURNING
};

class ObstacleAvoidance {
public:
    AvoidState state = AVOID_IDLE;

    void begin() {
        state = AVOID_IDLE;
        actionTimer = 0;
    }

    bool update(Sensors &sensors, Motors &motors) {
        unsigned long now = millis();

        switch (state) {

            case AVOID_IDLE:
                if (sensors.distFront < OBSTACLE_CLOSE) {
                    motors.brake();
                    state = AVOID_BRAKE;
                    actionTimer = now;
                    return true;
                }
                if (sensors.distFront < OBSTACLE_SLOW) {
                    int dir = sensors.bestDirection();
                    if (dir < 0)      motors.curveLeft(SPEED_SLOW);
                    else if (dir > 0) motors.curveRight(SPEED_SLOW);
                    else              motors.forward(SPEED_SLOW);
                    state = AVOID_SLOWDOWN;
                    actionTimer = now;
                    return true;
                }
                return false;

            case AVOID_BRAKE:
                motors.brake();
                if (sensors.distFront > OBSTACLE_CLEAR) {
                    state = AVOID_IDLE;
                    return false;
                }
                if (now - actionTimer >= 300) {
                    motors.backward(SPEED_SLOW);
                    actionTimer = now;
                    state = AVOID_REVERSING;
                }
                return true;

            case AVOID_SLOWDOWN:
                if (sensors.distFront < OBSTACLE_CLOSE) {
                    motors.brake();
                    state = AVOID_BRAKE;
                    actionTimer = now;
                    return true;
                }
                if (sensors.distFront > OBSTACLE_CLEAR) {
                    state = AVOID_IDLE;
                    return false;
                }
                if (now - actionTimer >= 500) {
                    int dir = sensors.bestDirection();
                    if (dir < 0)      motors.curveLeft(SPEED_SLOW);
                    else if (dir > 0) motors.curveRight(SPEED_SLOW);
                    else              motors.forward(SPEED_SLOW);
                    actionTimer = now;
                }
                return true;

            case AVOID_REVERSING:
                if (now - actionTimer >= 300) {
                    motors.brake();
                    int dir = sensors.bestDirection();
                    if (dir <= 0) motors.turnLeft(SPEED_TURN);
                    else          motors.turnRight(SPEED_TURN);
                    actionTimer = now;
                    state = AVOID_TURNING;
                }
                return true;

            case AVOID_TURNING:
                if (sensors.distFront > OBSTACLE_CLEAR) {
                    motors.brake();
                    state = AVOID_IDLE;
                    return false;
                }
                if (now - actionTimer >= 450) {
                    motors.brake();
                    if (sensors.distFront > OBSTACLE_SLOW) {
                        state = AVOID_IDLE;
                        return false;
                    }
                    state = AVOID_BRAKE;
                    actionTimer = now;
                }
                return true;
        }

        return false;
    }

private:
    unsigned long actionTimer;
};

#endif
