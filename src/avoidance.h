#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"

// NON-BLOCKING obstacle avoidance state machine
// Distances are always fresh from continuous Core 0 sweep

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
                    actionTimer = now;
                    state = AVOID_BRAKE;
                    return true;
                }
                if (sensors.distFront < OBSTACLE_SLOW) {
                    motors.forward(SPEED_SLOW);
                    actionTimer = now;
                    state = AVOID_SLOWDOWN;
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
                    actionTimer = now;
                    state = AVOID_BRAKE;
                    return true;
                }
                if (sensors.distFront > OBSTACLE_CLEAR) {
                    state = AVOID_IDLE;
                    return false;
                }
                if (now - actionTimer > 300) {
                    int dir = sensors.bestDirection();
                    if (dir < 0) motors.curveLeft(SPEED_SLOW);
                    else if (dir > 0) motors.curveRight(SPEED_SLOW);
                    else motors.forward(SPEED_SLOW);
                    actionTimer = now;
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
                    actionTimer = now;
                    state = AVOID_BRAKE;
                }
                return true;
        }

        return false;
    }

private:
    unsigned long actionTimer;
};

#endif
