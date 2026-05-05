#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "pathfinder.h"

enum NavState {
    NAV_IDLE,
    NAV_TURNING,
    NAV_SCAN_AHEAD,
    NAV_DRIVING,
    NAV_AVOIDING,
    NAV_REACHED,
    NAV_FAILED
};

enum AvoidPhase {
    AV_BRAKE,
    AV_REVERSE,
    AV_PICK_SIDE,
    AV_TURN,
    AV_RECHECK
};

class Navigator {
public:
    NavState state = NAV_IDLE;
    bool avoidActive = false;
    int  avoidCount = 0;
    bool stuckWarning = false;

    // Avoidance displacement telemetry (start pose -> end pose)
    float avoidStartX = 0, avoidStartY = 0, avoidStartHeading = 0;
    float avoidLastDX = 0, avoidLastDY = 0, avoidLastDHeading = 0;

    // Wall-following stuck (path-length / straight-line ratio)
    float pathLengthCm = 0;
    float targetStartX = 0, targetStartY = 0;
    unsigned long targetStartMs = 0;

    void begin() {
        state = NAV_IDLE;
        avoidActive = false;
        avoidCount = 0;
        stuckWarning = false;
        avoidPhase = AV_BRAKE;
        phaseTimer = 0;
        scanTimer = 0;
        avoidWindowStart = 0;
        pathLengthCm = 0;
        avoidLastDX = 0; avoidLastDY = 0; avoidLastDHeading = 0;
    }

    // Called when a new target is set; resets path-tracking baseline.
    void setTarget(Motors &motors, float curX = 0, float curY = 0) {
        motors.brake();
        state = NAV_TURNING;
        targetStartX = curX;
        targetStartY = curY;
        targetStartMs = millis();
        pathLengthCm = 0;
        avoidCount = 0;
        stuckWarning = false;
    }

    // Add to running path-length odometer. Brain calls this each tick.
    void accumulatePath(float distAvgCm) {
        pathLengthCm += fabs(distAvgCm);
    }

    void update(Sensors &sensors, Motors &motors, Pathfinder &pf,
                float posX, float posY, float headingDeg) {
        if (state == NAV_IDLE || state == NAV_REACHED || state == NAV_FAILED) return;

        float bearingDeg = calcBearing(posX, posY, pf.targetWorldX, pf.targetWorldY);
        float headingErr = angleDiff(bearingDeg, headingDeg);
        float distTarget = pf.distToTarget(posX, posY);

        // Wall-follow stuck detection: path much longer than straight-line distance
        if (state == NAV_DRIVING || state == NAV_TURNING || state == NAV_AVOIDING) {
            unsigned long elapsed = millis() - targetStartMs;
            float straight = distance(targetStartX, targetStartY, posX, posY);
            if (pathLengthCm > WALL_STUCK_MIN_PATH_CM &&
                elapsed > WALL_STUCK_MIN_MS &&
                straight > 1.0 &&
                pathLengthCm / straight > WALL_STUCK_RATIO) {
                state = NAV_FAILED;
                motors.stop();
                stuckWarning = true;
                return;
            }
        }

        switch (state) {
            case NAV_TURNING:
                if (smoothTurnStep(motors, headingErr, sensors.gyroRate)) {
                    // Reached tolerance or would overshoot → brake, let SCAN settle.
                    state = NAV_SCAN_AHEAD;
                    scanTimer = millis();
                }
                break;

            case NAV_SCAN_AHEAD:
                if (millis() - scanTimer > 150) {
                    state = NAV_DRIVING;
                }
                break;

            case NAV_DRIVING: {
                if (sensors.distFront < OBSTACLE_CLOSE ||
                    sensors.distNearLeft < OBSTACLE_CLOSE ||
                    sensors.distNearRight < OBSTACLE_CLOSE) {
                    enterAvoid(motors, posX, posY, headingDeg);
                    break;
                }

                if (abs(headingErr) > NAV_DRIFT_THRESHOLD) {
                    state = NAV_TURNING;
                    break;
                }

                if (distTarget < NAV_REACHED_CM) {
                    if (pf.isBacktracking()) {
                        if (!pf.updateBacktrack(posX, posY)) {
                            state = NAV_REACHED;
                            motors.stop();
                        } else {
                            state = NAV_TURNING;
                        }
                    } else {
                        state = NAV_REACHED;
                        motors.stop();
                        pf.targetReached = true;
                    }
                    break;
                }

                int driveSpeed = (distTarget > 50 && sensors.distFront > OBSTACLE_SLOW)
                                 ? SPEED_CRUISE : SPEED_SLOW;
                motors.setState(Motors::M_FORWARD, driveSpeed);
                break;
            }

            case NAV_AVOIDING:
                runAvoid(sensors, motors, pf, posX, posY, headingDeg);
                break;

            default:
                break;
        }
    }

    bool updateManualAvoid(Sensors &sensors, Motors &motors, Pathfinder &pf,
                           float posX, float posY, float headingDeg) {
        if (avoidActive) {
            runAvoid(sensors, motors, pf, posX, posY, headingDeg);
            return avoidActive;
        }

        if (sensors.distFront < OBSTACLE_CLOSE ||
            sensors.distNearLeft < OBSTACLE_CLOSE ||
            sensors.distNearRight < OBSTACLE_CLOSE) {
            enterAvoid(motors, posX, posY, headingDeg);
            return true;
        }

        return false;
    }

    const char* stateStr() {
        switch (state) {
            case NAV_IDLE:       return "IDLE";
            case NAV_TURNING:    return "TURNING";
            case NAV_SCAN_AHEAD: return "SCAN";
            case NAV_DRIVING:    return "DRIVING";
            case NAV_AVOIDING:   return "AVOIDING";
            case NAV_REACHED:    return "REACHED";
            case NAV_FAILED:     return "FAILED";
            default:             return "?";
        }
    }

private:
    AvoidPhase avoidPhase;
    unsigned long phaseTimer;
    unsigned long scanTimer;
    unsigned long avoidWindowStart;
    float avoidTargetHeading;
    int avoidDirection;

    float calcBearing(float fromX, float fromY, float toX, float toY) {
        float dx = toX - fromX;
        float dy = toY - fromY;
        float deg = atan2(dx, dy) * 180.0 / PI;
        while (deg < 0)    deg += 360;
        while (deg >= 360) deg -= 360;
        return deg;
    }

    float angleDiff(float target, float current) {
        float diff = target - current;
        while (diff > 180)  diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    float distance(float x1, float y1, float x2, float y2) {
        float dx = x1 - x2, dy = y1 - y2;
        return sqrt(dx * dx + dy * dy);
    }

    // One step of a smooth proportional turn. Returns true when the turn is
    // "done" (within tolerance OR predicted overshoot) and brakes; caller
    // should advance state. Otherwise issues setState with a proportional speed.
    //
    // headingErr: signed deg, positive = need to turn right
    // gyroRate:   signed deg/s, same sign convention as heading integration
    bool smoothTurnStep(Motors &motors, float headingErr, float gyroRate) {
        float absErr = fabs(headingErr);

        // Done: within tolerance.
        if (absErr < NAV_HEADING_TOLERANCE) {
            motors.brake();
            return true;
        }

        // Stopping-distance brake: under brake, angular velocity decays roughly
        // linearly to zero over NAV_TURN_STOP_TIME_MS, so the angle covered
        // during braking ≈ |gyroRate| * stopTime / 2. If that already meets or
        // exceeds the remaining error AND we're rotating toward the target,
        // brake NOW — coasting will land us inside tolerance instead of past it.
        bool movingToward = (gyroRate * headingErr) > 0;
        float stopDist = fabs(gyroRate) * (NAV_TURN_STOP_TIME_MS / 1000.0f) / 2.0f;
        if (movingToward && stopDist >= absErr) {
            motors.brake();
            return true;
        }

        // Proportional speed: full above NAV_TURN_FULL_DEG, linearly down to MIN
        // at the tolerance edge. With FULL_DEG = 40°, the full last quarter of
        // a 90° turn is in the ramp — gives the brake predictor time to engage.
        int turnSpeed;
        if (absErr >= NAV_TURN_FULL_DEG) {
            turnSpeed = SPEED_TURN;
        } else {
            float t = (absErr - NAV_HEADING_TOLERANCE) /
                      (NAV_TURN_FULL_DEG - NAV_HEADING_TOLERANCE);
            if (t < 0) t = 0;
            turnSpeed = NAV_TURN_MIN_SPEED +
                        (int)(t * (SPEED_TURN - NAV_TURN_MIN_SPEED));
        }

        if (headingErr > 0) motors.setState(Motors::M_TURN_R, turnSpeed);
        else                motors.setState(Motors::M_TURN_L, turnSpeed);
        return false;
    }

    void enterAvoid(Motors &motors, float posX, float posY, float headingDeg) {
        avoidActive = true;
        avoidPhase = AV_BRAKE;
        phaseTimer = millis();
        motors.brake();

        avoidStartX = posX;
        avoidStartY = posY;
        avoidStartHeading = headingDeg;

        unsigned long now = millis();
        if (now - avoidWindowStart > AVOID_REPEAT_WINDOW_MS) {
            avoidCount = 0;
            avoidWindowStart = now;
        }
        avoidCount++;
        stuckWarning = (avoidCount >= AVOID_REPEAT_LIMIT);

        if (state != NAV_AVOIDING && state != NAV_IDLE) state = NAV_AVOIDING;
    }

    void runAvoid(Sensors &sensors, Motors &motors, Pathfinder &pf,
                  float posX, float posY, float headingDeg) {
        unsigned long now = millis();

        switch (avoidPhase) {
            case AV_BRAKE:
                motors.brake();
                if (now - phaseTimer > 200) {
                    avoidPhase = AV_REVERSE;
                    phaseTimer = now;
                    motors.setState(Motors::M_BACKWARD, SPEED_SLOW);
                }
                break;

            case AV_REVERSE:
                if (now - phaseTimer > 400) {
                    motors.brake();
                    avoidPhase = AV_PICK_SIDE;
                    phaseTimer = now;
                }
                break;

            case AV_PICK_SIDE: {
                if (sensors.distLeft > sensors.distRight + 10) {
                    avoidDirection = -1;
                } else if (sensors.distRight > sensors.distLeft + 10) {
                    avoidDirection = 1;
                } else {
                    int gridPref = pf.gridSidePreference(headingDeg * PI / 180.0);
                    avoidDirection = (gridPref >= 0) ? 1 : -1;
                }

                if (stuckWarning) avoidDirection = -avoidDirection;

                avoidTargetHeading = headingDeg + (avoidDirection * 90);
                while (avoidTargetHeading >= 360) avoidTargetHeading -= 360;
                while (avoidTargetHeading < 0)    avoidTargetHeading += 360;

                avoidPhase = AV_TURN;
                phaseTimer = now;
                break;
            }

            case AV_TURN: {
                float err = angleDiff(avoidTargetHeading, headingDeg);
                if (now - phaseTimer > 3000 ||
                    smoothTurnStep(motors, err, sensors.gyroRate)) {
                    avoidPhase = AV_RECHECK;
                    phaseTimer = now;
                }
                break;
            }

            case AV_RECHECK:
                if (now - phaseTimer > 100) {
                    bool clear = sensors.distFront > OBSTACLE_SLOW &&
                                 sensors.distNearLeft > OBSTACLE_CLOSE &&
                                 sensors.distNearRight > OBSTACLE_CLOSE;
                    if (clear) {
                        avoidLastDX = posX - avoidStartX;
                        avoidLastDY = posY - avoidStartY;
                        avoidLastDHeading = angleDiff(headingDeg, avoidStartHeading);
                        avoidActive = false;
                        if (state == NAV_AVOIDING) state = NAV_TURNING;
                    } else {
                        avoidPhase = AV_BRAKE;
                        phaseTimer = now;
                    }
                }
                break;
        }
    }
};

#endif
