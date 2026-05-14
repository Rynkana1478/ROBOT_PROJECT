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
    NAV_TURN_ONLY,    // pure rotation: turn to targetHeading and stop, no driving
    NAV_REACHED,
    NAV_FAILED
};

enum AvoidPhase {
    AV_BRAKE,
    AV_PICK_SIDE,
    AV_TURN_INCREMENT,  // 4-wheel tank turn 30° per increment toward avoidTargetHeading
    AV_CHECK_FRONT,     // wait for fresh sensor reading after the turn settles
    AV_DRIVE_PAST       // drive forward AVOID_DRIVE_PAST_CM, then exit
};

class Navigator {
public:
    NavState state = NAV_IDLE;
    bool avoidActive = false;
    int  avoidCount = 0;
    bool stuckWarning = false;
    bool avoidGaveUp = false; // manual-mode cooldown: stay braked after a failed avoid
    float targetHeading = 0;  // for NAV_TURN_ONLY (degrees, normalized 0..360)

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
        avoidGaveUp = false;
        avoidPhase = AV_BRAKE;
        phaseTimer = 0;
        scanTimer = 0;
        avoidWindowStart = 0;
        pathLengthCm = 0;
        avoidLastDX = 0; avoidLastDY = 0; avoidLastDHeading = 0;
        turnIncrementCount = 0;
        driveStartX = 0; driveStartY = 0;
        currentSweepMode = SWEEP_NORMAL;
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

    // Pure rotation: turn deltaDeg from current heading and stop.
    // Caller (executeQueuedCmd) is responsible for setting pf.turnOnly so the
    // pathfinder skips its distance-based reached check while we rotate.
    void setTurnTarget(Motors &motors, float curHeadingDeg, float deltaDeg) {
        motors.brake();
        state = NAV_TURN_ONLY;
        targetHeading = curHeadingDeg + deltaDeg;
        while (targetHeading >= 360) targetHeading -= 360;
        while (targetHeading <    0) targetHeading += 360;
        targetStartMs = millis();
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

        // NAV_TURN_ONLY runs ahead of the pathfinder-based logic — it has no
        // X,Y target, just a heading goal. When done, mark pf.targetReached so
        // the brain loop's existing completion path fires.
        if (state == NAV_TURN_ONLY) {
            currentSweepMode = SWEEP_NORMAL;
            float err = angleDiff(targetHeading, headingDeg);
            if (smoothTurnStep(motors, err, sensors.gyroRate)) {
                state = NAV_REACHED;
                motors.stop();
                pf.targetReached = true;
                pf.turnOnly = false;
            }
            return;
        }

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
                // Sweep refreshes side cells while we turn; AV_PICK_SIDE will
                // need fresh distLeft / distRight if we hit something next.
                currentSweepMode = SWEEP_NORMAL;
                if (smoothTurnStep(motors, headingErr, sensors.gyroRate)) {
                    // Reached tolerance or would overshoot → brake, let SCAN settle.
                    state = NAV_SCAN_AHEAD;
                    scanTimer = millis();
                }
                break;

            case NAV_SCAN_AHEAD:
                currentSweepMode = SWEEP_NORMAL;
                if (millis() - scanTimer > 150) {
                    state = NAV_DRIVING;
                }
                break;

            case NAV_DRIVING: {
                // Lock servo to front while driving so distFront refreshes
                // every ~85 ms instead of ~340 ms — prevents the brake from
                // acting on stale data while the servo is at a side angle.
                currentSweepMode = SWEEP_FRONT_LOCK;
                // Front-only trigger: near-left/near-right cells (servo 55°/125°)
                // see things 35° off-center, which catches walls running parallel
                // to the chassis even when the forward path is clear. Only the
                // dead-center reading should drive the avoid → drive transition.
                // Enter avoid at OBSTACLE_SLOW (was OBSTACLE_CLOSE): cruise
                // momentum plus sensor lag can skip the 5cm gap between SLOW
                // and CLOSE in a single tick, hitting brake at full speed.
                // Triggering at SLOW gives the brake the full 30cm runway.
                if (sensors.distFront < OBSTACLE_SLOW) {
                    enterAvoid(motors, posX, posY, headingDeg);
                    break;
                }

                if (abs(headingErr) > NAV_DRIFT_THRESHOLD) {
                    state = NAV_TURNING;
                    break;
                }

                if (distTarget < NAV_REACHED_CM) {
                    // Same reach path for regular targets AND backtrack.
                    // Backtrack is now just a single target = (0,0); the
                    // pathfinder's distance check in updateRobotWorld() flips
                    // targetReached, and we ack here.
                    state = NAV_REACHED;
                    motors.stop();
                    pf.targetReached = true;
                    break;
                }

                // Approach speed: three-tier so the brake never fights cruise
                // momentum.
                //   distFront > OBSTACLE_SLOWDOWN && distTarget > 50 -> CRUISE
                //   OBSTACLE_SLOW < distFront <= OBSTACLE_SLOWDOWN  -> SLOW
                //   distFront <= OBSTACLE_SLOW                       -> avoid trigger above
                // distTarget gate ensures we slow down approaching the goal too.
                int driveSpeed;
                if (distTarget > 50 && sensors.distFront > OBSTACLE_SLOWDOWN) {
                    driveSpeed = SPEED_CRUISE;
                } else if (sensors.distFront > OBSTACLE_SLOW) {
                    driveSpeed = SPEED_SLOW;
                } else {
                    driveSpeed = AVOID_BYPASS_SPEED;   // unreachable: avoid already triggered
                }
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

    // Manual-mode avoid: called every brain tick while user holds forward.
    // Returns true if avoid (or its post-failure cooldown) is currently
    // driving the motors — caller must NOT issue executeManualState() in
    // that case. Returns false when the path is clear and normal manual
    // forward should run.
    bool updateManualAvoid(Sensors &sensors, Motors &motors, Pathfinder &pf,
                           float posX, float posY, float headingDeg) {
        // In-progress avoid drives motors; the FSM owns sweep mode too.
        if (avoidActive) {
            runAvoid(sensors, motors, pf, posX, posY, headingDeg);
            return true;
        }

        // After a give-up: stay braked until the path opens past OBSTACLE_CLEAR.
        // Prevents retrying the same blocked-on-all-sides geometry every tick.
        if (avoidGaveUp) {
            if (sensors.distFront >= OBSTACLE_CLEAR) {
                avoidGaveUp = false;
            } else {
                motors.brake();
                return true;
            }
        }

        // Trigger at OBSTACLE_SLOW (was OBSTACLE_CLOSE) — same reason as the
        // auto-mode trigger: cruise momentum plus sensor lag can skip the
        // 5cm gap to OBSTACLE_CLOSE in a single tick.
        if (sensors.distFront < OBSTACLE_SLOW) {
            enterAvoid(motors, posX, posY, headingDeg);
            return true;
        }

        return false;
    }

    // Called from the brain loop when the user lets go of forward (or hits
    // stop / a different direction) mid-avoid. Stops the FSM cleanly so the
    // user's command takes effect immediately on the next tick.
    void cancelManualAvoid(Motors &motors) {
        if (avoidActive) {
            avoidActive = false;
            avoidPhase = AV_BRAKE;
            motors.brake();
        }
        avoidGaveUp = false;
        currentSweepMode = SWEEP_NORMAL;
    }

    const char* stateStr() {
        switch (state) {
            case NAV_IDLE:       return "IDLE";
            case NAV_TURNING:    return "TURNING";
            case NAV_SCAN_AHEAD: return "SCAN";
            case NAV_DRIVING:    return "DRIVING";
            case NAV_AVOIDING:   return "AVOIDING";
            case NAV_TURN_ONLY:  return "TURN";
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

    // Incremental-turn state.
    int   turnIncrementCount = 0;
    float driveStartX = 0;
    float driveStartY = 0;

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
                // Sweep stays in NORMAL during the brake/pick/turn legs so
                // distLeft/distRight refresh to fresh data before AV_PICK_SIDE
                // reads them. 4-wheel tank-turn doesn't need clearance, so we
                // go straight to PICK_SIDE without ever reversing.
                currentSweepMode = SWEEP_NORMAL;
                if (now - phaseTimer > 200) {
                    avoidPhase = AV_PICK_SIDE;
                    phaseTimer = now;
                }
                break;

            case AV_PICK_SIDE: {
                // Reject sides that are already blocked. We only sidestep
                // toward space we can actually drive into.
                bool leftClear  = sensors.distLeft  >= OBSTACLE_SLOW;
                bool rightClear = sensors.distRight >= OBSTACLE_SLOW;

                if (!leftClear && !rightClear && stuckWarning) {
                    // Both walls confirmed close AND we've already retried —
                    // give up cleanly rather than thrash. avoidGaveUp signals
                    // manual mode to sit braked until the path opens up,
                    // instead of immediately retriggering avoid each tick.
                    motors.stop();
                    avoidActive = false;
                    avoidGaveUp = true;
                    if (state == NAV_AVOIDING) state = NAV_FAILED;
                    break;
                }

                if (leftClear && !rightClear) {
                    avoidDirection = -1;
                } else if (rightClear && !leftClear) {
                    avoidDirection = 1;
                } else {
                    // Both clear (or both blocked first try): rank by sweep
                    // distance with a 15 cm margin; on near-tie, pick the side
                    // that points closer to the target bearing.
                    if (sensors.distLeft  > sensors.distRight + 15) {
                        avoidDirection = -1;
                    } else if (sensors.distRight > sensors.distLeft  + 15) {
                        avoidDirection = 1;
                    } else {
                        float targetBearing = calcBearing(posX, posY,
                                                          pf.targetWorldX, pf.targetWorldY);
                        float relBearing = angleDiff(targetBearing, headingDeg);
                        avoidDirection = (relBearing >= 0) ? 1 : -1;
                    }
                }

                if (stuckWarning) avoidDirection = -avoidDirection;

                // First increment: 30° toward chosen side.
                avoidTargetHeading = headingDeg +
                                     (avoidDirection * AVOID_TURN_INCREMENT_DEG);
                while (avoidTargetHeading >= 360) avoidTargetHeading -= 360;
                while (avoidTargetHeading < 0)    avoidTargetHeading += 360;
                turnIncrementCount = 1;

                avoidPhase = AV_TURN_INCREMENT;
                phaseTimer = now;
                break;
            }

            case AV_TURN_INCREMENT: {
                float err = angleDiff(avoidTargetHeading, headingDeg);
                if (now - phaseTimer > 3000 ||
                    smoothTurnStep(motors, err, sensors.gyroRate)) {
                    avoidPhase = AV_CHECK_FRONT;
                    phaseTimer = now;
                }
                break;
            }

            case AV_CHECK_FRONT: {
                // Wait for one full sweep cycle so distFront reflects the
                // post-turn orientation, not a reading taken mid-rotation.
                if (now - phaseTimer < AVOID_CHECK_SETTLE_MS) {
                    motors.brake();
                    break;
                }

                if (sensors.distFront > OBSTACLE_SLOW) {
                    // Front is clear at the new heading — commit to driving past.
                    // Switch sweep to BYPASS_* watching the side we turned AWAY
                    // from. avoidDirection > 0 means we turned right (CW), so
                    // the trailing wall is on our LEFT → SWEEP_BYPASS_LEFT
                    // alternates idx 2 (front) with idx 3 (near-left 125°).
                    // Symmetric for the other side. distFront still refreshes
                    // every other sweep cycle (~170ms), good enough for the
                    // ~12 cm/s bypass speed.
                    driveStartX = posX;
                    driveStartY = posY;
                    currentSweepMode = (avoidDirection > 0)
                                       ? SWEEP_BYPASS_LEFT
                                       : SWEEP_BYPASS_RIGHT;
                    avoidPhase = AV_DRIVE_PAST;
                    phaseTimer = now;
                    break;
                }

                // Still blocked at this heading. Try another increment.
                if (turnIncrementCount >= AVOID_MAX_INCREMENTS) {
                    // Cumulative turn has hit the cap (default 120°). Give up.
                    motors.stop();
                    avoidActive = false;
                    avoidGaveUp = true;
                    if (state == NAV_AVOIDING) state = NAV_FAILED;
                    break;
                }

                avoidTargetHeading += avoidDirection * AVOID_TURN_INCREMENT_DEG;
                while (avoidTargetHeading >= 360) avoidTargetHeading -= 360;
                while (avoidTargetHeading < 0)    avoidTargetHeading += 360;
                turnIncrementCount++;
                avoidPhase = AV_TURN_INCREMENT;
                phaseTimer = now;
                break;
            }

            case AV_DRIVE_PAST: {
                // Front-clip: brake at OBSTACLE_SLOW (raised from CLOSE).
                // At AVOID_BYPASS_SPEED the chassis covers ~3 cm per sensor
                // refresh, so dropping the trigger to SLOW (30 cm) gives the
                // brake one full refresh cycle of runway.
                if (sensors.distFront < OBSTACLE_SLOW) {
                    motors.brake();
                    currentSweepMode = SWEEP_NORMAL;  // refresh sides for next pick
                    avoidPhase = AV_BRAKE;
                    phaseTimer = now;
                    break;
                }

                // Side-clip: we turned avoidDirection*30° AWAY from the wall,
                // so the trailing wall is on the opposite side of the chassis.
                // The near-side sensor (~35° off front) covers the front-side
                // corner that would clip first. Brake + re-pick if too close.
                float sideDist = (avoidDirection > 0)
                                 ? sensors.distNearLeft
                                 : sensors.distNearRight;
                if (sideDist < AVOID_SIDE_CLEAR_CM) {
                    motors.brake();
                    currentSweepMode = SWEEP_NORMAL;
                    avoidPhase = AV_BRAKE;
                    phaseTimer = now;
                    break;
                }

                float traveled = distance(driveStartX, driveStartY, posX, posY);
                if (traveled >= AVOID_DRIVE_PAST_CM) {
                    avoidLastDX = posX - avoidStartX;
                    avoidLastDY = posY - avoidStartY;
                    avoidLastDHeading = angleDiff(headingDeg, avoidStartHeading);
                    motors.brake();
                    avoidActive = false;
                    if (state == NAV_AVOIDING) state = NAV_TURNING;
                    break;
                }

                motors.setState(Motors::M_FORWARD, AVOID_BYPASS_SPEED);
                break;
            }
        }
    }
};

#endif
