#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"

// Dual wheel encoders — pure odometry (distance only)
// Direction comes from MPU6050 heading, not encoder ticks
// Translation only integrated for FORWARD/BACKWARD motor states.
// Tank turns: encoder ticks ignored for position (gyro provides heading).

volatile long encLeftCount  = 0;
volatile long encRightCount = 0;
static volatile unsigned long encLeftLastUs  = 0;
static volatile unsigned long encRightLastUs = 0;

#define ENC_DEBOUNCE_US 2000

void IRAM_ATTR encLeftISR() {
    unsigned long now = micros();
    if (now - encLeftLastUs > ENC_DEBOUNCE_US) {
        encLeftCount++;
        encLeftLastUs = now;
    }
}

void IRAM_ATTR encRightISR() {
    unsigned long now = micros();
    if (now - encRightLastUs > ENC_DEBOUNCE_US) {
        encRightCount++;
        encRightLastUs = now;
    }
}

class Encoder {
public:
    float posX = 0;
    float posY = 0;
    float totalDistCm = 0;
    bool encoderHealthy = true;
    float traction = 1.0;
    bool slipping = false;
    bool stalled = false;

    void begin() {
        pinMode(ENCODER_L_PIN, INPUT_PULLUP);
        pinMode(ENCODER_R_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encLeftISR, RISING);
        attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encRightISR, RISING);
        encLeftCount = 0;
        encRightCount = 0;
        prevLeft = 0;
        prevRight = 0;
        traction = 1.0;
        slipping = false;
        stalled = false;
        slipCycles = 0;
        lastTickMs = millis();
    }

    // motorState: from Motors::getState() — controls integration policy
    // accelY: forward-axis acceleration (m/s^2), low-pass filtered upstream
    // gyroZ:  yaw rate (deg/s), already deadzoned upstream
    void update(float headingRad, Motors::State motorState,
                float accelY, float gyroZ) {
        long left, right;
        portENTER_CRITICAL(&mux);
        left  = encLeftCount;
        right = encRightCount;
        portEXIT_CRITICAL(&mux);

        long dL = left - prevLeft;
        long dR = right - prevRight;
        prevLeft  = left;
        prevRight = right;

        // L/R balance health (only meaningful during forward/backward)
        if (motorState == Motors::M_FORWARD || motorState == Motors::M_BACKWARD) {
            if (dL > 1 && dR > 1) {
                float ratio = (dL > dR) ? (float)dL / dR : (float)dR / dL;
                encoderHealthy = (ratio < 3.0);
                if (ratio > 5.0) {
                    if (dL > dR) dL = dR * 5;
                    else dR = dL * 5;
                }
            }
        }
        if (dL == 0 && dR == 0) encoderHealthy = true;

        bool encoderMoving = (dL >= STALL_TICK_THRESHOLD || dR >= STALL_TICK_THRESHOLD);
        unsigned long now = millis();

        // ---- Stall detection: motor commanded but no ticks ----
        bool motorActive = (motorState == Motors::M_FORWARD ||
                            motorState == Motors::M_BACKWARD ||
                            motorState == Motors::M_TURN_L  ||
                            motorState == Motors::M_TURN_R);
        if (motorActive && !encoderMoving) {
            if (now - lastTickMs > STALL_THRESHOLD_MS) stalled = true;
        } else {
            lastTickMs = now;
            stalled = false;
        }

        // ---- Per-state slip rules ----
        updateSlip(motorState, encoderMoving, accelY, gyroZ);

        // ---- Position integration (motor-state gated) ----
        // Tank turns: NO translation, only heading (which gyro updates elsewhere).
        // Stop/Brake: nothing meaningful to integrate.
        if (motorState != Motors::M_FORWARD && motorState != Motors::M_BACKWARD) {
            return;
        }

        float distL = dL * CM_PER_TICK;
        float distR = dR * CM_PER_TICK;
        float distAvg = (distL + distR) / 2.0;
        if (motorState == Motors::M_BACKWARD) distAvg = -distAvg;

        // Slip flag is still raised by updateSlip() above and surfaces in
        // telemetry, but we no longer scale distance by `traction`. The slip
        // rule (encoder ticking + accelY < threshold) fires at constant
        // velocity — accel is zero at cruise — which used to drop traction
        // mid-trip and make 200 cm commands overshoot to 300-400 cm.

        totalDistCm += abs(distAvg);
        posX += distAvg * sin(headingRad);
        posY += distAvg * cos(headingRad);
    }

    int getGridX() { return (int)floor(posX / CELL_SIZE_CM) + (GRID_SIZE / 2); }
    int getGridY() { return (GRID_SIZE / 2) - (int)floor(posY / CELL_SIZE_CM); }

    void resetPosition() {
        portENTER_CRITICAL(&mux);
        encLeftCount = 0;
        encRightCount = 0;
        portEXIT_CRITICAL(&mux);
        prevLeft = 0;
        prevRight = 0;
        posX = 0;
        posY = 0;
        totalDistCm = 0;
        traction = 1.0;
        slipping = false;
        stalled = false;
        slipCycles = 0;
        lastTickMs = millis();
    }

    long getLeftCount()  { return encLeftCount; }
    long getRightCount() { return encRightCount; }

private:
    long prevLeft = 0;
    long prevRight = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    int slipCycles = 0;
    unsigned long lastTickMs = 0;
    unsigned long lastRotationMs = 0;       // last time gyro showed real rotation
    Motors::State prevMotorState = Motors::M_STOP;

    // Motor-state-aware slip: each state has its own "expected sensor signature"
    // FORWARD/BACKWARD: encoder ticks must coincide with sustained accelY.
    // TURN_L/TURN_R:    encoder ticks must coincide with recent gyro rotation
    //                   (NOT instantaneous — gyro takes 50-200ms to spin up,
    //                   and noise dips around the threshold cause false slips).
    void updateSlip(Motors::State motorState, bool encoderMoving,
                    float accelY, float gyroZ) {
        unsigned long now = millis();

        bool isTurning = (motorState == Motors::M_TURN_L ||
                          motorState == Motors::M_TURN_R);
        bool wasTurning = (prevMotorState == Motors::M_TURN_L ||
                           prevMotorState == Motors::M_TURN_R);

        // Prime the rotation timer on turn entry so the gyro ramp-up window
        // doesn't trip a false slip before the body actually starts rotating.
        if (isTurning && !wasTurning) lastRotationMs = now;
        prevMotorState = motorState;

        // Latch every cycle the gyro genuinely shows rotation.
        if (fabs(gyroZ) > SLIP_GYRO_THRESHOLD) lastRotationMs = now;

        bool expectedSignaturePresent = true;
        bool checkActive = false;

        if ((motorState == Motors::M_FORWARD || motorState == Motors::M_BACKWARD)
            && encoderMoving) {
            checkActive = true;
            expectedSignaturePresent = (fabs(accelY) > SLIP_ACCEL_THRESHOLD);
        } else if (isTurning && encoderMoving) {
            checkActive = true;
            // Slip = ticks but gyro hasn't shown rotation in SLIP_GYRO_TIMEOUT_MS.
            // Brief dips below threshold during a real turn don't trip this.
            expectedSignaturePresent =
                (now - lastRotationMs < SLIP_GYRO_TIMEOUT_MS);
        }

        if (checkActive && !expectedSignaturePresent) {
            slipCycles++;
            if (slipCycles >= SLIP_DETECT_CYCLES) {
                slipping = true;
                traction = max(0.0f, traction - (float)SLIP_DECAY_RATE);
            }
        } else {
            slipCycles = max(0, slipCycles - 1);
            if (slipCycles == 0) slipping = false;
            traction = min(1.0f, traction + (float)SLIP_RECOVER_RATE);
        }
    }
};

#endif
