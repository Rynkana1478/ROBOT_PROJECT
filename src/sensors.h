#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

struct SweepData {
    float dist[SWEEP_STEPS];
    bool  fresh;
};

SweepData    sweepData = {};
portMUX_TYPE sweepMux  = portMUX_INITIALIZER_UNLOCKED;

// Sweep mode controls Core 1's serviceSweepStateMachine angle cycling.
// NORMAL      = 5-angle ping-pong. Each angle ~85 ms; idx 2 (front) ~340 ms.
// BYPASS_LEFT = alternate idx 2 + idx 4 (chassis turned right, wall on left).
// BYPASS_RIGHT= alternate idx 2 + idx 0 (chassis turned left,  wall on right).
// FRONT_LOCK  = servo never leaves idx 2. distFront refreshes every ~85 ms
//               instead of ~340 ms. Used during forward driving so the brake
//               trigger doesn't act on stale data while the servo wanders to
//               side angles.
// In non-NORMAL modes only the listed indices in sweepData.dist[] refresh;
// other cells retain pre-mode values. Read sweepDist[idx] directly rather
// than the distLeft/distRight aggregates (which would mix stale cells).
enum SweepMode { SWEEP_NORMAL, SWEEP_BYPASS_LEFT, SWEEP_BYPASS_RIGHT, SWEEP_FRONT_LOCK };
volatile SweepMode currentSweepMode = SWEEP_NORMAL;

class Sensors {
public:
    float distFront     = 999;
    float distLeft      = 999;
    float distRight     = 999;
    float distNearLeft  = 999;
    float distNearRight = 999;
    float sweepDist[SWEEP_STEPS];

    // Spurious-999 filter state. Reject "no echo" if recent reading was close.
    unsigned long lastValidFrontMs = 0;
    static constexpr float FRONT_999_REJECT_BELOW_CM = 80.0f;  // close enough to brake-relevant
    static constexpr unsigned long FRONT_999_HOLD_MS = 250;    // ~2-3 sensor cycles

    float heading    = 0;
    float headingRad = 0;
    float gyroRate   = 0;

    // Filtered accel along forward axis (Y per chassis mounting: Y forward, X right)
    float accelX = 0;     // lateral
    float accelY = 0;     // forward (used for slip detection)
    float accelZ = 0;

    Adafruit_MPU6050 mpu;

    float gyroZOffset = 0;
    float prevGyroZ = 0;
    int badI2Ccount = 0;
    unsigned long lastHeadingUpdate = 0;
    bool mpuReady = false;

    // Auto re-cal: track stillness via RAW gyro stats so the variance check
    // isn't biased by the (possibly wrong) offset we're trying to correct.
    unsigned long stillSinceMs = 0;
    float gyroRawSum   = 0;     // sum of raw g.gyro.z (rad/s)
    float gyroRawSumSq = 0;     // sum of raw g.gyro.z² (rad²/s²)
    int   gyroRawSamples = 0;
    bool  motorsActiveExternal = false;  // updated each loop from main

    void begin() {
        pinMode(US_TRIG, OUTPUT);
        pinMode(US_ECHO, INPUT);
        for (int i = 0; i < SWEEP_STEPS; i++) sweepDist[i] = 999;
        initMPU();
    }

    float readUltrasonic() {
        digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US_TRIG, LOW);
        unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
        if (dur == 0) return 999;
        float d = dur / 58.0;
        return (d > 400) ? 999 : d;
    }

    void updateFromSweep() {
        float tempDist[SWEEP_STEPS];
        portENTER_CRITICAL(&sweepMux);
        if (!sweepData.fresh) {
            portEXIT_CRITICAL(&sweepMux);
            return;
        }
        memcpy(tempDist, sweepData.dist, sizeof(tempDist));
        sweepData.fresh = false;
        portEXIT_CRITICAL(&sweepMux);

        memcpy(sweepDist, tempDist, sizeof(sweepDist));

        // Spurious-999 filter for distFront: a 999 immediately after a close
        // reading is almost always a missed echo (vibration tilts the sensor,
        // echo misses the receiver). Reject for up to FRONT_999_HOLD_MS so
        // the brake doesn't unfire on a phantom "path clear" reading. After
        // the hold expires we accept 999 — it really is far.
        float frontRaw = tempDist[2];
        unsigned long now = millis();
        if (frontRaw >= 999.0f && distFront < FRONT_999_REJECT_BELOW_CM &&
            now - lastValidFrontMs < FRONT_999_HOLD_MS) {
            // keep previous distFront — don't update
        } else {
            distFront = frontRaw;
            if (frontRaw < 999.0f) lastValidFrontMs = now;
        }

        distNearRight = tempDist[1];
        distNearLeft  = tempDist[3];
        distRight     = min(tempDist[0], tempDist[1]);
        distLeft      = min(tempDist[3], tempDist[4]);
    }

    void updateHeading() {
        unsigned long now = millis();
        if (lastHeadingUpdate == 0) { lastHeadingUpdate = now; return; }
        float dt = (now - lastHeadingUpdate) / 1000.0;
        lastHeadingUpdate = now;

        float gz = 0;
        if (mpuReady) {
            sensors_event_t a, g, t;
            if (mpu.getEvent(&a, &g, &t)) {
                float az = abs(a.acceleration.z);
                if (az < ACCEL_Z_MIN || az > ACCEL_Z_MAX) {
                    badI2Ccount++;
                    gz = 0;
                } else {
                    // Low-pass accel (kills motor vibration, keeps DC offset)
                    accelX = accelX * (1 - ACCEL_LOWPASS_ALPHA) + a.acceleration.x * ACCEL_LOWPASS_ALPHA;
                    accelY = accelY * (1 - ACCEL_LOWPASS_ALPHA) + a.acceleration.y * ACCEL_LOWPASS_ALPHA;
                    accelZ = accelZ * (1 - ACCEL_LOWPASS_ALPHA) + a.acceleration.z * ACCEL_LOWPASS_ALPHA;

                    // Sign-flipped: this chassis's MPU mount produces NEGATIVE
                    // g.gyro.z when the chassis rotates physically clockwise
                    // (right turn). To match calcBearing convention (0=N, 90=E,
                    // increasing CW), heading must INCREASE for right turns.
                    // Validated by test 08 mode 8 — without this flip the
                    // navigator commands the wrong direction, robot spins for
                    // 2-3 revolutions before failsafe intercepts. See test
                    // session notes on absolute heading turn validation.
                    gz = -(g.gyro.z - gyroZOffset) * 180.0 / PI;
                    if (abs(gz - prevGyroZ) > GYRO_JUMP_LIMIT) gz = prevGyroZ;
                    if (abs(gz) > GYRO_MAX_RATE) gz = prevGyroZ;
                    if (abs(gz) < GYRO_DEADZONE) gz = 0;
                    prevGyroZ = gz;
                    badI2Ccount = 0;

                    // Auto-recal accumulator: stash raw gyro samples (rad/s) so we
                    // can compute true variance independent of the current offset.
                    if (!motorsActiveExternal) {
                        gyroRawSum   += g.gyro.z;
                        gyroRawSumSq += g.gyro.z * g.gyro.z;
                        gyroRawSamples++;
                    }
                }
            } else {
                badI2Ccount++;
                gz = 0;
            }
        }
        gyroRate = gz;
        heading += gz * dt;

        while (heading >= 360) heading -= 360;
        while (heading < 0)    heading += 360;
        headingRad = heading * PI / 180.0;

        // Auto-recal trigger: when motors stopped for HEADING_RECAL_STILL_MS,
        // check raw-gyro variance (offset-independent). If the board is truly
        // still, snap gyroZOffset to the raw mean — this can correct ANY bias,
        // including drift that exceeded the deadzone and made heading "spin"
        // on the dashboard.
        if (!motorsActiveExternal) {
            if (stillSinceMs == 0) stillSinceMs = now;
            if (now - stillSinceMs > HEADING_RECAL_STILL_MS && gyroRawSamples > 20) {
                float meanRad = gyroRawSum / gyroRawSamples;
                float varRad  = gyroRawSumSq / gyroRawSamples - meanRad * meanRad;
                // Compare in deg²/s² for readability
                float varDeg = varRad * (180.0 / PI) * (180.0 / PI);
                if (varDeg < HEADING_RECAL_GYRO_VAR * HEADING_RECAL_GYRO_VAR) {
                    gyroZOffset = meanRad;
                    prevGyroZ = 0;
                }
                gyroRawSum = 0;
                gyroRawSumSq = 0;
                gyroRawSamples = 0;
                stillSinceMs = now;
            }
        } else {
            stillSinceMs = 0;
            gyroRawSum = 0;
            gyroRawSumSq = 0;
            gyroRawSamples = 0;
        }
    }

    void resetHeading() {
        heading = 0;
        headingRad = 0;
        prevGyroZ = 0;
        lastHeadingUpdate = millis();
    }

    // Manual heading override (e.g. dashboard "north pin" snap)
    void setHeading(float deg) {
        while (deg >= 360) deg -= 360;
        while (deg < 0)    deg += 360;
        heading = deg;
        headingRad = deg * PI / 180.0;
        prevGyroZ = 0;
        lastHeadingUpdate = millis();
    }

    bool isPathClear()     { return distFront > OBSTACLE_CLEAR; }
    bool isObstacleClose() { return distFront < OBSTACLE_CLOSE; }

    float getBatteryVoltage() {
        int raw = analogRead(BATTERY_PIN);
        return (raw / 4095.0) * 3.3 * 7.67;
    }

private:
    void initMPU() {
        if (!mpu.begin(0x68, &Wire)) return;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        // 500 deg/s range: chassis tank-turns at SPEED_TURN can hit 200+ deg/s,
        // so 250 deg/s would saturate. 500 gives 2x headroom.
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        mpuReady = true;

        float sum = 0;
        int count = 0;
        unsigned long start = millis();
        while (millis() - start < 2000) {
            sensors_event_t a, g, t;
            mpu.getEvent(&a, &g, &t);
            sum += g.gyro.z;
            count++;
            delay(5);
        }
        if (count > 0) gyroZOffset = sum / count;
    }
};

#endif
