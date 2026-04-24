#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// Shared sweep data: Core 0 writes distances, Core 1 reads via updateFromSweep()
struct SweepData {
    float dist[SWEEP_STEPS];
    bool  fresh;
};

SweepData    sweepData = {};
portMUX_TYPE sweepMux  = portMUX_INITIALIZER_UNLOCKED;

class Sensors {
public:
    float distFront = 999;
    float distLeft  = 999;
    float distRight = 999;
    float heading   = 0;
    float headingRad = 0;
    float gyroRate = 0;

    Adafruit_MPU6050 mpu;

    float gyroZOffset = 0;
    float prevGyroZ = 0;
    int badI2Ccount = 0;
    unsigned long lastHeadingUpdate = 0;
    bool mpuReady = false;

    void begin() {
        pinMode(US_TRIG, OUTPUT);
        pinMode(US_ECHO, INPUT);
        initMPU();
    }

    // Called from Core 0 only
    float readUltrasonic() {
        digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US_TRIG, LOW);
        unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
        if (dur == 0) return 999;
        float d = dur / 58.0;
        return (d > 400) ? 999 : d;
    }

    // Called from Core 1 -- copies latest sweep data into front/left/right
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

        float minFront = 999, minLeft = 999, minRight = 999;
        for (int i = 0; i < SWEEP_STEPS; i++) {
            int angle = SWEEP_START_ANGLE + i * SWEEP_STEP_DEG;
            float d = tempDist[i];
            if (angle <= 50)  { if (d < minRight) minRight = d; }
            if (angle >= 130) { if (d < minLeft)  minLeft  = d; }
            if (angle >= 60 && angle <= 120) { if (d < minFront) minFront = d; }
        }
        distFront = minFront;
        distLeft  = minLeft;
        distRight = minRight;
    }

    // Gyro-only heading with I2C corruption detection
    void updateHeading() {
        unsigned long now = millis();
        if (lastHeadingUpdate == 0) { lastHeadingUpdate = now; return; }
        float dt = (now - lastHeadingUpdate) / 1000.0;
        lastHeadingUpdate = now;

        float gz = 0;
        if (mpuReady) {
            sensors_event_t a, g, t;
            if (mpu.getEvent(&a, &g, &t)) {
                // Gravity sanity check: accel Z should be ~9.8 m/s²
                // If it reads wildly off, the I2C data is corrupted
                float az = abs(a.acceleration.z);
                if (az < ACCEL_Z_MIN || az > ACCEL_Z_MAX) {
                    badI2Ccount++;
                    gz = 0;  // discard entire reading
                } else {
                    gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
                    // Reject if rate changed too fast between consecutive reads
                    if (abs(gz - prevGyroZ) > GYRO_JUMP_LIMIT) gz = prevGyroZ;
                    if (abs(gz) > GYRO_MAX_RATE) gz = prevGyroZ;
                    if (abs(gz) < GYRO_DEADZONE) gz = 0;
                    prevGyroZ = gz;
                    badI2Ccount = 0;
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
    }

    bool isPathClear()     { return distFront > OBSTACLE_CLEAR; }
    bool isObstacleClose() { return distFront < OBSTACLE_CLOSE; }

    int bestDirection() {
        if (distFront > OBSTACLE_CLEAR) return 0;
        if (distLeft > distRight && distLeft > OBSTACLE_SLOW) return -1;
        if (distRight > distLeft && distRight > OBSTACLE_SLOW) return 1;
        return (distLeft > distRight) ? -1 : 1;
    }

    float getBatteryVoltage() {
        int raw = analogRead(BATTERY_PIN);
        return (raw / 4095.0) * 3.3 * 7.67;
    }

private:
    void initMPU() {
        if (!mpu.begin(0x68, &Wire)) return;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
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
