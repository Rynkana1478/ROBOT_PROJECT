#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// Shared sweep data: Core 0 writes, Core 1 reads
struct SweepData {
    float dist[SWEEP_STEPS];
    int   angles[SWEEP_STEPS];
    bool  fresh;
};

extern SweepData    sweepData;
extern portMUX_TYPE sweepMux;

class Sensors {
public:
    float distFront = 999;
    float distLeft  = 999;
    float distRight = 999;
    float heading   = 0;
    float headingRad = 0;
    float gyroRate = 0;

    float accelX = 0, accelY = 0, accelZ = 0;
    float gyroX = 0, gyroY = 0;
    float mpuTemp = 0;

    float gyroZOffset = 0;
    unsigned long lastHeadingUpdate = 0;
    bool mpuReady = false;

    Adafruit_MPU6050 mpu;

    void begin() {
        pinMode(US_TRIG, OUTPUT);
        pinMode(US_ECHO, INPUT);
        initMPU();

        for (int i = 0; i < SWEEP_STEPS; i++) {
            sweepData.dist[i] = 999;
            sweepData.angles[i] = SWEEP_START_ANGLE + i * SWEEP_STEP_DEG;
        }
        sweepData.fresh = false;
    }

    // Pull latest distances from shared sweep array (call from Core 1)
    void updateFromSweep() {
        float local[SWEEP_STEPS];
        portENTER_CRITICAL(&sweepMux);
        memcpy(local, sweepData.dist, sizeof(local));
        sweepData.fresh = false;
        portEXIT_CRITICAL(&sweepMux);

        // Angles: 20, 40, 60, 80, 100, 120, 140, 160
        // Index:   0   1   2   3   4    5    6    7
        //         R         FR  Front   FL        L
        distFront = 999;
        distLeft  = 999;
        distRight = 999;

        for (int i = 0; i < SWEEP_STEPS; i++) {
            int angle = SWEEP_START_ANGLE + i * SWEEP_STEP_DEG;
            int offset = abs(angle - SERVO_CENTER);

            if (offset <= 30) {
                if (local[i] < distFront) distFront = local[i];
            }
            if (angle >= 130) {
                if (local[i] < distLeft) distLeft = local[i];
            }
            if (angle <= 50) {
                if (local[i] < distRight) distRight = local[i];
            }
        }
    }

    // Gyro-only heading (compass hardware removed)
    void updateHeading() {
        unsigned long now = millis();
        if (lastHeadingUpdate == 0) { lastHeadingUpdate = now; return; }
        float dt = (now - lastHeadingUpdate) / 1000.0;
        lastHeadingUpdate = now;
        if (dt > 0.5) dt = 0.05;

        float gz = 0;
        if (mpuReady) {
            sensors_event_t a, g, t;
            mpu.getEvent(&a, &g, &t);
            accelX = a.acceleration.x; accelY = a.acceleration.y; accelZ = a.acceleration.z;
            gyroX = g.gyro.x * 180.0 / PI; gyroY = g.gyro.y * 180.0 / PI;
            mpuTemp = t.temperature;
            gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
            if (abs(gz) < GYRO_DEADZONE) gz = 0;
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

    float readUltrasonic() {
        digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US_TRIG, LOW);
        unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
        if (dur == 0) return 999;
        float d = dur / 58.0;
        if (d < 2.0) return 999;
        return (d > 400) ? 999 : d;
    }

private:
    void initMPU() {
        if (!mpu.begin(0x68, &Wire)) return;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
