#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// HC-SR04 on servo sweep (non-blocking)
// MPU6050 gyro + QMC5883L compass = complementary filter heading

enum SweepState {
    SWEEP_CENTER, SWEEP_MOVE_LEFT, SWEEP_READ_LEFT,
    SWEEP_MOVE_CENTER, SWEEP_READ_CENTER,
    SWEEP_MOVE_RIGHT, SWEEP_READ_RIGHT,
    SWEEP_RETURN, SWEEP_DONE
};

class Sensors {
public:
    float distFront = 999;
    float distLeft  = 999;
    float distRight = 999;
    float heading   = 0;       // fused heading 0-360 degrees
    float headingRad = 0;
    float compassHeading = -1; // raw compass heading (-1 = unavailable)
    float gyroRate = 0;        // current gyro Z rate (deg/s)

    Servo sweepServo;
    Adafruit_MPU6050 mpu;

    SweepState sweepState = SWEEP_CENTER;
    unsigned long sweepTimer = 0;
    bool sweepActive = false;
    bool sweepComplete = false;

    float gyroZOffset = 0;
    unsigned long lastHeadingUpdate = 0;
    bool mpuReady = false;
    bool compassReady = false;
    bool servoReady = false;

    void begin() {
        pinMode(US_TRIG, OUTPUT);
        pinMode(US_ECHO, INPUT);

        // Servo: motors already grabbed LEDC ch 0/1 in motors.begin().
        // ESP32Servo auto-allocates a free timer/channel.
        // attach() returns 0 (or -1 on some forks) when no channel is free.
        sweepServo.setPeriodHertz(50);
        int ch = sweepServo.attach(SERVO_PIN, 500, 2400);
        servoReady = sweepServo.attached();

        if (servoReady) {
            // Visible bring-up sweep so you can SEE it works
            sweepServo.write(SERVO_LEFT);   delay(300);
            sweepServo.write(SERVO_RIGHT);  delay(300);
            sweepServo.write(SERVO_CENTER); delay(300);
        }

        initMPU();
        initCompass();
    }

    // --- Front distance (only when servo at center) ---
    bool readFrontNow() {
        if (servoAtCenter()) {
            distFront = readUltrasonic();
            return true;
        }
        return false;
    }

    bool servoAtCenter() {
        return sweepState == SWEEP_CENTER || sweepState == SWEEP_READ_CENTER || sweepState == SWEEP_DONE;
    }

    // --- Non-blocking sweep ---
    bool sweepTick() {
        unsigned long now = millis();
        switch (sweepState) {
            case SWEEP_CENTER:      return false;
            case SWEEP_MOVE_LEFT:   sweepServo.write(SERVO_LEFT);   sweepTimer=now; sweepState=SWEEP_READ_LEFT;   return false;
            case SWEEP_READ_LEFT:   if(now-sweepTimer>=SERVO_SWEEP_MS){distLeft=readUltrasonic(); sweepState=SWEEP_MOVE_CENTER;} return false;
            case SWEEP_MOVE_CENTER: sweepServo.write(SERVO_CENTER); sweepTimer=now; sweepState=SWEEP_READ_CENTER; return false;
            case SWEEP_READ_CENTER: if(now-sweepTimer>=SERVO_SWEEP_MS){distFront=readUltrasonic();sweepState=SWEEP_MOVE_RIGHT;} return false;
            case SWEEP_MOVE_RIGHT:  sweepServo.write(SERVO_RIGHT);  sweepTimer=now; sweepState=SWEEP_READ_RIGHT;  return false;
            case SWEEP_READ_RIGHT:  if(now-sweepTimer>=SERVO_SWEEP_MS){distRight=readUltrasonic();sweepState=SWEEP_RETURN;}     return false;
            case SWEEP_RETURN:      sweepServo.write(SERVO_CENTER); sweepTimer=now; sweepState=SWEEP_DONE;        return false;
            case SWEEP_DONE:
                if (now - sweepTimer >= SERVO_SWEEP_MS) {
                    sweepState = SWEEP_CENTER;
                    sweepActive = false;
                    sweepComplete = true;
                    return true;
                }
                return false;
        }
        return false;
    }

    void startSweep()     { if (!sweepActive) { sweepActive=true; sweepComplete=false; sweepState=SWEEP_MOVE_LEFT; } }
    bool isSweeping()     { return sweepActive; }
    bool isSweepDone()    { return sweepComplete; }
    void clearSweepDone() { sweepComplete = false; }

    // --- Heading: Gyro + Compass complementary filter ---
    void updateHeading() {
        unsigned long now = millis();
        if (lastHeadingUpdate == 0) { lastHeadingUpdate = now; return; }
        float dt = (now - lastHeadingUpdate) / 1000.0;
        lastHeadingUpdate = now;

        // Read gyro
        float gz = 0;
        if (mpuReady) {
            sensors_event_t a, g, t;
            mpu.getEvent(&a, &g, &t);
            gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;  // deg/s
            if (abs(gz) < GYRO_DEADZONE) gz = 0;
        }
        gyroRate = gz;

        // Gyro-predicted heading
        float gyroHeading = heading + gz * dt;

        // Read compass
        if (compassReady) {
            compassHeading = readCompassHeading();

            if (compassHeading >= 0) {
                // Complementary filter with proper angle wrapping
                float diff = compassHeading - gyroHeading;
                while (diff > 180)  diff -= 360;
                while (diff < -180) diff += 360;

                heading = gyroHeading + COMPASS_ALPHA * diff;
            } else {
                heading = gyroHeading;  // Compass read failed, gyro only
            }
        } else {
            heading = gyroHeading;  // No compass, gyro only
        }

        // Wrap 0-360
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
        // ESP32-S3 ADC: 12-bit (0-4095), 0-3.3V range
        // With 220K+33K divider: Vin = Vadc * (220+33)/33 = Vadc * 7.67
        return (raw / 4095.0) * 3.3 * 7.67;
    }

private:
    float readUltrasonic() {
        digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US_TRIG, LOW);
        unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
        if (dur == 0) return 999;
        float d = dur / 58.0;
        return (d > 400) ? 999 : d;
    }

    void initMPU() {
        if (!mpu.begin(0x68, &Wire)) return;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpuReady = true;

        // Calibrate gyro
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

    void initCompass() {
        Wire.beginTransmission(COMPASS_ADDR);
        if (Wire.endTransmission() != 0) return;

        // QMC5883L init
        Wire.beginTransmission(COMPASS_ADDR);
        Wire.write(0x0B); Wire.write(0x01);  // SET/RESET
        Wire.endTransmission();
        Wire.beginTransmission(COMPASS_ADDR);
        Wire.write(0x09); Wire.write(0x1D);  // Continuous, 200Hz, 8G, OSR512
        Wire.endTransmission();

        compassReady = true;
    }

    float readCompassHeading() {
        Wire.beginTransmission(COMPASS_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
        Wire.requestFrom((int)COMPASS_ADDR, 6);

        if (Wire.available() >= 6) {
            int16_t x = Wire.read() | (Wire.read() << 8);
            int16_t y = Wire.read() | (Wire.read() << 8);
            Wire.read(); Wire.read(); // skip Z

            if (x == 0 && y == 0) return -1;

            float h = atan2((float)y, (float)x) * 180.0 / PI;
            if (h < 0) h += 360;
            return h;
        }
        return -1;
    }
};

#endif
