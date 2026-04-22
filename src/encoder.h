#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// Dual wheel encoders with differential odometry
// Tracks position (X,Y) and can detect turns accurately

volatile long encLeftCount  = 0;
volatile long encRightCount = 0;
volatile int8_t encLeftDir  = 1;
volatile int8_t encRightDir = 1;

void IRAM_ATTR encLeftISR()  { encLeftCount  += encLeftDir; }
void IRAM_ATTR encRightISR() { encRightCount += encRightDir; }

class Encoder {
public:
    float posX = 0;
    float posY = 0;
    float totalDistCm = 0;
    float encoderHeading = 0;  // heading estimated from differential encoders (radians)

    void begin() {
        pinMode(ENCODER_L_PIN, INPUT_PULLUP);
        pinMode(ENCODER_R_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encLeftISR, RISING);
        attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encRightISR, RISING);
        encLeftCount = 0;
        encRightCount = 0;
        prevLeft = 0;
        prevRight = 0;
    }

    static void setDirection(int8_t dir) {
        encLeftDir = dir;
        encRightDir = dir;
    }

    static void setDirectionLR(int8_t left, int8_t right) {
        encLeftDir = left;
        encRightDir = right;
    }

    void update(float headingRad) {
        long left, right;
        portENTER_CRITICAL(&mux);
        left  = encLeftCount;
        right = encRightCount;
        portEXIT_CRITICAL(&mux);

        long dL = left - prevLeft;
        long dR = right - prevRight;
        prevLeft  = left;
        prevRight = right;

        // Differential drive odometry
        float distL = dL * CM_PER_TICK;
        float distR = dR * CM_PER_TICK;
        float distAvg = (distL + distR) / 2.0;

        // Total wheel travel (counts turns; pure spin still racks up cm).
        // Don't use distAvg here — turns make L=-R, distAvg=0, dashboard reads 0.
        totalDistCm += (abs(distL) + abs(distR)) / 2.0;

        // Update position using provided heading (from compass+gyro fusion)
        posX += distAvg * sin(headingRad);
        posY += distAvg * cos(headingRad);

        // Also compute heading change from encoder differential
        // (used as a sanity check, not primary heading source)
        float dTheta = (distR - distL) / WHEEL_BASE_CM;
        encoderHeading += dTheta;
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
        encoderHeading = 0;
    }

    long getLeftCount()  { return encLeftCount; }
    long getRightCount() { return encRightCount; }

private:
    long prevLeft = 0;
    long prevRight = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

#endif
