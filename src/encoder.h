#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// Dual wheel encoders — pure odometry (distance only)
// Direction comes from MPU6050 heading, not encoder ticks

volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

void IRAM_ATTR encLeftISR()  { encLeftCount++; }
void IRAM_ATTR encRightISR() { encRightCount++; }

class Encoder {
public:
    float posX = 0;
    float posY = 0;
    float totalDistCm = 0;

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

        float distL = dL * CM_PER_TICK;
        float distR = dR * CM_PER_TICK;
        float distAvg = (distL + distR) / 2.0;

        totalDistCm += distAvg;

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
    }

    long getLeftCount()  { return encLeftCount; }
    long getRightCount() { return encRightCount; }

private:
    long prevLeft = 0;
    long prevRight = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

#endif
