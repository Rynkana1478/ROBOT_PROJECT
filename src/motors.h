#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// TB6612FNG motor driver for 4WD chassis
// Separate direction pins (AIN1/AIN2) + PWM speed (PWMA)
// STBY pin must be HIGH for driver to be active
//
// Direction table:
//   AIN1=H AIN2=L → Forward
//   AIN1=L AIN2=H → Backward
//   AIN1=H AIN2=H → Brake
//   AIN1=L AIN2=L → Coast

class Motors {
public:
    void begin() {
        // Direction pins
        pinMode(MOTOR_L_AIN1, OUTPUT);
        pinMode(MOTOR_L_AIN2, OUTPUT);
        pinMode(MOTOR_R_BIN1, OUTPUT);
        pinMode(MOTOR_R_BIN2, OUTPUT);
        pinMode(MOTOR_STBY, OUTPUT);

        // PWM setup (ESP32 LEDC) - channel 0 for left, channel 1 for right
        ledcSetup(0, PWM_FREQ, PWM_BITS);
        ledcAttachPin(MOTOR_L_PWM, 0);
        ledcSetup(1, PWM_FREQ, PWM_BITS);
        ledcAttachPin(MOTOR_R_PWM, 1);

        // Enable driver
        digitalWrite(MOTOR_STBY, HIGH);

        stop();
    }

    void forward(int speed = SPEED_MEDIUM) {
        Encoder::setDirection(-1);
        setLeft(speed);
        setRight(speed);
    }

    void backward(int speed = SPEED_MEDIUM) {
        Encoder::setDirection(1);
        setLeft(-speed);
        setRight(-speed);
    }

    void turnLeft(int speed = SPEED_TURN) {
        Encoder::setDirectionLR(1, -1);
        kickAndCruise(-speed, speed);
    }

    void turnRight(int speed = SPEED_TURN) {
        Encoder::setDirectionLR(-1, 1);
        kickAndCruise(speed, -speed);
    }

    void curveLeft(int speed = SPEED_MEDIUM) {
        Encoder::setDirection(-1);
        setLeft(speed / 3);
        setRight(speed);
    }

    void curveRight(int speed = SPEED_MEDIUM) {
        Encoder::setDirection(-1);
        setLeft(speed);
        setRight(speed / 3);
    }

    void brake() {
        Encoder::setDirection(0);
        digitalWrite(MOTOR_L_AIN1, HIGH);
        digitalWrite(MOTOR_L_AIN2, HIGH);
        digitalWrite(MOTOR_R_BIN1, HIGH);
        digitalWrite(MOTOR_R_BIN2, HIGH);
        ledcWrite(0, SPEED_MAX);
        ledcWrite(1, SPEED_MAX);
    }

    void stop() {
        Encoder::setDirection(0);
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        digitalWrite(MOTOR_L_AIN1, LOW);
        digitalWrite(MOTOR_L_AIN2, LOW);
        digitalWrite(MOTOR_R_BIN1, LOW);
        digitalWrite(MOTOR_R_BIN2, LOW);
    }

    // Standby mode (low power, driver disabled)
    void sleep()  { digitalWrite(MOTOR_STBY, LOW); }
    void wake()   { digitalWrite(MOTOR_STBY, HIGH); }

private:
    // Full-power pulse to break static friction, then drop to cruise speed.
    // Stall-prone wheels otherwise sit still while the others carry all the current.
    void kickAndCruise(int leftCruise, int rightCruise) {
        int kickL = (leftCruise  > 0) ?  TURN_KICK_SPEED : (leftCruise  < 0 ? -TURN_KICK_SPEED : 0);
        int kickR = (rightCruise > 0) ?  TURN_KICK_SPEED : (rightCruise < 0 ? -TURN_KICK_SPEED : 0);
        setLeft(kickL);
        setRight(kickR);
        delay(TURN_KICK_MS);
        setLeft(leftCruise);
        setRight(rightCruise);
    }

    void setLeft(int speed) {
        if (speed > 0) {
            digitalWrite(MOTOR_L_AIN1, HIGH);
            digitalWrite(MOTOR_L_AIN2, LOW);
            ledcWrite(0, speed);
        } else if (speed < 0) {
            digitalWrite(MOTOR_L_AIN1, LOW);
            digitalWrite(MOTOR_L_AIN2, HIGH);
            ledcWrite(0, -speed);
        } else {
            digitalWrite(MOTOR_L_AIN1, LOW);
            digitalWrite(MOTOR_L_AIN2, LOW);
            ledcWrite(0, 0);
        }
    }

    void setRight(int speed) {
        if (speed > 0) {
            digitalWrite(MOTOR_R_BIN1, HIGH);
            digitalWrite(MOTOR_R_BIN2, LOW);
            ledcWrite(1, speed);
        } else if (speed < 0) {
            digitalWrite(MOTOR_R_BIN1, LOW);
            digitalWrite(MOTOR_R_BIN2, HIGH);
            ledcWrite(1, -speed);
        } else {
            digitalWrite(MOTOR_R_BIN1, LOW);
            digitalWrite(MOTOR_R_BIN2, LOW);
            ledcWrite(1, 0);
        }
    }
};

#endif
