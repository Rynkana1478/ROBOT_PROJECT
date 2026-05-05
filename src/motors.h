#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

// Pure state machine — no blocking calls.
// Brain calls setState(...) to declare intent;
// tick() applies pins/PWM each loop based on stored state.
// Turn-kick is handled internally as a timestamped sub-state, NOT a delay().
class Motors {
public:
    enum State { M_STOP, M_FORWARD, M_BACKWARD, M_TURN_L, M_TURN_R, M_BRAKE };

    void begin() {
        pinMode(MOTOR_L_AIN1, OUTPUT);
        pinMode(MOTOR_L_AIN2, OUTPUT);
        pinMode(MOTOR_R_BIN1, OUTPUT);
        pinMode(MOTOR_R_BIN2, OUTPUT);
        pinMode(MOTOR_STBY, OUTPUT);

        ledcSetup(0, PWM_FREQ, PWM_BITS);
        ledcAttachPin(MOTOR_L_PWM, 0);
        ledcSetup(1, PWM_FREQ, PWM_BITS);
        ledcAttachPin(MOTOR_R_PWM, 1);

        digitalWrite(MOTOR_STBY, HIGH);
        _desiredState = M_STOP;
        _appliedState = M_STOP;
        _desiredSpeed = 0;
        _kickUntil = 0;
        applyPins(M_STOP, 0);
    }

    // Declare intent. Cheap — just stores the desired state.
    // Turn-kick is set up here but applied non-blockingly by tick().
    void setState(State s, int speed = -1) {
        if (speed < 0) {
            switch (s) {
                case M_FORWARD:  speed = SPEED_CRUISE; break;
                case M_BACKWARD: speed = SPEED_SLOW;   break;
                case M_TURN_L:
                case M_TURN_R:   speed = SPEED_TURN;   break;
                default:         speed = 0;            break;
            }
        }

        bool isTurnTransition = (s == M_TURN_L || s == M_TURN_R) &&
                                (_desiredState != s);

        _desiredState = s;
        _desiredSpeed = speed;

        if (isTurnTransition) {
            _kickUntil = millis() + TURN_KICK_MS;
        } else if (s != M_TURN_L && s != M_TURN_R) {
            _kickUntil = 0;
        }
    }

    // Convenience aliases — keep existing call sites working.
    void forward(int speed = SPEED_CRUISE)  { setState(M_FORWARD, speed); }
    void backward(int speed = SPEED_SLOW)   { setState(M_BACKWARD, speed); }
    void turnLeft(int speed = SPEED_TURN)   { setState(M_TURN_L, speed); }
    void turnRight(int speed = SPEED_TURN)  { setState(M_TURN_R, speed); }
    void brake()                             { setState(M_BRAKE, 0); }
    void stop()                              { setState(M_STOP, 0); }

    // Apply current desired state to hardware. Call every loop.
    // Handles non-blocking turn-kick internally.
    void tick() {
        State s = _desiredState;
        int  spd = _desiredSpeed;

        if ((s == M_TURN_L || s == M_TURN_R) && millis() < _kickUntil) {
            spd = TURN_KICK_SPEED;
        }

        if (s == _appliedState && spd == _appliedSpeed) return;
        applyPins(s, spd);
        _appliedState = s;
        _appliedSpeed = spd;
    }

    bool  isReversing() { return _desiredState == M_BACKWARD; }
    State getState()    { return _desiredState; }
    int   getSpeed()    { return _desiredSpeed; }

    void sleep()  { digitalWrite(MOTOR_STBY, LOW); }
    void wake()   { digitalWrite(MOTOR_STBY, HIGH); }

private:
    State _desiredState = M_STOP;
    State _appliedState = M_STOP;
    int   _desiredSpeed = 0;
    int   _appliedSpeed = 0;
    unsigned long _kickUntil = 0;

    void applyPins(State s, int speed) {
        switch (s) {
            case M_FORWARD:
                setLeft(speed);  setRight(speed);  break;
            case M_BACKWARD:
                setLeft(-speed); setRight(-speed); break;
            case M_TURN_L:
                setLeft(-speed); setRight(speed);  break;
            case M_TURN_R:
                setLeft(speed);  setRight(-speed); break;
            case M_BRAKE:
                digitalWrite(MOTOR_L_AIN1, HIGH);
                digitalWrite(MOTOR_L_AIN2, HIGH);
                digitalWrite(MOTOR_R_BIN1, HIGH);
                digitalWrite(MOTOR_R_BIN2, HIGH);
                ledcWrite(0, SPEED_MAX);
                ledcWrite(1, SPEED_MAX);
                break;
            case M_STOP:
            default:
                digitalWrite(MOTOR_L_AIN1, LOW);
                digitalWrite(MOTOR_L_AIN2, LOW);
                digitalWrite(MOTOR_R_BIN1, LOW);
                digitalWrite(MOTOR_R_BIN2, LOW);
                ledcWrite(0, 0);
                ledcWrite(1, 0);
                break;
        }
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
