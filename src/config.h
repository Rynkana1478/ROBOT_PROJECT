#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// ESP32-S3 4WD Robot Chassis Configuration
// Motor: TB6612FNG | Battery: 2S 18650 (7.4V)
// Heading: MPU6050 gyro (QMC5883L compass optional, auto-detected)
// ============================================

// --- TB6612FNG Motor Driver ---
#define MOTOR_L_AIN1  4    // Left direction 1
#define MOTOR_L_AIN2  5    // Left direction 2
#define MOTOR_L_PWM   6    // Left speed (LEDC)
#define MOTOR_R_BIN1  7    // Right direction 1
#define MOTOR_R_BIN2  15   // Right direction 2
#define MOTOR_R_PWM   16   // Right speed (LEDC)
#define MOTOR_STBY    17   // Standby (HIGH=active)

// LEDC PWM config
#define PWM_FREQ      5000  // 5kHz
#define PWM_BITS      10    // 10-bit (0-1023)

// --- Ultrasonic Sensor (HC-SR04) ---
#define US_TRIG       18
#define US_ECHO       3

// --- Servo (ultrasonic sweep) ---
#define SERVO_PIN     9

// --- I2C Bus (MPU6050 + QMC5883L) ---
#define I2C_SDA       11
#define I2C_SCL       12

// --- MPU6050 ---
#define MPU6050_ADDR  0x68

// --- QMC5883L Compass ---
#define COMPASS_ADDR  0x0D

// --- Wheel Encoders ---
#define ENCODER_L_PIN 13
#define ENCODER_R_PIN 14

// --- Battery Monitor ---
#define BATTERY_PIN   1     // ADC1 channel
#define BATTERY_MAX_V 8.4   // 2S fully charged
#define BATTERY_MIN_V 6.0   // 2S cutoff

// --- Obstacle Thresholds (cm) ---
#define OBSTACLE_CLOSE  15
#define OBSTACLE_SLOW   30
#define OBSTACLE_CLEAR  50

// --- Motor Speeds (0-1023, 10-bit) ---
#define SPEED_STOP    0
#define SPEED_SLOW    300
#define SPEED_MEDIUM  500
#define SPEED_FAST    700
#define SPEED_MAX     1023

// Turns need more torque than straight-line — TT motors have higher
// static friction in reverse, and with 4 motors in parallel any
// marginally-stiff one stalls while the others carry the current.
#define SPEED_TURN       850    // cruise speed during a spin
#define TURN_KICK_SPEED  SPEED_MAX
#define TURN_KICK_MS     100    // full-power pulse to break static friction

// --- Servo Angles ---
#define SERVO_LEFT    160
#define SERVO_CENTER  90
#define SERVO_RIGHT   20

// --- Pathfinding Grid ---
#define GRID_SIZE     40    // 40x40 grid (4m x 4m visible)
#define CELL_SIZE_CM  10    // 10cm per cell

// --- Timing ---
#define SCAN_INTERVAL_MS    50    // 20Hz sensor loop (ESP32 can handle it)
#define PATH_UPDATE_MS      500
#define SERVO_SWEEP_MS      120

// --- Heading ---
#define GYRO_DEADZONE       0.3   // deg/s - ignore below this
#define COMPASS_ALPHA       0.02  // complementary filter weight (0.02 = slow compass correction)

// --- Encoder ---
#define TICKS_PER_REV       20
#define WHEEL_CIRCUMF_CM    20.42  // PI * 6.5cm
#define CM_PER_TICK          1.021
#define WHEEL_BASE_CM       15.0   // distance between left and right wheels

// --- Breadcrumbs ---
#define MAX_CRUMBS          100    // 20m range (every 20cm)

// --- WiFi / server / token ---
// All deployment-specific values live in secrets.h (gitignored).
// Copy secrets.example.h -> secrets.h and fill it in before building.
#include "secrets.h"

#define REPORT_INTERVAL_MS  200

// --- Debug ---
#define DEBUG_MODE    1

#endif
