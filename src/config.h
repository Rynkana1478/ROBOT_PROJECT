#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// ESP32-S3 4WD Robot Chassis Configuration
// Motor: TB6612FNG | Battery: 2S 18650 (7.4V)
// Heading: MPU6050 gyro
// ============================================

// --- TB6612FNG Motor Driver ---
#define MOTOR_L_AIN1  6
#define MOTOR_L_AIN2  5
#define MOTOR_L_PWM   4
#define MOTOR_R_BIN1  15
#define MOTOR_R_BIN2  16
#define MOTOR_R_PWM   17
#define MOTOR_STBY    7

// LEDC PWM config
#define PWM_FREQ      5000
#define PWM_BITS      10

// --- Ultrasonic Sensor (HC-SR04) ---
#define US_TRIG       18
#define US_ECHO       3

// --- Servo (ultrasonic sweep) ---
#define SERVO_PIN     9

// --- I2C Bus (MPU6050) ---
#define I2C_SDA       11
#define I2C_SCL       12

// --- MPU6050 ---
#define MPU6050_ADDR  0x68

// --- Wheel Encoders ---
#define ENCODER_L_PIN 13
#define ENCODER_R_PIN 14

// --- Battery Monitor ---
#define BATTERY_PIN   1
#define BATTERY_MAX_V 8.4
#define BATTERY_MIN_V 6.0

// --- Obstacle Thresholds (cm) ---
// OBSTACLE_CLOSE: brake-trigger distance. Sensor lag (~340 ms front refresh in
// SWEEP_NORMAL) plus brake coast at SPEED_SLOW means we need ~10-15 cm of
// runway between trigger and contact. Raised 15 -> 20 -> 25 progressively as
// real-world bumping showed momentum was beating the brake.
#define OBSTACLE_CLOSE  25
#define OBSTACLE_SLOW   30
#define OBSTACLE_CLEAR  40

// --- Motor Speeds (0-1023, 10-bit) ---
#define SPEED_STOP    0
#define SPEED_SLOW    500
#define SPEED_CRUISE  700
#define SPEED_MAX     1023

#define SPEED_TURN       700
#define TURN_KICK_SPEED  SPEED_MAX
#define TURN_KICK_MS     120

// --- Servo Angles ---
#define SERVO_LEFT    160
#define SERVO_CENTER  90
#define SERVO_RIGHT   20

// --- Grid ---
#define GRID_SIZE     20
#define CELL_SIZE_CM  5

// --- Timing ---
#define SCAN_INTERVAL_MS    50    // 20Hz sensor loop

// --- Sweep (5 angles) ---
#define SWEEP_STEPS     5
static const int SWEEP_ANGLES[SWEEP_STEPS] = {20, 55, 90, 125, 160};
#define SERVO_SETTLE_MS 80

// --- Heading ---
// MAX_RATE / JUMP_LIMIT were too tight: at SPEED_TURN the chassis spins
// >150 deg/s, which pegged the rate clamp and the jump filter rejected
// nearly every sample. Bumped to match a 500 deg/s MPU range so real
// rotations integrate correctly. See test 08 mode 6 for the diagnosis.
#define GYRO_DEADZONE       0.5
#define GYRO_MAX_RATE       250.0   // match MPU hardware range (was 150)
#define GYRO_JUMP_LIMIT     200.0   // allow real ramp-ups during turn-kick (was 50)
#define ACCEL_Z_MIN         2.0
#define ACCEL_Z_MAX         18.0

// --- Encoder ---
// WHEEL_BASE_CM is the EFFECTIVE skid-steer wheelbase, not the geometric
// distance between wheels. For a 4WD chassis where all four wheels skid
// sideways during tank turns, the effective pivot diameter is 2-3x the
// physical track width. Calibrated via test 08 mode 7 (50–52 cm range,
// physical track is ~17 cm).
#define TICKS_PER_REV       20
#define WHEEL_CIRCUMF_CM    20.74
#define CM_PER_TICK          1.037
#define WHEEL_BASE_CM       50.0

// --- Slip Detection (motor-state aware) ---
// Forward/backward state: encoder ticking but accelY ~ 0 = wheel spinning, body still
// Turn state:             encoder ticking but no recent gyro rotation = wheels slip
#define SLIP_ACCEL_THRESHOLD      0.30  // m/s^2 sustained forward accel signature
#define SLIP_GYRO_THRESHOLD       8.0   // deg/s — gyro must clear this to count as "rotating"
#define SLIP_GYRO_TIMEOUT_MS      600   // turn slip = no gyro rotation for this long
#define SLIP_DETECT_CYCLES        8     // 400ms at 20Hz before reducing traction
#define SLIP_RECOVER_RATE         0.15  // traction recovery per cycle
#define SLIP_DECAY_RATE           0.12  // traction decay per slip cycle
#define ACCEL_LOWPASS_ALPHA       0.20  // EMA factor for accel low-pass

// --- Stall Detection (motor commanded but encoder dead) ---
#define STALL_THRESHOLD_MS        500   // motor active + zero ticks > this = stall
#define STALL_TICK_THRESHOLD      1     // ticks below this counts as no motion

// --- Heading drift mitigation ---
#define HEADING_RECAL_STILL_MS    2000  // motors stopped this long → auto re-cal
#define HEADING_RECAL_GYRO_VAR    0.5   // deg/s variance below = robot truly still

// --- Breadcrumbs ---
#define MAX_CRUMBS          100

// --- Navigator ---
#define NAV_HEADING_TOLERANCE  3.0   // accept heading within this many deg of bearing
#define NAV_DRIFT_THRESHOLD    10.0  // re-enter TURN if drift exceeds during DRIVE
#define NAV_REACHED_CM         5.0
#define NAV_OBSTACLE_MARK_CM   80.0
#define OBSTACLE_DECAY_MS      30000
#define DECAY_CHECK_MS         5000
#define AVOID_REPEAT_LIMIT     3
#define AVOID_REPEAT_WINDOW_MS 5000

// Avoidance — incremental turn + check + drive:
//  1. Brake when distFront < OBSTACLE_CLOSE
//  2. Pick a side from distLeft / distRight
//  3. Turn AVOID_TURN_INCREMENT_DEG (30°) toward open side
//  4. Wait AVOID_CHECK_SETTLE_MS for a fresh sensor reading
//  5. If distFront is clear (> OBSTACLE_SLOW), drive forward for
//     AVOID_DRIVE_PAST_CM, then hand back to navigator (re-bear toward target)
//  6. If still blocked, increment another 30°. Up to AVOID_MAX_INCREMENTS
//     (4 × 30 = 120° total) before giving up to NAV_FAILED.
// AV_DRIVE_PAST uses encoder pose for distance; brakes + re-evaluates if a
// new obstacle appears (distFront < OBSTACLE_CLOSE) mid-drive.
#define AVOID_BYPASS_SPEED          400    // PWM during AV_DRIVE_PAST
#define AVOID_TURN_INCREMENT_DEG    30.0   // each turn step
#define AVOID_MAX_INCREMENTS        4      // hard cap on cumulative turn
#define AVOID_DRIVE_PAST_CM         50.0   // forward travel after front clears
                                           // 50 cm at 30° turn = 25 cm lateral
                                           // offset, enough to clear obstacles
                                           // wider than chassis half-width.
#define AVOID_CHECK_SETTLE_MS       400    // wait for fresh sweep before recheck

// Smooth-turn profile: proportional speed ramps from MIN at tolerance edge
// up to SPEED_TURN at FULL_DEG. STOP_TIME_MS models brake deceleration so
// we know when stopping distance equals current heading error -> brake now.
#define NAV_TURN_FULL_DEG      40.0  // err >= this -> full SPEED_TURN
#define NAV_TURN_MIN_SPEED     430   // smallest sustained turn PWM (above motor stall)
#define NAV_TURN_STOP_TIME_MS  220   // est. time from full turn to halt under brake

// Wall-following stuck detection (path-length / straight-line ratio)
#define WALL_STUCK_RATIO       3.0
#define WALL_STUCK_MIN_PATH_CM 100.0
#define WALL_STUCK_MIN_MS      30000

// Map-update gate: skip pushing sweep readings into the grid when the chassis
// is rotating, otherwise sweep angles and chassis heading are out of sync and
// the same wall gets painted as a fan of phantom obstacles around us.
#define MAP_GATE_GYRO_DEG_S    20.0

// Command queue (auto-mode only)
#define CMD_QUEUE_DEPTH        8

// --- WiFi / server ---
#include "secrets.h"

// Two FreeRTOS tasks on Core 0; rate degrades naturally if WiFi is slow
// Adaptive sync rate: manual needs snappy round-trips, auto can be lazy
// because the queue protocol holds commands until ack'd.
#define SYNC_INTERVAL_MANUAL_MS  50    // manual: 20Hz so button hold feels responsive
#define SYNC_INTERVAL_AUTO_MS    500   // auto: 2Hz; queue ack tolerates the delay
#define ULTRASONIC_INTERVAL_MS   33    // 30Hz ultrasonic fire rate
#define TELEMETRY_INTERVAL_MS    100   // include telemetry every Nth sync
#define HTTP_TIMEOUT_MS          1500  // LAN+Flask round-trips can spike to ~1s

// Deadman: no sync for this long → safety stop
#define MANUAL_DEADMAN_MS        300   // manual mode: drop to stop fast
#define AUTO_DEADMAN_MS          3000  // auto mode: longer grace period

// --- Debug ---
#define DEBUG_MODE    1

#endif
