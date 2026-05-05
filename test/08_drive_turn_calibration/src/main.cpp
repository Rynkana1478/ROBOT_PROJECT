// ============================================
// Test 08: Drive + Turn Calibration
//
// Verifies and helps recalibrate the two odometry constants in config.h:
//   CM_PER_TICK     (from drive 100cm test)
//   WHEEL_BASE_CM   (from turn 360 deg test)
//
// Hardware: full chassis (motors + dual encoders + MPU6050)
//
// Serial commands (single key + Enter):
//   1 = Drive 100 cm test  (forward at SPEED_SLOW until odometry says 100 cm)
//   2 = Turn 360 deg test  (spin right at SPEED_TURN until gyro reports 360)
//   3 = Drive custom: type "3 200" to drive 200 cm
//   4 = Turn  custom: type "4 90"  to turn  90 deg
//   r = reset encoders + heading
//   x = stop motors immediately
//   c = recalibrate gyro offset (keep robot still)
//
// After each test the [RESULT] line tells you the corrected config value to
// paste into src/config.h, e.g.:
//   [RESULT] DRIVE: target=96 ticks  actual=avg104  -> CM_PER_TICK = 0.962
//   [RESULT] TURN:  target=45 ticks  actual=avg58   -> WHEEL_BASE_CM = 19.13
// ============================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---- Pin & motor config (mirror src/config.h) ----
#define I2C_SDA       11
#define I2C_SCL       12

#define MOTOR_L_AIN1  6
#define MOTOR_L_AIN2  5
#define MOTOR_L_PWM   4
#define MOTOR_R_BIN1  15
#define MOTOR_R_BIN2  16
#define MOTOR_R_PWM   17
#define MOTOR_STBY    7

#define ENCODER_L_PIN 13
#define ENCODER_R_PIN 14

#define CH_L          0
#define CH_R          1
#define PWM_FREQ      5000
#define PWM_BITS      10

// ---- Calibration constants under test ----
#define TICKS_PER_REV      20
#define WHEEL_CIRCUMF_CM   20.74
#define CM_PER_TICK        1.037
#define WHEEL_BASE_CM      15.0

// ---- Speeds (mirror config) ----
#define SPEED_SLOW         500
#define SPEED_TURN         700
#define TURN_KICK_SPEED    1023
#define TURN_KICK_MS       120

// ---- Gyro filter (mirror sensors.h) ----
#define GYRO_DEADZONE      0.5
#define GYRO_MAX_RATE      150.0
#define GYRO_JUMP_LIMIT    50.0
#define ACCEL_Z_MIN        2.0
#define ACCEL_Z_MAX        18.0
#define ENC_DEBOUNCE_US    2000

// ---- Tolerance for pass/fail readout ----
#define DRIVE_TOLERANCE_PCT  10.0   // ±10% on tick count vs target
#define TURN_TOLERANCE_PCT   15.0   // turn is harder; allow more

// ============================================
// Encoders (with debounce, matching production firmware)
// ============================================
volatile long encL = 0;
volatile long encR = 0;
volatile unsigned long encLastL = 0;
volatile unsigned long encLastR = 0;

void IRAM_ATTR isrL() {
    unsigned long now = micros();
    if (now - encLastL > ENC_DEBOUNCE_US) { encL++; encLastL = now; }
}
void IRAM_ATTR isrR() {
    unsigned long now = micros();
    if (now - encLastR > ENC_DEBOUNCE_US) { encR++; encLastR = now; }
}

// ============================================
// Motors
// ============================================
void setLeft(int spd) {
    if (spd > 0)      { digitalWrite(MOTOR_L_AIN1, HIGH); digitalWrite(MOTOR_L_AIN2, LOW);  ledcWrite(CH_L,  spd); }
    else if (spd < 0) { digitalWrite(MOTOR_L_AIN1, LOW);  digitalWrite(MOTOR_L_AIN2, HIGH); ledcWrite(CH_L, -spd); }
    else              { digitalWrite(MOTOR_L_AIN1, LOW);  digitalWrite(MOTOR_L_AIN2, LOW);  ledcWrite(CH_L,    0); }
}
void setRight(int spd) {
    if (spd > 0)      { digitalWrite(MOTOR_R_BIN1, HIGH); digitalWrite(MOTOR_R_BIN2, LOW);  ledcWrite(CH_R,  spd); }
    else if (spd < 0) { digitalWrite(MOTOR_R_BIN1, LOW);  digitalWrite(MOTOR_R_BIN2, HIGH); ledcWrite(CH_R, -spd); }
    else              { digitalWrite(MOTOR_R_BIN1, LOW);  digitalWrite(MOTOR_R_BIN2, LOW);  ledcWrite(CH_R,    0); }
}
void stopMotors() { setLeft(0); setRight(0); }
void brakeMotors() {
    digitalWrite(MOTOR_L_AIN1, HIGH); digitalWrite(MOTOR_L_AIN2, HIGH);
    digitalWrite(MOTOR_R_BIN1, HIGH); digitalWrite(MOTOR_R_BIN2, HIGH);
    ledcWrite(CH_L, 1023); ledcWrite(CH_R, 1023);
}
void forward(int spd)  { setLeft(spd);  setRight(spd);  }
void spinRight(int spd){ setLeft(spd);  setRight(-spd); }
void spinLeft(int spd) { setLeft(-spd); setRight(spd);  }

// ============================================
// MPU6050 + heading
// ============================================
Adafruit_MPU6050 mpu;
float gyroZOffset = 0;
float headingDeg  = 0;
float prevGyroZ   = 0;
unsigned long lastHeadingMs = 0;
bool mpuReady = false;

void calibrateGyro() {
    Serial.println(F("[CAL] Calibrating gyro... keep still 2s"));
    if (!mpuReady) { Serial.println(F("[CAL] MPU not ready, skipping")); return; }
    sensors_event_t a, g, t;
    float sum = 0; int count = 0;
    unsigned long start = millis();
    while (millis() - start < 2000) {
        if (mpu.getEvent(&a, &g, &t)) { sum += g.gyro.z; count++; }
        delay(5);
    }
    gyroZOffset = (count > 0) ? sum / count : 0;
    Serial.printf("[CAL] offset=%.4f rad/s (%.2f deg/s) samples=%d\n",
                  gyroZOffset, gyroZOffset * 180.0 / PI, count);
}

void resetHeading() { headingDeg = 0; prevGyroZ = 0; lastHeadingMs = millis(); }

// Update heading from gyro. Same filter chain as sensors.h.
void updateHeading() {
    if (!mpuReady) return;
    unsigned long now = millis();
    if (lastHeadingMs == 0) { lastHeadingMs = now; return; }
    float dt = (now - lastHeadingMs) / 1000.0;
    lastHeadingMs = now;

    sensors_event_t a, g, t;
    if (!mpu.getEvent(&a, &g, &t)) return;

    float az = fabs(a.acceleration.z);
    if (az < ACCEL_Z_MIN || az > ACCEL_Z_MAX) return;

    // Sign-flipped: this chassis's MPU mount has CW (physical right turn)
    // produce NEGATIVE g.gyro.z. We want heading to INCREASE on right turn
    // to match calcBearing convention (0=N, 90=E, increasing CW). Flip here.
    float gz = -(g.gyro.z - gyroZOffset) * 180.0 / PI;
    if (fabs(gz - prevGyroZ) > GYRO_JUMP_LIMIT) gz = prevGyroZ;
    if (fabs(gz) > GYRO_MAX_RATE)               gz = prevGyroZ;
    if (fabs(gz) < GYRO_DEADZONE)               gz = 0;
    prevGyroZ = gz;
    headingDeg += gz * dt;   // unwrapped! we want total rotation, not [0,360)
}

// ============================================
// DRIVE TEST: forward until target ticks reached
// ============================================
void runDriveTest(float targetCm) {
    int targetTicks = (int)(targetCm / CM_PER_TICK + 0.5);
    Serial.println();
    Serial.println(F("======================================"));
    Serial.printf( "  DRIVE TEST: target = %.1f cm  (= %d ticks at CM_PER_TICK=%.3f)\n",
                   targetCm, targetTicks, CM_PER_TICK);
    Serial.println(F("======================================"));
    Serial.println(F("  Place robot at start mark. Starting in 3s..."));
    delay(3000);

    encL = 0; encR = 0;
    long startL = encL, startR = encR;

    forward(SPEED_SLOW);
    Serial.printf("  Driving at PWM %d...\n", SPEED_SLOW);

    unsigned long start = millis();
    long avgTicks = 0;
    while (avgTicks < targetTicks && millis() - start < 15000) {
        long dL = encL - startL;
        long dR = encR - startR;
        avgTicks = (dL + dR) / 2;
        if ((millis() - start) % 500 < 20) {
            Serial.printf("    t=%.1fs  L=%ld  R=%ld  avg=%ld\n",
                          (millis() - start) / 1000.0, dL, dR, avgTicks);
            delay(20);
        }
        delay(5);
    }

    brakeMotors();
    delay(400);                  // let momentum dissipate
    stopMotors();

    long dL = encL - startL;
    long dR = encR - startR;
    long avg = (dL + dR) / 2;
    float estCm = avg * CM_PER_TICK;
    float pctErr = 100.0 * (avg - targetTicks) / targetTicks;

    Serial.println();
    Serial.println(F("--------------------------------------"));
    Serial.printf( "  Target ticks      : %d\n", targetTicks);
    Serial.printf( "  Actual ticks      : L=%ld  R=%ld  avg=%ld\n", dL, dR, avg);
    Serial.printf( "  Wheel ratio L/R   : %.2f (1.0 = identical)\n",
                   dR > 0 ? (float)dL / dR : 0.0);
    Serial.printf( "  Estimated dist    : %.1f cm (per current CM_PER_TICK)\n", estCm);
    Serial.printf( "  Tick error vs tgt : %+.1f%%\n", pctErr);

    if (fabs(pctErr) < DRIVE_TOLERANCE_PCT) {
        Serial.println(F("  PASS: within tolerance, calibration looks correct"));
    } else {
        Serial.println(F("  FAIL: outside tolerance, calibration off"));
    }
    Serial.println();
    Serial.printf( "  >> Measure ACTUAL cm traveled. Then:\n");
    Serial.printf( "     new CM_PER_TICK    = actual_cm / %ld\n", avg);
    Serial.printf( "     new WHEEL_CIRCUMF  = new_CM_PER_TICK * %d\n", TICKS_PER_REV);
    Serial.printf( "  [RESULT] DRIVE target=%d actual=%ld diff=%+.1f%%\n",
                   targetTicks, avg, pctErr);
    Serial.println(F("======================================"));
}

// ============================================
// TURN TEST: spin right until gyro reports targetDeg
// ============================================
void runTurnTest(float targetDeg) {
    // Expected ticks per wheel for a 360 deg in-place tank turn:
    //   each wheel rolls along a circle of diameter = WHEEL_BASE_CM
    //   distance per wheel = pi * WHEEL_BASE_CM
    //   ticks = distance / CM_PER_TICK
    float fullTurnTicks = (PI * WHEEL_BASE_CM) / CM_PER_TICK;
    int targetTicks = (int)(fullTurnTicks * (targetDeg / 360.0) + 0.5);

    Serial.println();
    Serial.println(F("======================================"));
    Serial.printf( "  TURN TEST: rotate %.1f deg clockwise  (expect %d ticks/wheel\n",
                   targetDeg, targetTicks);
    Serial.printf( "                                       at WHEEL_BASE_CM=%.2f, CM_PER_TICK=%.3f)\n",
                   WHEEL_BASE_CM, CM_PER_TICK);
    Serial.println(F("======================================"));
    Serial.println(F("  Place on flat surface, clear of obstacles. Starting in 3s..."));
    delay(3000);

    if (!mpuReady) {
        Serial.println(F("  ABORT: MPU6050 not ready"));
        return;
    }

    calibrateGyro();
    resetHeading();
    encL = 0; encR = 0;
    long startL = encL, startR = encR;

    // turn-kick to break static friction, then steady SPEED_TURN
    spinRight(TURN_KICK_SPEED);
    unsigned long start = millis();
    unsigned long kickUntil = start + TURN_KICK_MS;

    Serial.println(F("  Spinning right..."));

    while (fabs(headingDeg) < targetDeg && millis() - start < 20000) {
        updateHeading();
        if (millis() > kickUntil) spinRight(SPEED_TURN);
        if ((millis() - start) % 300 < 15) {
            Serial.printf("    t=%.1fs  heading=%.1f deg  L=%ld  R=%ld\n",
                          (millis() - start) / 1000.0, headingDeg,
                          encL - startL, encR - startR);
            delay(15);
        }
        delay(5);
    }

    brakeMotors();
    // After brake, momentum continues — keep integrating heading for ~500ms.
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < 500) { updateHeading(); delay(5); }
    stopMotors();

    long dL = encL - startL;
    long dR = encR - startR;
    long avg = (dL + dR) / 2;
    float pctErr = 100.0 * (avg - targetTicks) / (float)targetTicks;

    // Inferred wheelbase from observed ticks vs gyro-confirmed degrees.
    // tank-turn formula reversed: WHEEL_BASE = (avg_ticks * CM_PER_TICK) / pi,
    // scaled to 360deg if we didn't actually complete a full revolution.
    float headingFraction = fabs(headingDeg) / 360.0;
    float wheelDistFor360 = (avg * CM_PER_TICK) / max(headingFraction, 0.001f);
    float inferredWheelbase = wheelDistFor360 / PI;

    float rotatedDeg = fabs(headingDeg);

    Serial.println();
    Serial.println(F("--------------------------------------"));
    Serial.printf( "  Requested rotation : %.1f deg clockwise\n", targetDeg);
    Serial.printf( "  Actual rotation    : %.1f deg (gyro-integrated)\n", rotatedDeg);
    Serial.printf( "  Target ticks/whl   : %d\n", targetTicks);
    Serial.printf( "  Actual ticks       : L=%ld  R=%ld  avg=%ld\n", dL, dR, avg);
    Serial.printf( "  Wheel ratio L/R    : %.2f (1.0 = identical)\n",
                   dR > 0 ? (float)dL / dR : 0.0);
    Serial.printf( "  Tick error vs tgt  : %+.1f%%\n", pctErr);
    Serial.printf( "  Inferred WHEEL_BASE_CM = %.2f (config says %.2f)\n",
                   inferredWheelbase, (float)WHEEL_BASE_CM);

    if (fabs(pctErr) < TURN_TOLERANCE_PCT) {
        Serial.println(F("  PASS: within tolerance, WHEEL_BASE_CM looks correct"));
    } else {
        Serial.println(F("  FAIL: outside tolerance — update WHEEL_BASE_CM"));
    }
    Serial.println();
    Serial.printf( "  >> If gyro overshot/undershot, also check the gyro itself.\n");
    Serial.printf( "  [RESULT] TURN req=%.1f rotated=%.1f ticks=%ld diff=%+.1f%% inferred_wheelbase=%.2f\n",
                   targetDeg, rotatedDeg, avg, pctErr, inferredWheelbase);
    Serial.println(F("======================================"));
}

// ============================================
// ENCODER-DRIVEN TURN: spin until ticks-for-target-angle hit, gyro is witness
// ============================================
void runEncoderTurnTest(float targetDeg) {
    // Inverse of runTurnTest: pick ticks first, let gyro just observe.
    float fullTurnTicks = (PI * WHEEL_BASE_CM) / CM_PER_TICK;
    int targetTicks = (int)(fullTurnTicks * (targetDeg / 360.0) + 0.5);

    Serial.println();
    Serial.println(F("======================================"));
    Serial.printf( "  ENC-TURN TEST: rotate %.1f deg clockwise  (drive %d ticks/wheel)\n",
                   targetDeg, targetTicks);
    Serial.printf( "                 using WHEEL_BASE_CM=%.2f, CM_PER_TICK=%.3f\n",
                   (float)WHEEL_BASE_CM, (float)CM_PER_TICK);
    Serial.println(F("======================================"));
    Serial.println(F("  Place on flat surface. Starting in 3s..."));
    delay(3000);

    if (mpuReady) { calibrateGyro(); resetHeading(); }
    encL = 0; encR = 0;
    long startL = encL, startR = encR;

    spinRight(TURN_KICK_SPEED);
    unsigned long start = millis();
    unsigned long kickUntil = start + TURN_KICK_MS;
    long avg = 0;

    while (avg < targetTicks && millis() - start < 15000) {
        if (mpuReady) updateHeading();
        if (millis() > kickUntil) spinRight(SPEED_TURN);
        long dL = encL - startL, dR = encR - startR;
        avg = (dL + dR) / 2;
        if ((millis() - start) % 300 < 15) {
            Serial.printf("    t=%.1fs  L=%ld  R=%ld  avg=%ld/%d  gyroHeading=%.1f\n",
                          (millis() - start) / 1000.0, dL, dR, avg, targetTicks, headingDeg);
            delay(15);
        }
        delay(5);
    }

    brakeMotors();
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < 500) { if (mpuReady) updateHeading(); delay(5); }
    stopMotors();

    long dL = encL - startL;
    long dR = encR - startR;
    long avgFinal = (dL + dR) / 2;
    float ratio = dR > 0 ? (float)dL / dR : 0.0;

    // Gyro check: if encoder & gyro agree, both calibrations are good.
    // If they disagree, the discrepancy tells us which is wrong.
    float gyroDegPerTick = (avgFinal > 0) ? fabs(headingDeg) / avgFinal : 0;
    float expectedDegPerTick = targetDeg / (float)targetTicks;
    float ratio_g_e = (expectedDegPerTick > 0) ? gyroDegPerTick / expectedDegPerTick : 0;

    float rotatedDeg = fabs(headingDeg);

    Serial.println();
    Serial.println(F("--------------------------------------"));
    Serial.printf( "  Requested rotation : %.1f deg clockwise\n", targetDeg);
    Serial.printf( "  Driving until      : %d ticks/wheel reached\n", targetTicks);
    Serial.printf( "  Actual ticks       : L=%ld  R=%ld  avg=%ld\n", dL, dR, avgFinal);
    Serial.printf( "  Wheel ratio L/R    : %.2f\n", ratio);
    Serial.printf( "  Gyro witnessed     : %.1f deg of rotation\n", rotatedDeg);
    Serial.printf( "  Per-cfg expected   : %.1f deg for %ld ticks\n",
                   targetDeg * avgFinal / targetTicks, avgFinal);
    Serial.printf( "  Gyro/encoder ratio : %.2f  (1.0 = both agree)\n", ratio_g_e);
    Serial.println();
    Serial.println(F("  Now use a protractor / mark + measure to get TRUE degrees turned."));
    Serial.println(F("  -> If gyro > true deg : gyro over-reads"));
    Serial.println(F("  -> If gyro < true deg : gyro under-reads (filter dropping samples)"));
    Serial.println(F("  -> If encoder ticks match true deg conversion : WHEEL_BASE_CM is correct"));
    Serial.printf( "  [RESULT] ENC-TURN req=%.1f ticks=%ld rotated=%.1f ratio=%.2f\n",
                   targetDeg, avgFinal, rotatedDeg, ratio_g_e);
    Serial.println(F("======================================"));
}

// ============================================
// GYRO BURN-IN: spin 3s, dump every sample with filter status
// ============================================
void runGyroBurnIn() {
    Serial.println();
    Serial.println(F("======================================"));
    Serial.println(F("  GYRO BURN-IN: spinning 3s, dumping every sample"));
    Serial.println(F("======================================"));

    if (!mpuReady) { Serial.println(F("  ABORT: MPU not ready")); return; }
    calibrateGyro();
    resetHeading();
    encL = 0; encR = 0;

    int total = 0, rejAccel = 0, rejJump = 0, rejMax = 0, accepted = 0;
    long startL = encL, startR = encR;

    spinRight(TURN_KICK_SPEED);
    unsigned long start = millis();
    unsigned long kickUntil = start + TURN_KICK_MS;
    float prevGz = 0;
    unsigned long lastMs = millis();

    Serial.println(F("  t_ms | raw_gz   filt_gz  accel_z  reject  hdg_deg"));
    while (millis() - start < 3000) {
        if (millis() > kickUntil) spinRight(SPEED_TURN);
        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0;
        lastMs = now;

        sensors_event_t a, g, t;
        bool gotEvent = mpu.getEvent(&a, &g, &t);
        if (!gotEvent) { rejAccel++; total++; delay(5); continue; }
        total++;

        float az = fabs(a.acceleration.z);
        float rawGz = -(g.gyro.z - gyroZOffset) * 180.0 / PI;  // sign-flipped, see updateHeading
        float filtGz = rawGz;
        const char* rej = "OK";

        if (az < ACCEL_Z_MIN || az > ACCEL_Z_MAX) {
            filtGz = 0; rejAccel++; rej = "ACCEL_Z";
        } else {
            if (fabs(rawGz - prevGz) > GYRO_JUMP_LIMIT) { filtGz = prevGz; rejJump++; rej = "JUMP"; }
            else if (fabs(rawGz) > GYRO_MAX_RATE)       { filtGz = prevGz; rejMax++;  rej = "MAX_RATE"; }
            else if (fabs(rawGz) < GYRO_DEADZONE)       { filtGz = 0; rej = "DEADZONE"; }
            else { accepted++; }
            prevGz = filtGz;
        }
        headingDeg += filtGz * dt;

        Serial.printf("  %5lu | %+7.1f  %+7.1f  %5.2f  %-9s  %+.1f\n",
                      now - start, rawGz, filtGz, az, rej, headingDeg);
        delay(5);
    }

    brakeMotors();
    delay(400);
    stopMotors();

    long dL = encL - startL, dR = encR - startR;
    Serial.println();
    Serial.println(F("  ----- SUMMARY -----"));
    Serial.printf( "  total samples : %d\n", total);
    Serial.printf( "  accepted      : %d  (%.0f%%)\n", accepted, 100.0 * accepted / total);
    Serial.printf( "  rejected ACCEL: %d  (%.0f%%)\n", rejAccel, 100.0 * rejAccel / total);
    Serial.printf( "  rejected JUMP : %d  (%.0f%%)  <-- watch this\n", rejJump, 100.0 * rejJump / total);
    Serial.printf( "  rejected MAX  : %d  (%.0f%%)  <-- watch this\n", rejMax,  100.0 * rejMax / total);
    Serial.printf( "  encoder ticks : L=%ld  R=%ld\n", dL, dR);
    Serial.printf( "  gyro heading  : %.1f deg\n", headingDeg);
    Serial.println();
    Serial.println(F("  If JUMP or MAX > 20%, the filter is killing rotation samples."));
    Serial.println(F("  In sensors.h: bump GYRO_JUMP_LIMIT to 200 and GYRO_MAX_RATE to 250."));
    Serial.println(F("======================================"));
}

// ============================================
// SLOW + WIDE-RANGE TURN TEST: SPEED_SLOW + 500 deg/s MPU range
// Isolates whether the gyro saturation/filter is the issue
// ============================================
void runSlowTurnTest(float targetDeg) {
    Serial.println();
    Serial.println(F("======================================"));
    Serial.printf( "  SLOW-TURN TEST: %.1f deg at SPEED_SLOW(%d) + MPU range 500 deg/s\n",
                   targetDeg, SPEED_SLOW);
    Serial.println(F("======================================"));
    Serial.println(F("  Starting in 3s..."));
    delay(3000);

    if (!mpuReady) { Serial.println(F("  ABORT: MPU not ready")); return; }

    // Switch to 500 deg/s range so saturation isn't an issue.
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println(F("  MPU range -> 500 deg/s"));
    calibrateGyro();
    resetHeading();
    encL = 0; encR = 0;
    long startL = encL, startR = encR;

    spinRight(SPEED_SLOW);   // no kick — sustained slow turn
    unsigned long start = millis();

    while (fabs(headingDeg) < targetDeg && millis() - start < 30000) {
        updateHeading();
        if ((millis() - start) % 500 < 20) {
            Serial.printf("    t=%.1fs  heading=%.1f deg  L=%ld  R=%ld\n",
                          (millis() - start) / 1000.0, headingDeg,
                          encL - startL, encR - startR);
            delay(20);
        }
        delay(5);
    }

    brakeMotors();
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < 500) { updateHeading(); delay(5); }
    stopMotors();

    // Restore default range.
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    long dL = encL - startL, dR = encR - startR;
    long avg = (dL + dR) / 2;
    float fullTurnTicks = (PI * WHEEL_BASE_CM) / CM_PER_TICK;
    int expectedTicks = (int)(fullTurnTicks * (targetDeg / 360.0) + 0.5);
    float gyroErrPct = 100.0 * (fabs(headingDeg) - targetDeg) / targetDeg;
    float wheelDistFor360 = (avg * CM_PER_TICK) / max(fabs(headingDeg) / 360.0f, 0.001f);
    float inferredWheelbase = wheelDistFor360 / PI;

    float rotatedDeg = fabs(headingDeg);

    Serial.println();
    Serial.println(F("--------------------------------------"));
    Serial.printf( "  Requested rotation : %.1f deg clockwise (relative)\n", targetDeg);
    Serial.printf( "  Actual rotation    : %.1f deg  (gyro err %+.1f%%)\n",
                   rotatedDeg, gyroErrPct);
    Serial.printf( "  Encoder ticks      : L=%ld R=%ld avg=%ld\n", dL, dR, avg);
    Serial.printf( "  Expected per cfg   : %d ticks  (this # comes from current WHEELBASE,\n", expectedTicks);
    Serial.printf( "                       not a target — used only to spot bad config)\n");
    Serial.println();
    Serial.printf( "  >> CALIBRATION RESULT <<\n");
    Serial.printf( "     Inferred WHEEL_BASE_CM = %.2f   (current config: %.2f)\n",
                   inferredWheelbase, (float)WHEEL_BASE_CM);
    if (fabs(gyroErrPct) < 10.0)
        Serial.println(F("     Gyro tracked within 10% of requested rotation -> trustworthy."));
    else
        Serial.println(F("     Gyro off >10% from rotation -> check filter / MPU range."));
    Serial.printf( "  [RESULT] SLOW-TURN req=%.1f rotated=%.1f ticks=%ld inferred_wheelbase=%.2f\n",
                   targetDeg, rotatedDeg, avg, inferredWheelbase);
    Serial.println(F("======================================"));
}

// ============================================
// ABS-HEADING TEST: mirrors production navigator's bearing-follow logic.
// Resets heading to 0 (= "current physical orientation"), then spins to
// reach the requested absolute heading using the SAME math as
// navigator.h: smoothTurnStep — proportional speed, sign-of-err picks
// direction. Lets us catch a gyro-sign mismatch in isolation.
// ============================================
void runAbsHeadingTest(float targetDeg) {
    // Normalize target to [0, 360)
    while (targetDeg < 0)    targetDeg += 360;
    while (targetDeg >= 360) targetDeg -= 360;

    Serial.println();
    Serial.println(F("======================================"));
    Serial.printf( "  ABS-HEADING TEST: turn to absolute %.1f deg\n", targetDeg);
    Serial.println(F("  (mimics production navigator bearing-follow logic)"));
    Serial.println(F("======================================"));
    Serial.println(F("  Heading PERSISTS across mode 8 runs (real absolute frame)."));
    Serial.println(F("  Use 'r' ONCE to set current direction = 0. After that, every"));
    Serial.println(F("  '8 X' turns to absolute X regardless of where you are now."));
    Serial.println(F("  Example sequence after 'r':"));
    Serial.println(F("    8 90  -> face east  (90 CW from your declared north)"));
    Serial.println(F("    8 270 -> face west  (90 CCW from declared north,"));
    Serial.println(F("                         or 180 from east)"));
    Serial.println(F("    8 0   -> face back to declared north"));
    Serial.println(F("  Starting in 3s..."));
    delay(3000);

    if (!mpuReady) { Serial.println(F("  ABORT: MPU not ready")); return; }
    // NOTE: do NOT call resetHeading() here. We want heading to PERSIST across
    // mode 8 runs so the target really is absolute. Use 'r' once at the start
    // to declare "current direction = 0", then every '8 X' turns to absolute X.
    Serial.printf("  Starting heading: %.1f deg (this is your current 'absolute' frame)\n",
                  headingDeg);
    encL = 0; encR = 0;

    int firstDirection = 0;       // 1 = right, -1 = left
    bool firstStep = true;
    unsigned long start = millis();

    while (millis() - start < 20000) {
        updateHeading();

        // Wrap headingDeg to [0, 360) for compare to absolute target
        float h = headingDeg;
        while (h < 0)    h += 360;
        while (h >= 360) h -= 360;

        // Same wrap-aware error as navigator's angleDiff()
        float err = targetDeg - h;
        while (err > 180)  err -= 360;
        while (err < -180) err += 360;

        if (fabs(err) < 5.0) {
            brakeMotors();
            break;
        }

        if (firstStep) {
            firstDirection = (err > 0) ? 1 : -1;
            firstStep = false;
            Serial.printf("  Initial err=%.1f -> commanding %s turn first\n",
                          err, firstDirection > 0 ? "RIGHT" : "LEFT");
        }

        if (err > 0) spinRight(SPEED_SLOW);
        else         spinLeft(SPEED_SLOW);
        delay(10);
    }

    unsigned long brakeStart = millis();
    while (millis() - brakeStart < 500) { updateHeading(); delay(5); }
    stopMotors();

    float h = headingDeg;
    while (h < 0)    h += 360;
    while (h >= 360) h -= 360;

    Serial.println();
    Serial.println(F("--------------------------------------"));
    Serial.printf( "  Requested abs heading : %.1f deg\n", targetDeg);
    Serial.printf( "  Final heading variable: %.1f deg  (err vs target = %+.1f)\n",
                   h, h - targetDeg);
    Serial.printf( "  Initial commanded dir : %s turn\n",
                   firstDirection > 0 ? "RIGHT (CW)" : "LEFT (CCW)");
    Serial.println();
    Serial.println(F("  >> CHECK PHYSICALLY: <<"));
    Serial.println(F("     Did the chassis end up FACING the target direction?"));
    Serial.println(F("       (for target=90 from north, chassis should face east, NOT west)"));
    Serial.println(F("     Did it take the SHORT way?"));
    Serial.println(F("       (target=90 should be ~quarter turn, not three-quarter)"));
    Serial.println();
    Serial.println(F("  IF chassis took the LONG way OR faces wrong direction:"));
    Serial.println(F("    --> gyro sign is inverted in production sensors.h"));
    Serial.println(F("    --> fix: gz = -((g.gyro.z - gyroZOffset) * 180.0 / PI);"));
    Serial.printf( "  [RESULT] ABS-HEADING target=%.1f finalHdg=%.1f firstDir=%s\n",
                   targetDeg, h, firstDirection > 0 ? "RIGHT" : "LEFT");
    Serial.println(F("======================================"));
}

// ============================================
// Setup + Menu
// ============================================
void printMenu() {
    Serial.println();
    Serial.println(F("##############################################"));
    Serial.println(F("#  Test 08: Drive + Turn Calibration         #"));
    Serial.println(F("##############################################"));
    Serial.println(F("  All 'deg' values are RELATIVE rotation amount, not absolute heading."));
    Serial.println(F("  All turns are clockwise (right). All distances are forward."));
    Serial.println();
    Serial.println(F("  1            -> Drive 100 cm forward"));
    Serial.println(F("  2            -> Rotate 360 deg CW  (gyro-driven, may fail at high speed)"));
    Serial.println(F("  3 <cm>       -> Drive custom forward distance (e.g. '3 50')"));
    Serial.println(F("  4 <deg>      -> Rotate custom angle CW (gyro-driven)"));
    Serial.println(F("  5 <deg>      -> Rotate by encoder ticks; gyro witnesses (e.g. '5 360')"));
    Serial.println(F("  6            -> Gyro burn-in 3s — dump every sample + filter rejects"));
    Serial.println(F("  7 <deg>      -> SLOW rotation at SPEED_SLOW + 500 deg/s MPU range"));
    Serial.println(F("  8 <deg>      -> Turn TO absolute heading (heading PERSISTS — use 'r'"));
    Serial.println(F("                  once to declare current direction = 0)"));
    Serial.println(F("  r            -> reset encoders + heading"));
    Serial.println(F("  x            -> emergency stop"));
    Serial.println(F("  c            -> recalibrate gyro offset"));
    Serial.println(F("  ?            -> show this menu"));
    Serial.println(F("##############################################"));
    Serial.printf ( "  Current config: CM_PER_TICK=%.3f  WHEEL_BASE_CM=%.2f\n",
                    (float)CM_PER_TICK, (float)WHEEL_BASE_CM);
    Serial.printf ( "                  TICKS_PER_REV=%d   WHEEL_CIRCUMF=%.2fcm\n",
                    TICKS_PER_REV, (float)WHEEL_CIRCUMF_CM);
    Serial.println(F("##############################################"));
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Motor pins
    pinMode(MOTOR_L_AIN1, OUTPUT); pinMode(MOTOR_L_AIN2, OUTPUT);
    pinMode(MOTOR_R_BIN1, OUTPUT); pinMode(MOTOR_R_BIN2, OUTPUT);
    pinMode(MOTOR_STBY,   OUTPUT);
    ledcSetup(CH_L, PWM_FREQ, PWM_BITS); ledcAttachPin(MOTOR_L_PWM, CH_L);
    ledcSetup(CH_R, PWM_FREQ, PWM_BITS); ledcAttachPin(MOTOR_R_PWM, CH_R);
    digitalWrite(MOTOR_STBY, HIGH);
    stopMotors();

    // Encoders
    pinMode(ENCODER_L_PIN, INPUT_PULLUP);
    pinMode(ENCODER_R_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), isrR, RISING);

    // I2C + MPU6050
    Wire.begin(I2C_SDA, I2C_SCL);
    if (mpu.begin(0x68, &Wire)) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        mpuReady = true;
        Serial.println(F("[MPU] ready"));
        calibrateGyro();
    } else {
        Serial.println(F("[MPU] FAIL — turn test will be unavailable"));
    }

    printMenu();
}

void loop() {
    if (!Serial.available()) { delay(20); return; }

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    char cmd = line.charAt(0);
    String arg = (line.length() > 1) ? line.substring(1) : String("");
    arg.trim();

    switch (cmd) {
        case '1': runDriveTest(100.0); break;
        case '2': runTurnTest(360.0);  break;
        case '3': {
            float cm = arg.length() > 0 ? arg.toFloat() : 100.0;
            if (cm <= 0) cm = 100.0;
            runDriveTest(cm);
            break;
        }
        case '4': {
            float deg = arg.length() > 0 ? arg.toFloat() : 360.0;
            if (deg <= 0) deg = 360.0;
            runTurnTest(deg);
            break;
        }
        case '5': {
            float deg = arg.length() > 0 ? arg.toFloat() : 360.0;
            if (deg <= 0) deg = 360.0;
            runEncoderTurnTest(deg);
            break;
        }
        case '6':
            runGyroBurnIn();
            break;
        case '7': {
            float deg = arg.length() > 0 ? arg.toFloat() : 360.0;
            if (deg <= 0) deg = 360.0;
            runSlowTurnTest(deg);
            break;
        }
        case '8': {
            float deg = arg.length() > 0 ? arg.toFloat() : 90.0;
            runAbsHeadingTest(deg);
            break;
        }
        case 'r':
            encL = 0; encR = 0; resetHeading();
            Serial.println(F("[RESET] encoders + heading cleared"));
            break;
        case 'x':
            stopMotors();
            Serial.println(F("[STOP] motors halted"));
            break;
        case 'c':
            stopMotors();
            calibrateGyro();
            break;
        case '?':
            printMenu();
            break;
        default:
            Serial.printf("[?] unknown command: '%c'\n", cmd);
    }
}
