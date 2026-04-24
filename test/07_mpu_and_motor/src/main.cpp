// ============================================
// Test 07: MPU6050 + TB6612FNG Combined
// Diagnose gyro behavior while motors are running
//
// Serial commands (type + Enter):
//   w = forward    s = backward
//   a = spin left  d = spin right
//   q = left only  e = right only
//   x = stop       b = brake
//   1 = speed LOW (300)
//   2 = speed MED (500)
//   3 = speed HIGH (700)
//   4 = speed MAX (1023)
//   r = reset heading to 0
//   c = recalibrate gyro (keep still!)
//   p = pause/resume output
//
// Output (every 200ms):
//   [DATA] time,heading,gyroZ,accelZ,bad_i2c,motor_state,speed
// ============================================

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- Pin config (matches config.h) ---
#define I2C_SDA   11
#define I2C_SCL   12
#define AIN1      6
#define AIN2      5
#define PWMA      4
#define BIN1      15
#define BIN2      16
#define PWMB      17
#define STBY      7

#define CH_L  0
#define CH_R  1

// --- Gyro tuning (same as main firmware) ---
#define GYRO_DEADZONE     0.3
#define GYRO_MAX_RATE     150.0
#define GYRO_JUMP_LIMIT   50.0
#define ACCEL_Z_MIN       2.0
#define ACCEL_Z_MAX       18.0

Adafruit_MPU6050 mpu;
float gyroZOffset = 0;
float heading = 0;
float prevGyroZ = 0;
int badI2C = 0;
int totalReads = 0;
int motorSpeed = 500;
const char* motorState = "STOP";
bool outputPaused = false;

// --- Raw stats (unfiltered, for comparison) ---
float rawGz = 0;

// Motor helpers
void stopMotors() {
    ledcWrite(CH_L, 0); ledcWrite(CH_R, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    motorState = "STOP";
}

void brakeMotors() {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    ledcWrite(CH_L, 1023); ledcWrite(CH_R, 1023);
    motorState = "BRAKE";
}

void forward(int spd) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(CH_L, spd); ledcWrite(CH_R, spd);
    motorState = "FWD";
}

void backward(int spd) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(CH_L, spd); ledcWrite(CH_R, spd);
    motorState = "BWD";
}

void spinLeft(int spd) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(CH_L, spd); ledcWrite(CH_R, spd);
    motorState = "SPIN_L";
}

void spinRight(int spd) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(CH_L, spd); ledcWrite(CH_R, spd);
    motorState = "SPIN_R";
}

void leftOnly(int spd) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(CH_L, spd);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    ledcWrite(CH_R, 0);
    motorState = "LEFT_ONLY";
}

void rightOnly(int spd) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    ledcWrite(CH_L, 0);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(CH_R, spd);
    motorState = "RIGHT_ONLY";
}

void calibrateGyro() {
    Serial.println("[CAL] Calibrating gyro... keep still 2 sec");
    sensors_event_t a, g, t;
    float sum = 0;
    int count = 0;
    unsigned long start = millis();
    while (millis() - start < 2000) {
        mpu.getEvent(&a, &g, &t);
        sum += g.gyro.z;
        count++;
        delay(5);
    }
    gyroZOffset = (count > 0) ? sum / count : 0;
    Serial.printf("[CAL] Done. offset=%.6f rad/s (%.3f deg/s) samples=%d\n",
        gyroZOffset, gyroZOffset * 180.0 / PI, count);
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("\n########################################");
    Serial.println("# Test 07: MPU6050 + Motor EMI Test    #");
    Serial.println("# Commands: w/s/a/d/q/e = drive        #");
    Serial.println("#           x=stop b=brake 1-4=speed   #");
    Serial.println("#           r=reset c=recal p=pause     #");
    Serial.println("########################################\n");

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Motor pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    ledcSetup(CH_L, 5000, 10);
    ledcSetup(CH_R, 5000, 10);
    ledcAttachPin(PWMA, CH_L);
    ledcAttachPin(PWMB, CH_R);
    digitalWrite(STBY, HIGH);
    stopMotors();
    Serial.println("[MOTOR] Ready");

    // MPU6050
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("[MPU] FAIL — check wiring");
        while (1) delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.println("[MPU] Ready (250dps, 2G, 5Hz BW)");

    calibrateGyro();

    Serial.println("\n[DATA] time_s,heading,filtered_gz,raw_gz,accel_z,bad_i2c,bad_pct,motor,speed");
    Serial.println("[INFO] Drive motors and watch heading. Should stay ~0 if going straight.\n");
}

unsigned long lastUpdate = 0;
unsigned long startTime = 0;

void loop() {
    unsigned long now = millis();
    if (startTime == 0) startTime = now;
    if (lastUpdate == 0) { lastUpdate = now; return; }
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    // --- Read serial commands ---
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'w': forward(motorSpeed); break;
            case 's': backward(motorSpeed); break;
            case 'a': spinLeft(motorSpeed); break;
            case 'd': spinRight(motorSpeed); break;
            case 'q': leftOnly(motorSpeed); break;
            case 'e': rightOnly(motorSpeed); break;
            case 'x': stopMotors(); break;
            case 'b': brakeMotors(); break;
            case '1': motorSpeed = 300;  Serial.println("[SPD] 300 (LOW)"); break;
            case '2': motorSpeed = 500;  Serial.println("[SPD] 500 (MED)"); break;
            case '3': motorSpeed = 700;  Serial.println("[SPD] 700 (HIGH)"); break;
            case '4': motorSpeed = 1023; Serial.println("[SPD] 1023 (MAX)"); break;
            case 'r': heading = 0; badI2C = 0; totalReads = 0;
                      Serial.println("[RESET] Heading=0, counters cleared"); break;
            case 'c': stopMotors(); calibrateGyro(); break;
            case 'p': outputPaused = !outputPaused;
                      Serial.printf("[%s]\n", outputPaused ? "PAUSED" : "RESUMED"); break;
        }
    }

    // --- Read MPU6050 with same filtering as main firmware ---
    float gz = 0;
    float az = 0;
    sensors_event_t a, g, t;
    totalReads++;

    bool ok = mpu.getEvent(&a, &g, &t);
    az = a.acceleration.z;
    rawGz = (g.gyro.z - gyroZOffset) * 180.0 / PI;

    if (!ok) {
        badI2C++;
        gz = 0;
    } else if (abs(az) < ACCEL_Z_MIN || abs(az) > ACCEL_Z_MAX) {
        badI2C++;
        gz = 0;
    } else {
        gz = rawGz;
        if (abs(gz - prevGyroZ) > GYRO_JUMP_LIMIT) { gz = prevGyroZ; badI2C++; }
        if (abs(gz) > GYRO_MAX_RATE) { gz = prevGyroZ; badI2C++; }
        if (abs(gz) < GYRO_DEADZONE) gz = 0;
        prevGyroZ = gz;
    }

    heading += gz * dt;
    while (heading >= 360) heading -= 360;
    while (heading < 0) heading += 360;

    // --- Output every 200ms ---
    static unsigned long lastPrint = 0;
    if (!outputPaused && now - lastPrint >= 200) {
        lastPrint = now;
        float elapsed = (now - startTime) / 1000.0;
        float badPct = totalReads > 0 ? 100.0 * badI2C / totalReads : 0;
        Serial.printf("[DATA] %.1f,%.1f,%.2f,%.2f,%.2f,%d,%.1f%%,%s,%d\n",
            elapsed, heading, gz, rawGz, az, badI2C, badPct, motorState, motorSpeed);
    }

    delay(10);
}
