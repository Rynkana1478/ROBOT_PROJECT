// ============================================
// Test 02: MPU6050 Diagnostic for Tuning
// Wire: SDA=GPIO11, SCL=GPIO12, VCC=3.3V, GND
//
// Tests run in sequence:
//   1. I2C scan + init
//   2. Accelerometer gravity baseline
//   3. Gyro calibration + noise floor (still, 5 sec)
//   4. Filter bandwidth comparison (5Hz, 10Hz, 21Hz, 44Hz)
//   5. Gyro range comparison (250, 500 deg/s)
//   6. Drift test (30 sec, keep still)
//   7. I2C stress test (rapid reads, count failures)
//   8. Continuous output (for live tuning)
//
// IMPORTANT: Keep robot STILL and motors OFF for tests 1-7.
//            Test 8 runs forever — rotate to verify response.
// ============================================

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define I2C_SDA 11
#define I2C_SCL 12

Adafruit_MPU6050 mpu;
float gyroZOffset = 0;

void calibrateGyro(int durationMs) {
    sensors_event_t a, g, t;
    float sum = 0;
    int count = 0;
    unsigned long start = millis();
    while (millis() - start < (unsigned long)durationMs) {
        mpu.getEvent(&a, &g, &t);
        sum += g.gyro.z;
        count++;
        delay(5);
    }
    gyroZOffset = (count > 0) ? sum / count : 0;
}

// ==========================================
// TEST 1: I2C scan
// ==========================================
bool test1_i2c() {
    Serial.println("\n[TEST1] === I2C Scan ===");
    int found = 0;
    bool mpuFound = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            const char* name = "unknown";
            if (addr == 0x68) { name = "MPU6050"; mpuFound = true; }
            else if (addr == 0x0D) name = "QMC5883L";
            Serial.printf("[TEST1] FOUND 0x%02X = %s\n", addr, name);
            found++;
        }
    }
    Serial.printf("[TEST1] RESULT devices=%d mpu=%s\n", found, mpuFound ? "OK" : "FAIL");
    return mpuFound;
}

// ==========================================
// TEST 2: Accelerometer gravity baseline
// ==========================================
void test2_accel() {
    Serial.println("\n[TEST2] === Accelerometer Gravity Baseline (2 sec) ===");
    Serial.println("[TEST2] Keep robot FLAT and STILL");

    float sumX = 0, sumY = 0, sumZ = 0;
    float minZ = 999, maxZ = -999;
    int count = 0;
    sensors_event_t a, g, t;

    unsigned long start = millis();
    while (millis() - start < 2000) {
        mpu.getEvent(&a, &g, &t);
        sumX += a.acceleration.x;
        sumY += a.acceleration.y;
        sumZ += a.acceleration.z;
        if (a.acceleration.z < minZ) minZ = a.acceleration.z;
        if (a.acceleration.z > maxZ) maxZ = a.acceleration.z;
        count++;
        delay(5);
    }

    float avgX = sumX / count, avgY = sumY / count, avgZ = sumZ / count;
    float magnitude = sqrt(avgX*avgX + avgY*avgY + avgZ*avgZ);

    Serial.printf("[TEST2] AVG  X=%.3f Y=%.3f Z=%.3f m/s2\n", avgX, avgY, avgZ);
    Serial.printf("[TEST2] Z_RANGE min=%.3f max=%.3f spread=%.3f\n", minZ, maxZ, maxZ - minZ);
    Serial.printf("[TEST2] MAGNITUDE %.3f m/s2 (expect ~9.81)\n", magnitude);
    Serial.printf("[TEST2] TEMP %.1f C\n", t.temperature);
    Serial.printf("[TEST2] SAMPLES %d in 2000ms (%.0f Hz)\n", count, count / 2.0);

    bool flat = abs(avgZ) > 8.0 && abs(avgZ) < 12.0;
    bool stable = (maxZ - minZ) < 2.0;
    Serial.printf("[TEST2] RESULT flat=%s stable=%s\n",
        flat ? "OK" : "FAIL", stable ? "OK" : "NOISY");
}

// ==========================================
// TEST 3: Gyro noise floor (still, 5 sec)
// ==========================================
void test3_noise() {
    Serial.println("\n[TEST3] === Gyro Noise Floor (5 sec, keep STILL) ===");

    calibrateGyro(2000);
    Serial.printf("[TEST3] OFFSET %.6f rad/s (%.3f deg/s)\n",
        gyroZOffset, gyroZOffset * 180.0 / PI);

    float sumGz = 0, sumGzSq = 0;
    float minGz = 999, maxGz = -999;
    int count = 0;
    int aboveThreshold[5] = {0, 0, 0, 0, 0};  // >0.3, >0.5, >1.0, >2.0, >5.0
    float thresholds[] = {0.3, 0.5, 1.0, 2.0, 5.0};
    sensors_event_t a, g, t;

    unsigned long start = millis();
    while (millis() - start < 5000) {
        mpu.getEvent(&a, &g, &t);
        float gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;  // deg/s

        sumGz += gz;
        sumGzSq += gz * gz;
        if (gz < minGz) minGz = gz;
        if (gz > maxGz) maxGz = gz;

        for (int i = 0; i < 5; i++) {
            if (abs(gz) > thresholds[i]) aboveThreshold[i]++;
        }
        count++;
        delay(5);
    }

    float mean = sumGz / count;
    float variance = (sumGzSq / count) - (mean * mean);
    float stddev = sqrt(abs(variance));

    Serial.printf("[TEST3] SAMPLES %d\n", count);
    Serial.printf("[TEST3] MEAN %.4f deg/s\n", mean);
    Serial.printf("[TEST3] STDDEV %.4f deg/s\n", stddev);
    Serial.printf("[TEST3] RANGE min=%.3f max=%.3f spread=%.3f deg/s\n", minGz, maxGz, maxGz - minGz);
    for (int i = 0; i < 5; i++) {
        Serial.printf("[TEST3] ABOVE_%.1f %d/%d (%.1f%%)\n",
            thresholds[i], aboveThreshold[i], count, 100.0 * aboveThreshold[i] / count);
    }
    Serial.printf("[TEST3] RESULT deadzone_recommend=%.1f deg/s\n",
        max(0.3f, stddev * 3));
}

// ==========================================
// TEST 4: Filter bandwidth comparison
// ==========================================
void test4_bandwidth() {
    Serial.println("\n[TEST4] === Filter Bandwidth Comparison (keep STILL) ===");

    struct BWTest {
        mpu6050_bandwidth_t bw;
        const char* name;
    } tests[] = {
        {MPU6050_BAND_5_HZ,  "5Hz"},
        {MPU6050_BAND_10_HZ, "10Hz"},
        {MPU6050_BAND_21_HZ, "21Hz"},
        {MPU6050_BAND_44_HZ, "44Hz"},
    };

    for (int t = 0; t < 4; t++) {
        mpu.setFilterBandwidth(tests[t].bw);
        delay(100);  // let filter settle

        calibrateGyro(1000);

        float sumGzSq = 0, minGz = 999, maxGz = -999;
        int count = 0;
        int spikes = 0;
        sensors_event_t a, g, temp;

        unsigned long start = millis();
        while (millis() - start < 3000) {
            mpu.getEvent(&a, &g, &temp);
            float gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
            sumGzSq += gz * gz;
            if (gz < minGz) minGz = gz;
            if (gz > maxGz) maxGz = gz;
            if (abs(gz) > 1.0) spikes++;
            count++;
            delay(5);
        }

        float rmsNoise = sqrt(sumGzSq / count);
        Serial.printf("[TEST4] BW=%s rms=%.4f spread=%.3f spikes_above_1=%d/%d\n",
            tests[t].name, rmsNoise, maxGz - minGz, spikes, count);
    }

    // Restore to 10Hz (our current setting)
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    calibrateGyro(1000);
    Serial.println("[TEST4] RESULT pick lowest rms with acceptable response time");
}

// ==========================================
// TEST 5: Gyro range comparison
// ==========================================
void test5_range() {
    Serial.println("\n[TEST5] === Gyro Range Comparison (keep STILL) ===");

    struct RangeTest {
        mpu6050_gyro_range_t range;
        const char* name;
    } tests[] = {
        {MPU6050_RANGE_250_DEG,  "250dps"},
        {MPU6050_RANGE_500_DEG,  "500dps"},
    };

    for (int t = 0; t < 2; t++) {
        mpu.setGyroRange(tests[t].range);
        delay(100);

        calibrateGyro(1000);

        float sumGzSq = 0;
        int count = 0;
        sensors_event_t a, g, temp;

        unsigned long start = millis();
        while (millis() - start < 3000) {
            mpu.getEvent(&a, &g, &temp);
            float gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
            sumGzSq += gz * gz;
            count++;
            delay(5);
        }

        float rmsNoise = sqrt(sumGzSq / count);
        Serial.printf("[TEST5] RANGE=%s rms=%.4f deg/s (lower range = better resolution)\n",
            tests[t].name, rmsNoise);
    }

    // Restore 250dps
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    calibrateGyro(1000);
    Serial.println("[TEST5] RESULT 250dps has half the noise of 500dps (higher resolution per bit)");
}

// ==========================================
// TEST 6: Drift test (30 sec, keep still)
// ==========================================
void test6_drift() {
    Serial.println("\n[TEST6] === Heading Drift Test (30 sec, keep STILL) ===");

    calibrateGyro(2000);

    float heading = 0;
    unsigned long lastUpdate = millis();
    sensors_event_t a, g, t;
    int badReads = 0;
    float prevGz = 0;

    Serial.println("[TEST6] TIME_S,HEADING_DEG,GYRO_DPS,ACCEL_Z,BAD_READS");

    unsigned long start = millis();
    unsigned long lastPrint = start;

    while (millis() - start < 30000) {
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0;
        lastUpdate = now;

        bool ok = mpu.getEvent(&a, &g, &t);
        float az = a.acceleration.z;
        float gz = 0;

        if (!ok) {
            badReads++;
        } else if (abs(az) < 5.0 || abs(az) > 15.0) {
            badReads++;
        } else {
            gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
            if (abs(gz - prevGz) > 50.0) {
                badReads++;
                gz = prevGz;
            }
            if (abs(gz) < 1.0) gz = 0;
            prevGz = gz;
        }

        heading += gz * dt;

        if (now - lastPrint >= 1000) {
            lastPrint = now;
            float elapsed = (now - start) / 1000.0;
            Serial.printf("[TEST6] %.1f,%.2f,%.2f,%.2f,%d\n",
                elapsed, heading, gz, az, badReads);
        }

        delay(10);
    }

    Serial.printf("[TEST6] FINAL_DRIFT %.2f deg in 30s (%.3f deg/s avg drift)\n",
        heading, heading / 30.0);
    Serial.printf("[TEST6] BAD_READS %d total\n", badReads);
    Serial.printf("[TEST6] RESULT %s\n",
        abs(heading) < 5.0 ? "GOOD (< 5 deg/30s)" :
        abs(heading) < 15.0 ? "OK (< 15 deg/30s)" : "BAD (> 15 deg/30s, needs work)");
}

// ==========================================
// TEST 7: I2C stress test (rapid reads)
// ==========================================
void test7_i2c_stress() {
    Serial.println("\n[TEST7] === I2C Stress Test (1000 rapid reads) ===");

    int total = 1000;
    int getEventFail = 0;
    int accelBad = 0;
    int gyroSpike = 0;
    float prevGz = 0;
    sensors_event_t a, g, t;

    unsigned long start = millis();
    for (int i = 0; i < total; i++) {
        bool ok = mpu.getEvent(&a, &g, &t);
        if (!ok) {
            getEventFail++;
            continue;
        }

        float az = a.acceleration.z;
        if (abs(az) < 5.0 || abs(az) > 15.0) accelBad++;

        float gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
        if (abs(gz - prevGz) > 50.0) gyroSpike++;
        prevGz = gz;

        // No delay — max I2C speed
    }
    unsigned long elapsed = millis() - start;

    Serial.printf("[TEST7] TIME %lu ms for %d reads (%.0f reads/sec)\n",
        elapsed, total, 1000.0 * total / elapsed);
    Serial.printf("[TEST7] GETEVENT_FAIL %d/%d (%.1f%%)\n",
        getEventFail, total, 100.0 * getEventFail / total);
    Serial.printf("[TEST7] ACCEL_CORRUPT %d/%d (%.1f%%)\n",
        accelBad, total, 100.0 * accelBad / total);
    Serial.printf("[TEST7] GYRO_SPIKE %d/%d (%.1f%%)\n",
        gyroSpike, total, 100.0 * gyroSpike / total);
    Serial.printf("[TEST7] RESULT %s\n",
        (getEventFail + accelBad + gyroSpike) == 0 ? "CLEAN" :
        (getEventFail + accelBad + gyroSpike) < 10 ? "MOSTLY OK" : "NOISY I2C BUS");
}

// ==========================================
// TEST 8: Continuous (live tuning)
// ==========================================
void test8_continuous() {
    Serial.println("\n[TEST8] === Continuous Output (rotate to test, runs forever) ===");
    Serial.println("[TEST8] FORMAT: heading,gyroZ_dps,accelZ,temp,bad_i2c_total");
    Serial.println("[TEST8] Rotate 90 deg right -> heading should read ~90");
    Serial.println("[TEST8] Return to start -> heading should read ~0");
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("\n########################################");
    Serial.println("# MPU6050 FULL DIAGNOSTIC              #");
    Serial.println("# Keep robot STILL + motors OFF        #");
    Serial.println("# Tests 1-7 auto-run (~60 sec total)   #");
    Serial.println("# Test 8 runs forever for live tuning  #");
    Serial.println("########################################\n");

    Wire.begin(I2C_SDA, I2C_SCL);

    if (!test1_i2c()) {
        Serial.println("ABORT: No MPU6050 found");
        while (1) delay(1000);
    }

    Serial.println("\n[INIT] Starting MPU6050...");
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("ABORT: MPU6050 init failed");
        while (1) delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    Serial.println("[INIT] MPU6050 ready (250dps, 2G, 10Hz BW)\n");

    test2_accel();
    test3_noise();
    test4_bandwidth();
    test5_range();
    test6_drift();
    test7_i2c_stress();
    test8_continuous();
}

// ==========================================
// LOOP: continuous heading output
// ==========================================
float heading = 0;
unsigned long lastUpdate = 0;
float prevGz = 0;
int badI2C = 0;

void loop() {
    unsigned long now = millis();
    if (lastUpdate == 0) { lastUpdate = now; return; }
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    sensors_event_t a, g, t;
    float gz = 0;

    bool ok = mpu.getEvent(&a, &g, &t);
    float az = a.acceleration.z;

    if (!ok || abs(az) < 5.0 || abs(az) > 15.0) {
        badI2C++;
    } else {
        gz = (g.gyro.z - gyroZOffset) * 180.0 / PI;
        if (abs(gz - prevGz) > 50.0) { gz = prevGz; badI2C++; }
        if (abs(gz) < 1.0) gz = 0;
        prevGz = gz;
    }

    heading += gz * dt;
    while (heading >= 360) heading -= 360;
    while (heading < 0) heading += 360;

    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("[TEST8] %.1f,%.2f,%.2f,%.1f,%d\n",
            heading, gz, az, t.temperature, badI2C);
    }

    delay(10);
}
