// ============================================
// ESP32-S3 4WD Robot Chassis
// TB6612FNG | Ultrasonic + Servo Sweep
// Dual Encoders | MPU6050 + QMC5883L Compass
// A* Pathfinding | Obstacle Avoidance
// Dual-Core: Core1=Robot, Core0=WiFi
// ============================================

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "config.h"
#include "encoder.h"
#include "motors.h"
#include "sensors.h"
#include "avoidance.h"
#include "pathfinder.h"
#include "debug.h"

// --- Global Objects ---
Motors            motors;
Sensors           sensors;
Encoder           encoder;
ObstacleAvoidance avoidance;
Pathfinder        pathfinder;

// --- Timing ---
unsigned long lastScan   = 0;
unsigned long lastPath   = 0;

// --- Robot State ---
bool autonomousMode = true;
bool emergencyBrake = false;

// --- WiFi Task (Core 0) ---
TaskHandle_t wifiTaskHandle;
SemaphoreHandle_t stateMutex;

// Shared state for WiFi task (protected by mutex)
struct TelemetryData {
    float front, left, right, heading, compassHeading, gyroRate;
    float posX, posY, distance;
    long encL, encR;
    float battery;
    int avoidState, pathLen;
    bool autoMode, hasTarget, targetReached, backtracking;
    int gridX, gridY, targetGX, targetGY;
    float targetWX, targetWY;
    int crumbs;
    bool compassOk, mpuOk;
} telemetry;

// Command received from server (Core 0 writes, Core 1 reads — needs lock)
struct Command {
    char cmd[20];
    float x, y;
    bool pending;
} pendingCmd = {"none", 0, 0, false};
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Server URL builder
String serverURL(const char* path) {
    String url = SERVER_HTTPS ? "https://" : "http://";
    url += SERVER_HOST;
    if (!SERVER_HTTPS) { url += ":"; url += SERVER_PORT; }
    url += path;
    return url;
}

// HTTP client with token
WiFiClientSecure secureClient;

void httpBegin(HTTPClient &http, const char* path) {
    if (SERVER_HTTPS) {
        secureClient.setInsecure(); // Skip cert validation (simple, works for free hosting)
        http.begin(secureClient, serverURL(path));
    } else {
        http.begin(serverURL(path));
    }
    http.setTimeout(500);
    http.addHeader("X-Robot-Token", API_TOKEN);
}

// Forward declarations
void wifiTask(void* param);
void postTelemetry();
void fetchCommand();
void processCommand();
void navigateWithPath();
void updateMap();
void setupWiFi();
void debugFlush();

// Hardware tests
void runTest(const char* test);
void testI2C();
void testUltrasonic();
void testServo();
void testMPU6050();
void testCompass();
void testMotors();
void testEncoders();
void testBattery();
void testAll();

// ============================================
// SETUP (runs on Core 1)
// ============================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32-S3 4WD Robot Starting ===");

    Wire.begin(I2C_SDA, I2C_SCL);

    motors.begin();
    sensors.begin();
    encoder.begin();
    avoidance.begin();
    pathfinder.begin();
    Debug::begin();

    Serial.printf("Init: servo=%d mpu=%d compass=%d\n",
                  sensors.servoReady, sensors.mpuReady, sensors.compassReady);
    if (!sensors.servoReady) {
        Serial.println("!!! SERVO ATTACH FAILED — check wiring/power. Avoidance sweep disabled.");
    }

    stateMutex = xSemaphoreCreateMutex();

    setupWiFi();

    // Launch WiFi on Core 0
    xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, NULL, 1, &wifiTaskHandle, 0);

    Debug::log("Boot complete. Dual-core active.");
    Serial.println("=== Robot Ready (Core1=Robot, Core0=WiFi) ===");
}

// ============================================
// MAIN LOOP (Core 1 - never blocked by WiFi)
// ============================================
void loop() {
    unsigned long now = millis();

    // Sweep tick every cycle
    sensors.sweepTick();

    // Safety: front check always
    if (sensors.readFrontNow() && sensors.distFront < OBSTACLE_CLOSE) {
        if (!autonomousMode) {
            motors.brake();
            emergencyBrake = true;
        }
    } else if (emergencyBrake && sensors.servoAtCenter() && sensors.distFront > OBSTACLE_SLOW) {
        emergencyBrake = false;
    }

    // Process any pending command from WiFi task.
    // Always honor stop/auto/reset, even during emergency brake.
    if (pendingCmd.pending) {
        bool isOverride = strcmp(pendingCmd.cmd, "stop") == 0 ||
                          strcmp(pendingCmd.cmd, "auto") == 0 ||
                          strcmp(pendingCmd.cmd, "reset") == 0;
        if (!emergencyBrake || isOverride) processCommand();
    }

    if (!autonomousMode) return;

    // Sensor + Encoder + Avoidance (50ms = 20Hz)
    if (now - lastScan >= SCAN_INTERVAL_MS) {
        lastScan = now;

        sensors.updateHeading();
        encoder.update(sensors.headingRad);
        pathfinder.updateRobotWorld(encoder.posX, encoder.posY);

        if (pathfinder.isBacktracking()) {
            pathfinder.updateBacktrack(encoder.posX, encoder.posY);
        }

        bool avoiding = avoidance.update(sensors, motors);

        if (!avoiding) {
            updateMap();
            navigateWithPath();
        }

        // Update telemetry snapshot for WiFi task
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            telemetry.front = sensors.distFront;
            telemetry.left  = sensors.distLeft;
            telemetry.right = sensors.distRight;
            telemetry.heading = sensors.heading;
            telemetry.compassHeading = sensors.compassHeading;
            telemetry.gyroRate = sensors.gyroRate;
            telemetry.posX = encoder.posX;
            telemetry.posY = encoder.posY;
            telemetry.distance = encoder.totalDistCm;
            telemetry.encL = encoder.getLeftCount();
            telemetry.encR = encoder.getRightCount();
            telemetry.battery = sensors.getBatteryVoltage();
            telemetry.avoidState = (int)avoidance.state;
            telemetry.pathLen = pathfinder.pathLength;
            telemetry.autoMode = autonomousMode;
            telemetry.hasTarget = pathfinder.hasTarget;
            telemetry.targetReached = pathfinder.targetReached;
            telemetry.backtracking = pathfinder.isBacktracking();
            telemetry.gridX = pathfinder.robotPos.x;
            telemetry.gridY = pathfinder.robotPos.y;
            telemetry.targetGX = pathfinder.targetGrid.x;
            telemetry.targetGY = pathfinder.targetGrid.y;
            telemetry.targetWX = pathfinder.targetWorldX;
            telemetry.targetWY = pathfinder.targetWorldY;
            telemetry.crumbs = pathfinder.crumbCount;
            telemetry.compassOk = sensors.compassReady;
            telemetry.mpuOk = sensors.mpuReady;
            xSemaphoreGive(stateMutex);
        }
    }

    // Pathfinding recalc (500ms)
    if (now - lastPath >= PATH_UPDATE_MS) {
        lastPath = now;
        pathfinder.findPath();
    }
}

// ============================================
// WIFI TASK (Core 0 - independent)
// ============================================
void wifiTask(void* param) {
    unsigned long lastReport = 0;
    unsigned long lastDebug = 0;

    while (true) {
        unsigned long now = millis();

        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        if (now - lastReport >= REPORT_INTERVAL_MS) {
            lastReport = now;
            postTelemetry();
            fetchCommand();
        }

        if (now - lastDebug >= 500) {
            lastDebug = now;
            debugFlush();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ============================================
// TELEMETRY
// ============================================
void postTelemetry() {
    HTTPClient http;
    httpBegin(http, "/api/robot/report");
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;

    if (xSemaphoreTake(stateMutex, 50 / portTICK_PERIOD_MS) == pdTRUE) {
        doc["front"] = telemetry.front;
        doc["left"]  = telemetry.left;
        doc["right"] = telemetry.right;
        doc["heading"] = telemetry.heading;
        doc["compass_heading"] = telemetry.compassHeading;
        doc["gyro_rate"] = telemetry.gyroRate;
        doc["pos_x"] = telemetry.posX;
        doc["pos_y"] = telemetry.posY;
        doc["distance"] = telemetry.distance;
        doc["enc_l"] = telemetry.encL;
        doc["enc_r"] = telemetry.encR;
        doc["battery"] = telemetry.battery;
        doc["state"]  = telemetry.avoidState;
        doc["path_length"] = telemetry.pathLen;
        doc["auto"]   = telemetry.autoMode;
        doc["has_target"] = telemetry.hasTarget;
        doc["target_reached"] = telemetry.targetReached;
        doc["backtracking"] = telemetry.backtracking;
        doc["grid_x"] = telemetry.gridX;
        doc["grid_y"] = telemetry.gridY;
        doc["target_x"] = telemetry.targetGX;
        doc["target_y"] = telemetry.targetGY;
        doc["target_wx"] = telemetry.targetWX;
        doc["target_wy"] = telemetry.targetWY;
        doc["crumbs"] = telemetry.crumbs;
        doc["compass_ok"] = telemetry.compassOk;
        doc["mpu_ok"] = telemetry.mpuOk;
        xSemaphoreGive(stateMutex);
    }

    doc["wifi_rssi"]  = WiFi.RSSI();
    doc["free_heap"]  = ESP.getFreeHeap();
    doc["uptime"]     = millis() / 1000;
    doc["debug_mode"] = (bool)DEBUG_MODE;

    String payload;
    serializeJson(doc, payload);
    http.POST(payload);
    http.end();
}

// ============================================
// FETCH + PROCESS COMMAND
// ============================================
void fetchCommand() {
    HTTPClient http;
    httpBegin(http, "/api/robot/command");

    int code = http.GET();
    if (code == 200) {
        JsonDocument doc;
        if (!deserializeJson(doc, http.getString())) {
            const char* cmd = doc["cmd"] | "none";
            if (strcmp(cmd, "none") != 0) {
                portENTER_CRITICAL(&cmdMux);
                strncpy(pendingCmd.cmd, cmd, 19);
                pendingCmd.cmd[19] = '\0';
                pendingCmd.x = doc["x"] | 0.0f;
                pendingCmd.y = doc["y"] | 0.0f;
                pendingCmd.pending = true;
                portEXIT_CRITICAL(&cmdMux);
            }
        }
    }
    http.end();
}

void processCommand() {
    // Snapshot under lock so Core 0 can't half-write while we read
    char cmdBuf[20];
    float cmdX, cmdY;
    portENTER_CRITICAL(&cmdMux);
    strncpy(cmdBuf, pendingCmd.cmd, 20);
    cmdX = pendingCmd.x;
    cmdY = pendingCmd.y;
    pendingCmd.pending = false;
    portEXIT_CRITICAL(&cmdMux);
    const char* cmd = cmdBuf;

    if (strcmp(cmd, "forward") == 0)      { autonomousMode = false; motors.forward(SPEED_MEDIUM); }
    else if (strcmp(cmd, "back") == 0)    { autonomousMode = false; motors.backward(SPEED_MEDIUM); }
    else if (strcmp(cmd, "left") == 0)    { autonomousMode = false; motors.turnLeft(SPEED_TURN); }
    else if (strcmp(cmd, "right") == 0)   { autonomousMode = false; motors.turnRight(SPEED_TURN); }
    else if (strcmp(cmd, "stop") == 0)    { autonomousMode = false; motors.stop(); }
    else if (strcmp(cmd, "auto") == 0)    { autonomousMode = true; }
    else if (strcmp(cmd, "set_target") == 0) {
        pathfinder.setTargetWorld(cmdX, cmdY);
        autonomousMode = true;
        Debug::logf("[CMD] Target: X=%.0f Y=%.0f", cmdX, cmdY);
    }
    else if (strcmp(cmd, "backtrack") == 0) { pathfinder.startBacktrack(); autonomousMode = true; }
    else if (strcmp(cmd, "reset") == 0)   { encoder.resetPosition(); pathfinder.begin(); autonomousMode = true; }
    else if (strncmp(cmd, "test_", 5) == 0) { runTest(cmd); }
}

// ============================================
// NAVIGATION
// ============================================
void navigateWithPath() {
    if (pathfinder.targetReached) { motors.stop(); return; }

    if (pathfinder.pathLength < 2) {
        if (pathfinder.hasTarget) motors.forward(SPEED_SLOW);
        else motors.stop();
        return;
    }

    int dir = pathfinder.getNextDirection(sensors.headingRad);
    switch (dir) {
        case 0:  motors.forward(sensors.distFront > OBSTACLE_SLOW ? SPEED_MEDIUM : SPEED_SLOW); break;
        case -1: motors.curveLeft(SPEED_MEDIUM); break;
        case 1:  motors.curveRight(SPEED_MEDIUM); break;
        case -2: motors.stop(); break;
    }
}

void updateMap() {
    if (sensors.distFront < 300) pathfinder.markObstacle(sensors.distFront, sensors.headingRad);
    if (sensors.distLeft  < 300) pathfinder.markObstacle(sensors.distLeft,  sensors.headingRad - PI / 2);
    if (sensors.distRight < 300) pathfinder.markObstacle(sensors.distRight, sensors.headingRad + PI / 2);
}

// ============================================
// WIFI SETUP
// ============================================
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nConnected: %s (RSSI: %d)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
        Debug::logf("WiFi: %s RSSI=%d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("\nWiFi failed - offline mode");
    }
}

// ============================================
// DEBUG FLUSH
// ============================================
void debugFlush() {
    // Snapshot under lock so Core 1 can keep logging while we POST.
    char snapshot[Debug::MAX_LOGS][Debug::MAX_MSG_LEN];
    int n = 0;
    portENTER_CRITICAL(&Debug::mux);
    if (Debug::logCount > 0) {
        int start = (Debug::logHead - Debug::logCount + Debug::MAX_LOGS) % Debug::MAX_LOGS;
        for (int i = 0; i < Debug::logCount; i++) {
            strncpy(snapshot[n], Debug::logs[(start + i) % Debug::MAX_LOGS], Debug::MAX_MSG_LEN);
            snapshot[n][Debug::MAX_MSG_LEN - 1] = '\0';
            n++;
        }
        Debug::logCount = 0;
    }
    portEXIT_CRITICAL(&Debug::mux);

    if (n == 0) return;

    HTTPClient http;
    httpBegin(http, "/api/robot/debug");
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;
    JsonArray arr = doc["logs"].to<JsonArray>();
    for (int i = 0; i < n; i++) arr.add(snapshot[i]);

    String payload;
    serializeJson(doc, payload);
    http.POST(payload);
    http.end();
}

// ============================================
// HARDWARE TESTS
// ============================================
void runTest(const char* test) {
    autonomousMode = false;
    motors.stop();

    Debug::logf("===== %s =====", test);

    if (strcmp(test, "test_all") == 0)         testAll();
    else if (strcmp(test, "test_i2c") == 0)    testI2C();
    else if (strcmp(test, "test_ultrasonic") == 0) testUltrasonic();
    else if (strcmp(test, "test_servo") == 0)  testServo();
    else if (strcmp(test, "test_mpu") == 0)    testMPU6050();
    else if (strcmp(test, "test_compass") == 0) testCompass();
    else if (strcmp(test, "test_motors") == 0) testMotors();
    else if (strcmp(test, "test_encoder") == 0) testEncoders();
    else if (strcmp(test, "test_battery") == 0) testBattery();
    else                                        Debug::logf("[TEST] Unknown: %s", test);

    Debug::log("===== test done =====");
    // Don't call debugFlush() here — it does HTTP from Core 1 and races
    // Core 0's wifiTask. Core 0 will pick up the buffer within 500ms.
}

void testI2C() {
    Debug::log("[TEST] I2C scan...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            found++;
            if (addr == MPU6050_ADDR)      Debug::logf("[I2C] 0x%02X = MPU6050", addr);
            else if (addr == COMPASS_ADDR) Debug::logf("[I2C] 0x%02X = QMC5883L", addr);
            else                           Debug::logf("[I2C] 0x%02X = unknown", addr);
        }
    }
    Debug::logf("[I2C] %d device(s)", found);
}

void testUltrasonic() {
    // Five readings so a single bad pulse doesn't make the test look broken
    int errors = 0;
    for (int i = 0; i < 5; i++) {
        digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
        digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US_TRIG, LOW);
        unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
        if (dur == 0) { Debug::logf("[US %d/5] no echo", i + 1); errors++; }
        else          { Debug::logf("[US %d/5] %.1f cm", i + 1, dur / 58.0); }
        delay(80);
    }
    if (errors == 5)      Debug::log("[US] FAIL: nothing in front, or sensor unwired");
    else if (errors > 0)  Debug::logf("[US] %d/5 misses (acceptable)", errors);
    else                  Debug::log("[US] OK");
}

void testServo() {
    if (!sensors.servoReady) {
        Debug::log("[SERVO] FAIL: attach() returned false at boot — servo never initialized");
        Debug::log("[SERVO] Check: 5V power, signal wire on GPIO 9, common GND");
        return;
    }
    Debug::log("[SERVO] -> CENTER (90)"); sensors.sweepServo.write(SERVO_CENTER); delay(600);
    Debug::log("[SERVO] -> LEFT (160)");  sensors.sweepServo.write(SERVO_LEFT);   delay(600);
    Debug::log("[SERVO] -> RIGHT (20)");  sensors.sweepServo.write(SERVO_RIGHT);  delay(600);
    Debug::log("[SERVO] -> CENTER (90)"); sensors.sweepServo.write(SERVO_CENTER); delay(400);
    Debug::log("[SERVO] OK (if you saw it move)");
}

void testMPU6050() {
    Debug::log("[TEST] MPU6050...");
    sensors_event_t a, g, t;
    sensors.mpu.getEvent(&a, &g, &t);
    Debug::logf("[MPU] Accel Z=%.2f g", a.acceleration.z / 9.81);
    Debug::logf("[MPU] Gyro  Z=%.1f d/s", g.gyro.z * 180.0 / PI);
    Debug::logf("[MPU] Temp=%.1f C", t.temperature);
    Debug::log(abs(a.acceleration.z) > 7 ? "[MPU] OK" : "[MPU] WARNING: weak Z");
}

void testCompass() {
    Debug::log("[TEST] Compass (QMC5883L)...");
    Wire.beginTransmission(COMPASS_ADDR);
    if (Wire.endTransmission() != 0) {
        Debug::logf("[COMPASS] ERROR: No response at 0x%02X", COMPASS_ADDR);
        return;
    }
    float h = sensors.compassHeading;
    Debug::logf("[COMPASS] Heading: %.1f deg", h);
    Debug::log(h >= 0 ? "[COMPASS] OK" : "[COMPASS] WARNING: no data");
}

void testMotors() {
    Debug::log("[TEST] Motors...");
    Debug::log("[MOTOR] Left FWD");  ledcWrite(0, SPEED_SLOW); digitalWrite(MOTOR_L_AIN1, HIGH); digitalWrite(MOTOR_L_AIN2, LOW); delay(1000);
    Debug::log("[MOTOR] Left BWD");  digitalWrite(MOTOR_L_AIN1, LOW); digitalWrite(MOTOR_L_AIN2, HIGH); delay(1000);
    ledcWrite(0, 0); motors.stop(); delay(200);
    Debug::log("[MOTOR] Right FWD"); ledcWrite(1, SPEED_SLOW); digitalWrite(MOTOR_R_BIN1, HIGH); digitalWrite(MOTOR_R_BIN2, LOW); delay(1000);
    Debug::log("[MOTOR] Right BWD"); digitalWrite(MOTOR_R_BIN1, LOW); digitalWrite(MOTOR_R_BIN2, HIGH); delay(1000);
    ledcWrite(1, 0); motors.stop(); delay(200);
    Debug::log("[MOTOR] Both FWD");  motors.forward(SPEED_SLOW); delay(1000);
    motors.stop();
    Debug::log("[MOTOR] Done");
}

void testEncoders() {
    Debug::log("[TEST] Encoders...");
    long startL = encoder.getLeftCount();
    long startR = encoder.getRightCount();
    Debug::logf("[ENC] L=%ld R=%ld. Push robot 3 sec...", startL, startR);
    delay(3000);
    long dL = encoder.getLeftCount() - startL;
    long dR = encoder.getRightCount() - startR;
    Debug::logf("[ENC] Delta L=%ld R=%ld", dL, dR);
    Debug::log((dL > 0 || dR > 0) ? "[ENC] OK" : "[ENC] ERROR: No ticks!");
}

void testBattery() {
    Debug::log("[TEST] Battery...");
    float v = sensors.getBatteryVoltage();
    Debug::logf("[BATT] %.2f V (range %.1f-%.1f)", v, BATTERY_MIN_V, BATTERY_MAX_V);
    if (v < BATTERY_MIN_V)      Debug::log("[BATT] WARNING: LOW!");
    else if (v > BATTERY_MAX_V) Debug::log("[BATT] WARNING: over-voltage or no divider");
    else                         Debug::log("[BATT] OK");
}

void testAll() {
    Debug::log("========== FULL HARDWARE TEST ==========");
    testBattery(); testI2C(); testMPU6050(); testCompass();
    testUltrasonic(); testServo(); testMotors(); testEncoders();
    Debug::log("========== TEST COMPLETE ==========");
}
