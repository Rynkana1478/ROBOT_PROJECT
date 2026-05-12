// ============================================
// ESP32-S3 4WD Robot Chassis
// Core 1: Brain — sensor fusion, navigator, motor/servo state machine
// Core 0: Two FreeRTOS tasks — ultrasonicTask + wifiTask
//         Backpressure via scheduler (HTTP slow → WiFi task sleeps,
//         ultrasonic keeps firing).
// ============================================

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include "config.h"
#include "encoder.h"
#include "motors.h"
#include "sensors.h"
#include "pathfinder.h"
#include "navigator.h"
#include "command_queue.h"
#include "debug.h"

// --- Global Objects ---
Motors       motors;
Sensors      sensors;
Encoder      encoder;
Pathfinder   pathfinder;
Navigator    navigator;
CommandQueue cmdQueue;

// --- Mode + manual hard override ---
enum Mode { MODE_AUTO, MODE_MANUAL };
volatile Mode currentMode = MODE_MANUAL;
char manualState[16]  = "stop";   // applied immediately while in manual mode
volatile bool freshBoot   = true;
volatile unsigned long lastSyncMs    = 0;   // last successful server round-trip
volatile unsigned long lastWifiOkMs  = 0;   // last time WiFi link was up (any cause)

// --- Timing ---
unsigned long lastScan  = 0;
unsigned long lastDecay = 0;

bool lowBattery = false;

// --- Sweep state machine (servo lives on Core 1 now) ---
Servo sweepServo;
bool  servoReady = false;
volatile int  currentAngleIdx   = -1;   // -1 = servo in transit
volatile int  measuredAngleIdx  = -1;   // angle of last completed measurement
unsigned long servoSettleUntil  = 0;
int sweepStep = 0;
int sweepDir  = 1;

static const float SWEEP_OFFSETS[SWEEP_STEPS] = {
    1.2217, 0.6109, 0.0, -0.6109, -1.2217
};

// --- Inter-core ---
TaskHandle_t ultrasonicHandle;
TaskHandle_t wifiHandle;
SemaphoreHandle_t stateMutex;

struct TelemetryData {
    float front, left, right, heading, gyroRate;
    float nearLeft, nearRight;
    float posX, posY, distance;
    float accelX, accelY;
    long encL, encR;
    float battery;
    Mode  mode;
    bool hasTarget, targetReached, backtracking;
    int gridX, gridY, targetGX, targetGY;
    float targetWX, targetWY;
    int crumbs;
    bool mpuOk, encoderHealthy;
    const char* navState;
    bool avoidActive, battLow, stuckWarning, slipping, stalled;
    float traction;
    float pathLengthCm;
    float avoidDX, avoidDY, avoidDHeading;
    uint32_t ingestedThrough, currentTargetId, lastCompletedId;
    const char* lastCompletionStatus;
    int  queueSize;
    bool freshBoot;
} telemetry;

portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
struct PendingResp {
    bool   hasMode;
    Mode   mode;
    bool   hasManual;
    char   manualState[16];
    QueuedCmd cmds[CMD_QUEUE_DEPTH];
    int    cmdCount;
    bool   hasHeadingOverride;
    float  headingOverride;
    bool   queueClear;
} pendingResp = {};

static char gridRLEBuf[3072];
static int gridRLELen = 0;

WiFiClientSecure secureClient;

String serverURL(const char* path) {
    String url = SERVER_HTTPS ? "https://" : "http://";
    url += SERVER_HOST;
    if (!SERVER_HTTPS) { url += ":"; url += SERVER_PORT; }
    url += path;
    return url;
}

void httpBegin(HTTPClient &http, const char* path) {
    if (SERVER_HTTPS) {
        secureClient.setInsecure();
        http.begin(secureClient, serverURL(path));
    } else {
        http.begin(serverURL(path));
    }
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.setReuse(true);
    http.addHeader("X-Chassis-ID", CHASSIS_ID);
}

// Forward declarations
void ultrasonicTask(void* param);
void wifiTask(void* param);
void syncWithServer();
void ingestPendingResp();
void executeQueuedCmd(const QueuedCmd &cmd);
void executeManualState();
void serviceSweepStateMachine();
void updateMap();
void setupWiFi();

void runTest(const char* test);
void testI2C();      void testUltrasonic(); void testServo();
void testMPU6050();  void testMotors();     void testEncoders();
void testBattery();  void testAll();

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32-S3 4WD Robot Starting ===");

    Wire.begin(I2C_SDA, I2C_SCL);

    motors.begin();
    sensors.begin();
    encoder.begin();
    pathfinder.begin();
    navigator.begin();
    cmdQueue.begin();
    Debug::begin();

    // Servo on Core 1
    sweepServo.setPeriodHertz(50);
    sweepServo.attach(SERVO_PIN, 500, 2400);
    servoReady = sweepServo.attached();
    if (servoReady) {
        sweepServo.write(SERVO_CENTER);
        servoSettleUntil = millis() + 300;
    }

    Serial.printf("Init: mpu=%d servo=%d\n", sensors.mpuReady, servoReady);

    stateMutex = xSemaphoreCreateMutex();
    setupWiFi();

    xTaskCreatePinnedToCore(ultrasonicTask, "us",   4096, NULL, 1, &ultrasonicHandle, 0);
    xTaskCreatePinnedToCore(wifiTask,       "wifi", 8192, NULL, 1, &wifiHandle,       0);

    Debug::log("Boot complete. Core1=Brain  Core0=US+WiFi (split tasks)");
    Serial.println("=== Robot Ready ===");
}

// ============================================
// MAIN LOOP — Core 1 (Brain)
// 50Hz tick: sensors → fusion → navigator/manual → motors+servo
// ============================================
void loop() {
    motors.tick();                            // non-blocking apply (every iter)
    serviceSweepStateMachine();               // non-blocking servo step

    unsigned long now = millis();
    if (now - lastScan < SCAN_INTERVAL_MS) return;
    lastScan = now;

    // ---- Sensors / fusion ----
    sensors.motorsActiveExternal =
        (motors.getState() != Motors::M_STOP && motors.getState() != Motors::M_BRAKE);

    // Invalidate sweepData on turn-state transitions so we don't carry forward
    // readings that were captured at the wrong heading.
    static Motors::State prevMotorStateLoop = Motors::M_STOP;
    Motors::State curState = motors.getState();
    bool wasTurningLoop = (prevMotorStateLoop == Motors::M_TURN_L ||
                           prevMotorStateLoop == Motors::M_TURN_R);
    bool isTurningLoop  = (curState == Motors::M_TURN_L ||
                           curState == Motors::M_TURN_R);
    if (wasTurningLoop != isTurningLoop) {
        portENTER_CRITICAL(&sweepMux);
        sweepData.fresh = false;
        portEXIT_CRITICAL(&sweepMux);
        for (int i = 0; i < SWEEP_STEPS; i++) sensors.sweepDist[i] = 999;
    }
    prevMotorStateLoop = curState;

    sensors.updateFromSweep();
    sensors.updateHeading();

    float prevPosX = encoder.posX, prevPosY = encoder.posY;
    encoder.update(sensors.headingRad, motors.getState(),
                   sensors.accelY, sensors.gyroRate);
    pathfinder.updateRobotWorld(encoder.posX, encoder.posY);
    navigator.accumulatePath(
        sqrt((encoder.posX - prevPosX) * (encoder.posX - prevPosX) +
             (encoder.posY - prevPosY) * (encoder.posY - prevPosY)));
    updateMap();

    // ---- Battery cutoff ----
    float voltage = sensors.getBatteryVoltage();
    if (voltage > 1.0 && voltage < BATTERY_MIN_V) {
        if (!lowBattery) { lowBattery = true; motors.stop();
                           Debug::log("[BATT] LOW! motors disabled"); }
    } else if (lowBattery && voltage > BATTERY_MIN_V + 0.5) {
        lowBattery = false; Debug::log("[BATT] Recovered");
    }

    // ---- Obstacle map decay ----
    if (now - lastDecay > DECAY_CHECK_MS) {
        lastDecay = now;
        pathfinder.decayObstacles();
    }

    // ---- Ingest server response (mode / manual / queue) ----
    ingestPendingResp();

    // ---- Deadman: WiFi-link based, NOT HTTP latency ----
    // Avoidance, e-brake, and manual driving all run locally on Core 1 from
    // ultrasonic sensor data. They do not need the server. A slow internet
    // connection used to trip the old HTTP-latency deadman and stutter-stop
    // the robot mid-avoid; that was wrong. We only halt when the WiFi link
    // itself has been down long enough that the operator truly can't reach us.
    bool wifiUp = (WiFi.status() == WL_CONNECTED);
    if (wifiUp) lastWifiOkMs = now;
    if (lastWifiOkMs > 0 && !wifiUp && (now - lastWifiOkMs > WIFI_DEADMAN_MS)) {
        if (motors.getState() != Motors::M_STOP) {
            motors.stop();
            strncpy(manualState, "stop", sizeof(manualState));
            Debug::logf("[DEADMAN] WiFi down %lums → stop", now - lastWifiOkMs);
        }
    }

    if (lowBattery) goto snapshot;

    // ---- Drive: manual is hard override; auto drains queue ----
    if (currentMode == MODE_MANUAL) {
        // Manual forward with assisted avoidance:
        //   1. distFront >= OBSTACLE_SLOW    -> normal forward (executeManualState)
        //   2. distFront <  OBSTACLE_SLOW    -> updateManualAvoid takes over:
        //        AV_BRAKE (200ms)  → pick side → turn 30° → check → drive 30cm
        //        Repeats per increment up to AVOID_MAX_INCREMENTS (120°).
        //        On give-up: stays braked until distFront >= OBSTACLE_CLEAR.
        //   3. User releases forward         -> cancelManualAvoid (immediate stop)
        // While avoid is active the FSM owns sweepMode (BRAKE/PICK use NORMAL,
        // DRIVE_PAST uses BYPASS_* to watch the near-side wall).
        bool wantsForward = (strcmp(manualState, "forward") == 0);

        // User let go of forward (or pressed back/turn/stop) mid-avoid → cancel.
        if (!wantsForward) {
            navigator.cancelManualAvoid(motors);
        }

        if (wantsForward) {
            currentSweepMode = SWEEP_FRONT_LOCK;  // default; avoid overrides per phase
            if (!navigator.updateManualAvoid(sensors, motors, pathfinder,
                                             encoder.posX, encoder.posY,
                                             sensors.heading)) {
                // Approach slowdown: between OBSTACLE_SLOWDOWN and OBSTACLE_SLOW,
                // run at SPEED_SLOW instead of SPEED_CRUISE. Cruise momentum is
                // too high for the brake to dump fully in 30cm; pre-slowing
                // gives the brake enough margin to actually stop the chassis.
                if (sensors.distFront < OBSTACLE_SLOWDOWN) {
                    motors.setState(Motors::M_FORWARD, SPEED_SLOW);
                } else {
                    executeManualState();
                }
            }
        } else {
            currentSweepMode = SWEEP_NORMAL;
            executeManualState();
        }
    } else {
        // Auto mode: drain queue; navigator drives toward current target.
        if (!pathfinder.hasTarget && !cmdQueue.isEmpty()) {
            QueuedCmd next;
            if (cmdQueue.pop(next)) executeQueuedCmd(next);
        }

        if (pathfinder.hasTarget && !pathfinder.targetReached &&
            navigator.state != NAV_FAILED) {
            if (pathfinder.isBacktracking())
                pathfinder.updateBacktrack(encoder.posX, encoder.posY);
            navigator.update(sensors, motors, pathfinder,
                             encoder.posX, encoder.posY, sensors.heading);
        } else {
            // Target finished or failed → ack and clear, ready for next pop
            if (pathfinder.targetReached) {
                cmdQueue.completeCurrent("REACHED");
                pathfinder.hasTarget = false;
                Debug::log("[NAV] target reached");
            } else if (navigator.state == NAV_FAILED) {
                cmdQueue.completeCurrent("FAILED_STUCK");
                pathfinder.hasTarget = false;
                Debug::log("[NAV] target failed (stuck)");
                navigator.state = NAV_IDLE;
            }
            if (motors.getState() != Motors::M_STOP &&
                motors.getState() != Motors::M_BRAKE)
                motors.stop();
        }
    }

snapshot:
    // ---- Telemetry snapshot for wifiTask ----
    if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
        telemetry.front     = sensors.distFront;
        telemetry.left      = sensors.distLeft;
        telemetry.right     = sensors.distRight;
        telemetry.nearLeft  = sensors.distNearLeft;
        telemetry.nearRight = sensors.distNearRight;
        telemetry.heading   = sensors.heading;
        telemetry.gyroRate  = sensors.gyroRate;
        telemetry.accelX    = sensors.accelX;
        telemetry.accelY    = sensors.accelY;
        telemetry.posX      = encoder.posX;
        telemetry.posY      = encoder.posY;
        telemetry.distance  = encoder.totalDistCm;
        telemetry.encL      = encoder.getLeftCount();
        telemetry.encR      = encoder.getRightCount();
        telemetry.battery   = voltage;
        telemetry.mode      = currentMode;
        telemetry.hasTarget = pathfinder.hasTarget;
        telemetry.targetReached = pathfinder.targetReached;
        telemetry.backtracking  = pathfinder.isBacktracking();
        telemetry.gridX     = pathfinder.robotPos.x;
        telemetry.gridY     = pathfinder.robotPos.y;
        telemetry.targetGX  = pathfinder.targetGrid.x;
        telemetry.targetGY  = pathfinder.targetGrid.y;
        telemetry.targetWX  = pathfinder.targetWorldX;
        telemetry.targetWY  = pathfinder.targetWorldY;
        telemetry.crumbs    = pathfinder.crumbCount;
        telemetry.mpuOk     = sensors.mpuReady;
        telemetry.encoderHealthy = encoder.encoderHealthy;
        telemetry.navState  = navigator.stateStr();
        telemetry.avoidActive  = navigator.avoidActive;
        telemetry.battLow      = lowBattery;
        telemetry.stuckWarning = navigator.stuckWarning;
        telemetry.slipping     = encoder.slipping;
        telemetry.stalled      = encoder.stalled;
        telemetry.traction     = encoder.traction;
        telemetry.pathLengthCm = navigator.pathLengthCm;
        telemetry.avoidDX        = navigator.avoidLastDX;
        telemetry.avoidDY        = navigator.avoidLastDY;
        telemetry.avoidDHeading  = navigator.avoidLastDHeading;
        telemetry.ingestedThrough = cmdQueue.ingestedThrough();
        telemetry.currentTargetId = cmdQueue.currentTargetId();
        telemetry.lastCompletedId = cmdQueue.lastCompletedId();
        telemetry.lastCompletionStatus = cmdQueue.lastCompletionStatus();
        telemetry.queueSize       = cmdQueue.size();
        telemetry.freshBoot       = freshBoot;
        if (pathfinder.gridDirty) {
            gridRLELen = pathfinder.encodeGridRLE(gridRLEBuf, sizeof(gridRLEBuf));
            pathfinder.gridDirty = false;
        }
        xSemaphoreGive(stateMutex);
    }
}

// ============================================
// SERVO SWEEP STATE MACHINE (non-blocking, on Core 1)
// Three phases tracked by (currentAngleIdx, servoSettleUntil):
//   A) idx=-1, settleUntil=0       -> write servo, arm settle deadline
//   B) idx=-1, now>=settleUntil    -> publish idx so ultrasonicTask can fire
//   C) idx>=0, measured==current   -> advance sweepStep, reset to phase A
//
// The previous version re-entered phase A every loop iteration after settle
// expired, which kept resetting servoSettleUntil and starved phase B. Core 1
// now owns publishing; Core 0's ultrasonicTask only reads idx and measures.
// ============================================
// Returns the next sweepStep for the active sweep mode.
//   NORMAL       -> ping-pong 0..SWEEP_STEPS-1
//   BYPASS_LEFT  -> alternate idx 2 (front 90°)  ↔ idx 3 (near-left  125°)
//   BYPASS_RIGHT -> alternate idx 2 (front 90°)  ↔ idx 1 (near-right  55°)
//   FRONT_LOCK   -> always idx 2 (servo never leaves 90°)
// Why near-side (idx 1/3) not far-side (idx 0/4): during AV_DRIVE_PAST the
// chassis just turned 30° away from an obstacle; the wall is now ~30° off
// the new heading, which falls on the near-side cone (35° off-center).
// Far-side (160°/20°) is nearly perpendicular and misses the clipping zone.
// In non-NORMAL modes, sweepDir is unused but left untouched so a return to
// NORMAL just resumes ping-pong direction from wherever we are.
static int advanceSweepStep(SweepMode mode, int curStep, int* dir) {
    if (mode == SWEEP_FRONT_LOCK)   return 2;
    if (mode == SWEEP_BYPASS_LEFT)  return (curStep == 2) ? 3 : 2;
    if (mode == SWEEP_BYPASS_RIGHT) return (curStep == 2) ? 1 : 2;
    int next = curStep + *dir;
    if (next >= SWEEP_STEPS) { *dir = -1; return SWEEP_STEPS - 2; }
    if (next < 0)            { *dir =  1; return 1; }
    return next;
}

void serviceSweepStateMachine() {
    if (!servoReady) return;
    unsigned long now = millis();

    // Phase A: arm new sweep position
    if (currentAngleIdx == -1 && servoSettleUntil == 0) {
        sweepServo.write(SWEEP_ANGLES[sweepStep]);
        servoSettleUntil = now + SERVO_SETTLE_MS;
        return;
    }

    // Phase B: settle elapsed -> publish so Core 0 can fire ultrasonic
    if (currentAngleIdx == -1 && now >= servoSettleUntil) {
        currentAngleIdx = sweepStep;
        return;
    }

    // Phase C: measurement landed -> advance sweep, force re-arm next call
    if (currentAngleIdx >= 0 && measuredAngleIdx == currentAngleIdx) {
        currentAngleIdx = -1;
        servoSettleUntil = 0;
        sweepStep = advanceSweepStep(currentSweepMode, sweepStep, &sweepDir);
        // Reset measuredAngleIdx so Core 0 will fire again on the NEXT
        // publication. Critical for FRONT_LOCK and BYPASS_* modes where
        // advanceSweepStep returns the SAME idx repeatedly — without this
        // reset, Core 0's `idx == measuredAngleIdx` guard skipped every
        // republication and the ultrasonic stopped firing entirely (distFront
        // froze at whatever value was captured the first time the mode
        // engaged, so the e-brake never saw approaching obstacles).
        measuredAngleIdx = -1;
    }
}

// ============================================
// ULTRASONIC TASK (Core 0)
// Reads currentAngleIdx (published by Core 1 after servo settles),
// fires ultrasonic, stores by angle, signals back via measuredAngleIdx.
// ============================================
void ultrasonicTask(void* param) {
    unsigned long lastFire = 0;
    while (true) {
        int idx = currentAngleIdx;     // Core 1 publishes when servo settled
        if (idx < 0 || idx >= SWEEP_STEPS) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }
        if (idx == measuredAngleIdx) {
            // Already measured this position; wait for Core 1 to advance.
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        unsigned long now = millis();
        if (now - lastFire < ULTRASONIC_INTERVAL_MS) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }
        lastFire = now;

        float d = sensors.readUltrasonic();
        portENTER_CRITICAL(&sweepMux);
        sweepData.dist[idx] = d;
        sweepData.fresh = true;
        portEXIT_CRITICAL(&sweepMux);
        measuredAngleIdx = idx;
    }
}

// ============================================
// WIFI TASK (Core 0)
// Independent loop, blocks on socket I/O while ultrasonicTask continues.
// ============================================
void wifiTask(void* param) {
    unsigned long lastSync = 0;
    unsigned long lastWifiLog = 0;
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            unsigned long now = millis();
            if (now - lastWifiLog > 5000) {
                Serial.printf("[WIFI] disconnected, status=%d, reconnecting...\n",
                              WiFi.status());
                lastWifiLog = now;
            }
            WiFi.reconnect();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        unsigned long now = millis();
        unsigned long syncInt = (currentMode == MODE_MANUAL)
            ? SYNC_INTERVAL_MANUAL_MS : SYNC_INTERVAL_AUTO_MS;
        if (now - lastSync < syncInt) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }
        lastSync = now;
        syncWithServer();
        lastSyncMs = millis();
    }
}

// ============================================
// SYNC (always sends telemetry; smaller payload when nothing changed)
// ============================================
void syncWithServer() {
    HTTPClient http;
    httpBegin(http, "/api/robot/sync");
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;

    // Debug logs
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
    if (n > 0) {
        JsonArray arr = doc["debug"].to<JsonArray>();
        for (int i = 0; i < n; i++) arr.add(snapshot[i]);
    }

    if (xSemaphoreTake(stateMutex, 50 / portTICK_PERIOD_MS) == pdTRUE) {
        doc["front"]     = telemetry.front;
        doc["left"]      = telemetry.left;
        doc["right"]     = telemetry.right;
        doc["near_left"] = telemetry.nearLeft;
        doc["near_right"]= telemetry.nearRight;
        doc["heading"]   = telemetry.heading;
        doc["gyro_rate"] = telemetry.gyroRate;
        doc["accel_x"]   = telemetry.accelX;
        doc["accel_y"]   = telemetry.accelY;
        doc["pos_x"]     = telemetry.posX;
        doc["pos_y"]     = telemetry.posY;
        doc["distance"]  = telemetry.distance;
        doc["enc_l"]     = telemetry.encL;
        doc["enc_r"]     = telemetry.encR;
        doc["battery"]   = telemetry.battery;
        doc["mode"]      = (telemetry.mode == MODE_AUTO ? "auto" : "manual");
        doc["auto"]      = (telemetry.mode == MODE_AUTO);
        doc["has_target"] = telemetry.hasTarget;
        doc["target_reached"] = telemetry.targetReached;
        doc["backtracking"]   = telemetry.backtracking;
        doc["grid_x"]    = telemetry.gridX;
        doc["grid_y"]    = telemetry.gridY;
        doc["target_x"]  = telemetry.targetGX;
        doc["target_y"]  = telemetry.targetGY;
        doc["target_wx"] = telemetry.targetWX;
        doc["target_wy"] = telemetry.targetWY;
        doc["crumbs"]    = telemetry.crumbs;
        doc["mpu_ok"]    = telemetry.mpuOk;
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["free_heap"] = ESP.getFreeHeap();
        doc["uptime"]    = millis() / 1000;
        doc["enc_healthy"]   = telemetry.encoderHealthy;
        doc["nav_state"]     = telemetry.navState;
        doc["control"]       = (telemetry.mode == MODE_AUTO ? "auto" :
                               (telemetry.avoidActive ? "avoid" : "manual"));
        doc["avoid_active"]  = telemetry.avoidActive;
        doc["batt_low"]      = telemetry.battLow;
        doc["stuck"]         = telemetry.stuckWarning;
        doc["slipping"]      = telemetry.slipping;
        doc["stalled"]       = telemetry.stalled;
        doc["traction"]      = telemetry.traction;
        doc["path_length"]   = telemetry.pathLengthCm;
        doc["avoid_dx"]      = telemetry.avoidDX;
        doc["avoid_dy"]      = telemetry.avoidDY;
        doc["avoid_dheading"]= telemetry.avoidDHeading;
        doc["ingested_through"] = telemetry.ingestedThrough;
        doc["current_target_id"]= telemetry.currentTargetId;
        doc["last_completed_id"]= telemetry.lastCompletedId;
        doc["last_completion_status"] = telemetry.lastCompletionStatus;
        doc["queue_size"]       = telemetry.queueSize;
        doc["fresh_boot"]       = telemetry.freshBoot;
        if (gridRLELen > 0) {
            static char localGrid[3072];
            memcpy(localGrid, gridRLEBuf, gridRLELen);
            localGrid[gridRLELen] = '\0';
            gridRLELen = 0;
            doc["grid"] = (const char*)localGrid;
        }
        xSemaphoreGive(stateMutex);
    }

    String payload;
    serializeJson(doc, payload);
    int code = http.POST(payload);

    if (code != 200) {
        // Rate-limited Serial log so we can see real connectivity errors
        // (HTTP -1=connect refused, -11=read timeout, -2=DNS, etc).
        // Can't use Debug::log here — it pipes through this same sync that
        // just failed.
        static unsigned long lastErrLog = 0;
        static int lastErrCode = 0;
        unsigned long now = millis();
        if (code != lastErrCode || now - lastErrLog > 5000) {
            Serial.printf("[SYNC] HTTP %d url=%s rssi=%d heap=%u\n",
                          code, serverURL("/api/robot/sync").c_str(),
                          WiFi.RSSI(), ESP.getFreeHeap());
            lastErrCode = code;
            lastErrLog = now;
        }
    }

    if (code == 200) {
        JsonDocument resp;
        if (!deserializeJson(resp, http.getString())) {
            // Stage incoming directives into pendingResp; brain ingests next loop.
            portENTER_CRITICAL(&cmdMux);
            pendingResp = {};   // clear

            if (resp["mode"].is<const char*>()) {
                const char* m = resp["mode"].as<const char*>();
                pendingResp.hasMode = true;
                pendingResp.mode = (strcmp(m, "auto") == 0) ? MODE_AUTO : MODE_MANUAL;
            }

            if (resp["manual_state"].is<const char*>()) {
                const char* ms = resp["manual_state"].as<const char*>();
                pendingResp.hasManual = true;
                strncpy(pendingResp.manualState, ms, sizeof(pendingResp.manualState));
                pendingResp.manualState[sizeof(pendingResp.manualState) - 1] = '\0';
            }

            if (resp["queue_clear"].is<bool>()) {
                pendingResp.queueClear = resp["queue_clear"].as<bool>();
            }

            if (resp["set_heading"].is<float>()) {
                pendingResp.hasHeadingOverride = true;
                pendingResp.headingOverride = resp["set_heading"].as<float>();
            }

            if (resp["cmds"].is<JsonArray>()) {
                JsonArray arr = resp["cmds"].as<JsonArray>();
                int n = 0;
                for (JsonVariant v : arr) {
                    if (n >= CMD_QUEUE_DEPTH) break;
                    QueuedCmd &c = pendingResp.cmds[n];
                    c.id = v["id"] | 0;
                    const char* t = v["type"] | "";
                    c.x = v["x"] | 0.0f;
                    c.y = v["y"] | 0.0f;
                    c.heading = v["heading"] | 0.0f;
                    if      (strcmp(t, "set_target")    == 0) c.type = CMD_SET_TARGET;
                    else if (strcmp(t, "move_relative") == 0) c.type = CMD_MOVE_RELATIVE;
                    else if (strcmp(t, "turn_relative") == 0) c.type = CMD_TURN_RELATIVE;
                    else if (strcmp(t, "backtrack")     == 0) c.type = CMD_BACKTRACK;
                    else if (strcmp(t, "reset")         == 0) c.type = CMD_RESET;
                    else if (strcmp(t, "clear_map")     == 0) c.type = CMD_CLEAR_MAP;
                    else if (strcmp(t, "calibrate")     == 0) c.type = CMD_CALIBRATE;
                    else if (strcmp(t, "set_heading")   == 0) c.type = CMD_SET_HEADING;
                    else                                       c.type = CMD_NONE;
                    if (c.type != CMD_NONE) n++;
                }
                pendingResp.cmdCount = n;
            }
            portEXIT_CRITICAL(&cmdMux);
        }
        if (freshBoot) freshBoot = false;
    }
    http.end();
}

// ============================================
// INGEST PENDING SERVER RESPONSE (Core 1, brain loop)
// Manual hard override is applied via this path — manual_state going to a
// non-stop value flips mode to MANUAL automatically.
// ============================================
void ingestPendingResp() {
    PendingResp local;
    portENTER_CRITICAL(&cmdMux);
    local = pendingResp;
    pendingResp = {};
    portEXIT_CRITICAL(&cmdMux);

    if (local.hasManual) {
        // Authoritative: whatever the server says, that's the manual state —
        // including "stop". The previous version dropped stop while in auto,
        // which left manualState stale (e.g. "forward") and the brain loop
        // would re-issue the old motion the next tick. Always copy.
        strncpy(manualState, local.manualState, sizeof(manualState));
        manualState[sizeof(manualState) - 1] = '\0';

        // Hard override: any manual command (incl. stop) flips us to MANUAL.
        // Server already does this on its side; we mirror so a stop press
        // yanks an in-flight auto plan even if hasMode arrived later.
        if (currentMode != MODE_MANUAL) {
            currentMode = MODE_MANUAL;
            navigator.state = NAV_IDLE;
            navigator.avoidActive = false;   // drop any in-flight auto avoid
            navigator.avoidGaveUp = false;
            motors.stop();   // immediate halt during the mode transition
            Debug::log("[MODE] manual hard override");
        }
    }

    if (local.hasMode) {
        if (local.mode != currentMode) {
            currentMode = local.mode;
            navigator.state = NAV_IDLE;
            navigator.avoidActive = false;
            navigator.avoidGaveUp = false;
            Debug::logf("[MODE] -> %s",
                        currentMode == MODE_AUTO ? "auto" : "manual");
            if (currentMode == MODE_AUTO) {
                strncpy(manualState, "stop", sizeof(manualState));
            }
        }
    }

    if (local.queueClear) {
        cmdQueue.clear();
        pathfinder.hasTarget = false;
        navigator.state = NAV_IDLE;
        motors.stop();
        Debug::log("[QUEUE] cleared by server");
    }

    if (local.hasHeadingOverride) {
        sensors.setHeading(local.headingOverride);
        Debug::logf("[HEADING] set to %.1f", local.headingOverride);
    }

    for (int i = 0; i < local.cmdCount; i++) {
        // Immediate-priority types execute now, never queued.
        const QueuedCmd &c = local.cmds[i];
        if (c.type == CMD_RESET) {
            motors.stop();
            sensors.resetHeading();
            encoder.resetPosition();
            pathfinder.begin();
            navigator.begin();
            cmdQueue.clear();
            Debug::log("[CMD] RESET");
        } else if (c.type == CMD_CLEAR_MAP) {
            pathfinder.clearMap();
            pathfinder.gridDirty = true;
            Debug::log("[CMD] CLEAR_MAP");
        } else if (c.type == CMD_CALIBRATE) {
            sensors.resetHeading();
            Debug::log("[CMD] CALIBRATE heading=0");
        } else if (c.type == CMD_SET_HEADING) {
            sensors.setHeading(c.heading);
            Debug::logf("[CMD] SET_HEADING %.1f", c.heading);
        } else {
            // Queued (auto-mode targets / backtrack)
            if (!cmdQueue.push(c)) {
                Debug::logf("[QUEUE] full, dropped id=%lu", c.id);
            }
        }
    }
}

// ============================================
// EXECUTE A POPPED QUEUED COMMAND
// ============================================
void executeQueuedCmd(const QueuedCmd &cmd) {
    switch (cmd.type) {
        case CMD_SET_TARGET:
            pathfinder.setTargetWorld(cmd.x, cmd.y);
            navigator.setTarget(motors, encoder.posX, encoder.posY);
            Debug::logf("[QUEUE] -> set_target id=%lu (%.0f,%.0f)",
                        cmd.id, cmd.x, cmd.y);
            break;
        case CMD_MOVE_RELATIVE: {
            float h = sensors.headingRad;
            float dx = cmd.y * sin(h) + cmd.x * cos(h);
            float dy = cmd.y * cos(h) - cmd.x * sin(h);
            pathfinder.setTargetWorld(encoder.posX + dx, encoder.posY + dy);
            navigator.setTarget(motors, encoder.posX, encoder.posY);
            Debug::logf("[QUEUE] -> move_relative id=%lu fwd=%.0f right=%.0f",
                        cmd.id, cmd.y, cmd.x);
            break;
        }
        case CMD_TURN_RELATIVE: {
            // Pure rotation: no X,Y target — use turnOnly so pathfinder skips
            // its distance-based reached check; navigator sets pf.targetReached
            // when the heading goal is hit.
            pathfinder.hasTarget = true;
            pathfinder.targetReached = false;
            pathfinder.turnOnly = true;
            navigator.setTurnTarget(motors, sensors.heading, cmd.heading);
            Debug::logf("[QUEUE] -> turn_relative id=%lu deg=%.1f",
                        cmd.id, cmd.heading);
            break;
        }
        case CMD_BACKTRACK:
            pathfinder.startBacktrack();
            navigator.setTarget(motors, encoder.posX, encoder.posY);
            Debug::logf("[QUEUE] -> backtrack id=%lu", cmd.id);
            break;
        default:
            break;
    }
}

// ============================================
// EXECUTE MANUAL STATE (immediate)
// ============================================
void executeManualState() {
    if      (strcmp(manualState, "forward") == 0) motors.setState(Motors::M_FORWARD);
    else if (strcmp(manualState, "back")    == 0) motors.setState(Motors::M_BACKWARD);
    else if (strcmp(manualState, "left")    == 0) motors.setState(Motors::M_TURN_L);
    else if (strcmp(manualState, "right")   == 0) motors.setState(Motors::M_TURN_R);
    else if (strncmp(manualState, "test_", 5) == 0) {
        // One-shot tests dispatched through the manual channel
        char t[20]; strncpy(t, manualState, sizeof(t));
        strncpy(manualState, "stop", sizeof(manualState));
        runTest(t);
    } else                                          motors.setState(Motors::M_STOP);
}

// ============================================
// MAP UPDATE (mark obstacles from sweep)
// Skipped while the chassis is rotating — sweep readings span ~400ms, during
// which a fast turn rotates the body more than the spread of the SWEEP_OFFSETS
// itself. Painting those rays at the *current* heading puts the same wall at
// many false positions, which is what was producing phantom obstacles.
// ============================================
void updateMap() {
    Motors::State s = motors.getState();
    if (s == Motors::M_TURN_L || s == Motors::M_TURN_R) return;
    if (fabs(sensors.gyroRate) > MAP_GATE_GYRO_DEG_S) return;

    for (int i = 0; i < SWEEP_STEPS; i++) {
        if (sensors.sweepDist[i] < 300) {
            pathfinder.markObstacle(sensors.sweepDist[i],
                                    sensors.headingRad + SWEEP_OFFSETS[i]);
        }
    }
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
        Serial.printf("\nConnected: %s (RSSI: %d)\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        Debug::logf("WiFi: %s RSSI=%d",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("\nWiFi failed - offline mode");
    }
}

// ============================================
// HARDWARE TESTS (unchanged behavior, called via manualState test_*)
// ============================================
void runTest(const char* test) {
    motors.stop();
    Debug::logf("===== %s =====", test);
    if      (strcmp(test, "test_all") == 0)        testAll();
    else if (strcmp(test, "test_i2c") == 0)        testI2C();
    else if (strcmp(test, "test_ultrasonic") == 0) testUltrasonic();
    else if (strcmp(test, "test_servo") == 0)      testServo();
    else if (strcmp(test, "test_mpu") == 0)        testMPU6050();
    else if (strcmp(test, "test_motors") == 0)     testMotors();
    else if (strcmp(test, "test_encoder") == 0)    testEncoders();
    else if (strcmp(test, "test_battery") == 0)    testBattery();
    else                                            Debug::logf("[TEST] Unknown: %s", test);
    Debug::log("===== test done =====");
}

void testI2C() {
    Debug::log("[TEST] I2C scan...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            found++;
            if (addr == MPU6050_ADDR) Debug::logf("[I2C] 0x%02X = MPU6050", addr);
            else                       Debug::logf("[I2C] 0x%02X = unknown", addr);
        }
    }
    Debug::logf("[I2C] %d device(s)", found);
}

void testUltrasonic() {
    int errors = 0;
    for (int i = 0; i < 5; i++) {
        float d = sensors.readUltrasonic();
        if (d >= 999) { Debug::logf("[US %d/5] no echo", i + 1); errors++; }
        else          { Debug::logf("[US %d/5] %.1f cm", i + 1, d); }
        delay(80);
    }
    if (errors == 5)      Debug::log("[US] FAIL: nothing in front, or sensor unwired");
    else if (errors > 0)  Debug::logf("[US] %d/5 misses (acceptable)", errors);
    else                  Debug::log("[US] OK");
}

void testServo() {
    if (!servoReady) { Debug::log("[SERVO] FAIL: attach() returned false"); return; }
    Debug::log("[SERVO] -> CENTER (90)"); sweepServo.write(SERVO_CENTER); delay(600);
    Debug::log("[SERVO] -> LEFT (160)");  sweepServo.write(SERVO_LEFT);   delay(600);
    Debug::log("[SERVO] -> RIGHT (20)");  sweepServo.write(SERVO_RIGHT);  delay(600);
    Debug::log("[SERVO] -> CENTER (90)"); sweepServo.write(SERVO_CENTER); delay(400);
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

void testMotors() {
    Debug::log("[TEST] Motors...");
    motors.setState(Motors::M_FORWARD, SPEED_SLOW); motors.tick(); delay(800);
    motors.setState(Motors::M_BACKWARD, SPEED_SLOW); motors.tick(); delay(800);
    motors.setState(Motors::M_TURN_L, SPEED_TURN);   motors.tick(); delay(600);
    motors.setState(Motors::M_TURN_R, SPEED_TURN);   motors.tick(); delay(600);
    motors.stop(); motors.tick();
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
    testBattery(); testI2C(); testMPU6050();
    testUltrasonic(); testServo(); testMotors(); testEncoders();
    Debug::log("========== TEST COMPLETE ==========");
}
