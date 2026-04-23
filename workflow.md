# Robot Chassis — System Workflow

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32-S3                             │
│                                                             │
│  ┌─────────────────────┐    ┌───────────────────────────┐  │
│  │     Core 0           │    │        Core 1              │  │
│  │   "Eyes + Comms"     │    │       "Brain"              │  │
│  │                      │    │                            │  │
│  │  Servo sweep (20°)   │    │  Read sweep → distances    │  │
│  │  Ultrasonic read     │    │  IMU heading (gyro)        │  │
│  │  WiFi sync           │    │  Encoder odometry          │  │
│  │                      │    │  Obstacle avoidance        │  │
│  │  Shared: sweepData[] │◄──►│  A* pathfinding            │  │
│  │  Shared: command     │◄──►│  Motor control             │  │
│  │  Shared: telemetry   │◄──►│  Command processing        │  │
│  └─────────────────────┘    └───────────────────────────┘  │
│                    │                                        │
└────────────────────┼────────────────────────────────────────┘
                     │  WiFi (phone hotspot)
                     │  Single HTTP round-trip
                     │
┌────────────────────▼────────────────────────────────────────┐
│                   Flask Server (PC)                          │
│                                                              │
│  /api/robot/sync    ← ESP32 POST telemetry, GET command     │
│  /api/dashboard     ← Browser GET status+debug+chat (1 req) │
│  /api/control       ← Browser POST button press             │
│  /api/target        ← Browser POST map click                │
│  /api/ai/translate  ← Browser POST natural language         │
│                                                              │
│  threaded=True → handles ESP32 + browser in parallel        │
│  Disk I/O → background threads (never blocks response)      │
│                                                              │
│  Cloudflare tunnel → public URL (survives server restart)   │
└──────────────────────────────────────────────────────────────┘
                     │
                     │  HTTP polling (500ms)
                     │
┌────────────────────▼────────────────────────────────────────┐
│                Web Dashboard (Browser)                       │
│                                                              │
│  Single poll: /api/dashboard → status + debug + chat        │
│  Controls: D-pad, keyboard, map click, AI chat, voice       │
│  Display: grid map, sensor data, heading, battery, logs     │
└──────────────────────────────────────────────────────────────┘
```

---

## Boot Sequence

### 1. ESP32 Power On

```
setup() on Core 1
    │
    ├── Serial.begin(115200)
    ├── Wire.begin(SDA=11, SCL=12)        ← I2C bus for MPU6050
    │
    ├── motors.begin()                     ← TB6612FNG: PWM channels 0,1 + direction pins
    ├── sensors.begin()                    ← MPU6050 init + 2s gyro calibration
    ├── encoder.begin()                    ← Attach interrupts on pins 13,14
    ├── avoidance.begin()                  ← Reset state machine to IDLE
    ├── pathfinder.begin()                 ← Clear 40x40 grid
    ├── Debug::begin()                     ← Clear log buffer
    │
    ├── setupWiFi()                        ← Connect to phone hotspot (30 attempts)
    │
    └── xTaskCreatePinnedToCore(core0Task) ← Launch Core 0: sweep + WiFi
         │
         ├── Servo.attach(pin 9)           ← PWM channel auto-allocated
         ├── Servo → center (90°)          ← Park position
         └── Enter main loop               ← Start sweep + sync cycle
```

### 2. Server Start

```
python app.py
    │
    ├── Check for existing Cloudflare tunnel (reuse if alive)
    │   └── If none → start cloudflared as detached process
    │       └── Read URL from .cloudflared.log (up to 20s)
    │
    ├── Print connection info (LAN IPs, public IP, tunnel URL)
    │
    └── app.run(threaded=True)             ← Multi-threaded Flask
```

---

## Core 0 — Eyes + Communications

Runs continuously as a FreeRTOS task. Interleaves servo sweep with WiFi.

```
┌─────────────────────────────────────────────┐
│              SWEEP CYCLE (~100ms)            │
│                                              │
│  1. Move servo to angle                      │
│     angles: 20°, 40°, 60°, 80°, 100°,      │
│             120°, 140°, 160°                 │
│     (bounces: → right to left → back)        │
│                                              │
│  2. Wait 80ms for servo to settle            │
│     ┌──────────────────────────────┐         │
│     │ During wait: WiFi sync       │         │
│     │                              │         │
│     │ Every 100ms:                 │         │
│     │   POST /api/robot/sync       │         │
│     │   body: {} (command only)    │         │
│     │   response: pending command  │         │
│     │                              │         │
│     │ Every 1000ms:                │         │
│     │   POST /api/robot/sync       │         │
│     │   body: {telemetry + debug}  │         │
│     │   response: pending command  │         │
│     └──────────────────────────────┘         │
│                                              │
│  3. Read ultrasonic (2-20ms)                 │
│     └── pulseIn(ECHO, HIGH, 30000)           │
│     └── Filter: <2cm → 999 (motor noise)     │
│                                              │
│  4. Store in sweepData[step]                 │
│     └── portENTER_CRITICAL (spinlock)        │
│                                              │
│  5. Advance step (bounce at edges)           │
│     step 0→1→2→3→4→5→6→7→6→5→4→...         │
│                                              │
│  Full sweep: ~800ms (8 steps × 100ms)        │
└─────────────────────────────────────────────┘
```

### Sweep Data → Distance Zones

```
Servo angle:  20°   40°   60°   80°  100°  120°  140°  160°
Index:         0     1     2     3     4     5     6     7
Zone:        RIGHT        F-R  FRONT  FRONT  F-L       LEFT

distRight = min(index 0, 1)       ← angles ≤ 50°
distFront = min(index 2, 3, 4)    ← angles within ±30° of center
distLeft  = min(index 5, 6, 7)    ← angles ≥ 130°
```

---

## Core 1 — Brain

Runs as the Arduino `loop()` at 20Hz (50ms intervals).

```
┌─────────────────────────────────────────────┐
│            MAIN LOOP (every tick)            │
│                                              │
│  1. Read sweep distances                     │
│     └── Copy sweepData[] → local array       │
│     └── Derive distFront/Left/Right          │
│                                              │
│  2. Process pending command (if any)         │
│     └── forward/back/left/right/stop         │
│     └── auto/backtrack/reset/set_target      │
│     └── test_* → hardware tests              │
│                                              │
│  ┌─── Every 50ms (20Hz) ──────────────────┐ │
│  │                                         │ │
│  │  3. Update IMU heading (gyro only)      │ │
│  │     └── gz = (gyro.z - offset) × dt     │ │
│  │     └── heading += gz × dt              │ │
│  │     └── Wrap 0-360°                     │ │
│  │                                         │ │
│  │  4. Update encoder odometry             │ │
│  │     └── dL, dR → distance, position     │ │
│  │     └── posX, posY in world cm          │ │
│  │                                         │ │
│  │  5a. AUTO MODE:                         │ │
│  │      ├── Avoidance state machine        │ │
│  │      │   IDLE → SLOWDOWN → BRAKE →      │ │
│  │      │   REVERSING → TURNING → IDLE     │ │
│  │      ├── If not avoiding:               │ │
│  │      │   ├── Update obstacle map        │ │
│  │      │   └── Navigate A* path           │ │
│  │      └── Priority: Safety > Navigation  │ │
│  │                                         │ │
│  │  5b. MANUAL MODE:                       │ │
│  │      └── Emergency brake only (<15cm)   │ │
│  │          └── Blocks forward, allows     │ │
│  │              back/left/right/stop       │ │
│  │                                         │ │
│  │  6. Update telemetry snapshot           │ │
│  │     └── All sensor + state data         │ │
│  │     └── Protected by semaphore          │ │
│  └─────────────────────────────────────────┘ │
│                                              │
│  ┌─── Every 500ms ────────────────────────┐ │
│  │  7. Recalculate A* path                 │ │
│  │     └── 40×40 grid, 8-directional       │ │
│  │     └── Obstacle inflation              │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

---

## Server Communication

### ESP32 → Server: `/api/robot/sync` (POST)

One HTTP round-trip replaces the old 3 separate requests.

```
Request (command-only, every 100ms):
  POST /api/robot/sync
  body: {}

Request (with telemetry, every 1000ms):
  POST /api/robot/sync
  body: {
    "front": 45.2, "left": 120, "right": 88,
    "heading": 172.5, "gyro_rate": -0.3,
    "pos_x": 34.1, "pos_y": -12.8,
    "distance": 156, "enc_l": 153, "enc_r": 148,
    "battery": 7.62, "state": 0, "auto": true,
    "grid_x": 23, "grid_y": 19,
    ... (30+ fields)
    "debug": ["[BOOT] Ready", "[CMD] forward"]
  }

Response (always):
  {"cmd": "none"}
  or
  {"cmd": "forward"}
  or
  {"cmd": "set_target", "x": 100, "y": 50}
```

### Browser → Server: `/api/dashboard` (GET)

One request replaces 3 separate polls.

```
Request:
  GET /api/dashboard?d=42&c=10
  (d = debug log index, c = chat index)

Response:
  {
    "status": { ...all robot state fields... },
    "debug":  { "logs": [...new lines...], "total": 55 },
    "chat":   { "messages": [...new msgs...], "total": 12 }
  }
```

### Browser → Server: Control endpoints

```
POST /api/control     body: {"cmd": "forward"}     ← D-pad / keyboard
POST /api/target      body: {"x": 100, "y": 50}    ← Map click
POST /api/ai/translate body: {"text": "go forward 2 meters", "user": "Nate"}
POST /api/debug/clear                               ← Clear log
```

---

## Avoidance State Machine

```
                     distFront > 50cm
              ┌──────────────────────────┐
              ▼                          │
         ┌─────────┐              ┌──────────┐
         │  IDLE    │──< 30cm ──►│ SLOWDOWN  │
         │ (pass)   │              │ curve away│
         └─────────┘              └──────────┘
              ▲                       │ < 15cm
              │ > 50cm                ▼
         ┌─────────┐              ┌──────────┐
         │ TURNING  │◄── 300ms ──│  BRAKE    │
         │ spin away│              │ full stop │
         └─────────┘              └──────────┘
              │ timeout                │ 300ms
              ▼                        ▼
         ┌──────────┐            ┌──────────┐
         │  BRAKE   │◄───────── │ REVERSING │
         └──────────┘            │ back up   │
                                 └──────────┘
```

All transitions are non-blocking. No `delay()` calls — uses timer comparisons.
Distances come from continuous sweep (always fresh).

---

## Data Flow Summary

```
                    ┌──────────────┐
    Servo + HC-SR04 │   Core 0     │ WiFi HTTP
    (sweep 20° step)│              │ (sync endpoint)
         │          │  sweepData[] │      │
         │          └──────┬───────┘      │
         │                 │ spinlock      │
         │          ┌──────▼───────┐      │
    MPU6050 (I2C)   │   Core 1     │      │
    Encoders (ISR)  │              │      │
         │          │ dist F/L/R   │      │
         │          │ heading      │      │
         │          │ position     │      │
         │          │ avoidance    │      │
         │          │ pathfinding  │      │
         │          │ motors       │      │
         │          │ telemetry    │──────┘
         │          └──────────────┘  semaphore

    Core 0 owns:  Servo PWM, Ultrasonic GPIO, WiFi stack
    Core 1 owns:  I2C bus (MPU6050), Motor PWM, Encoder ISR
    No resource contention between cores.
```

---

## Timing Budget

| Operation | Interval | Duration | Core |
|-----------|----------|----------|------|
| Servo step + settle | ~100ms | 80ms wait | 0 |
| Ultrasonic read | per step | 2-20ms | 0 |
| WiFi sync (cmd only) | 100ms | 20-100ms | 0 |
| WiFi sync (telemetry) | 1000ms | 50-200ms | 0 |
| IMU heading update | 50ms | <1ms | 1 |
| Encoder update | 50ms | <1ms | 1 |
| Avoidance decision | 50ms | <1ms | 1 |
| A* pathfinding | 500ms | 1-10ms | 1 |
| Dashboard poll | 500ms | 5-20ms | Server |
| Full sweep cycle | ~800ms | 8 steps | 0 |

### Latency: Button Press → Motor Response

```
Old: Button → Server → ESP32 polls (200ms) → waits behind POST (200ms) → Core 1
     Total: 400-800ms

New: Button → Server → ESP32 sync (100ms) → command in response → Core 1
     Total: 100-200ms
```

---

## File Structure

```
src/
├── config.h          ← Pins, speeds, sweep angles, timing constants
├── sensors.h         ← IMU heading, ultrasonic read, sweep data struct
├── motors.h          ← TB6612FNG PWM control, direction, brake
├── encoder.h         ← Dual wheel odometry, position tracking
├── avoidance.h       ← Non-blocking obstacle avoidance state machine
├── pathfinder.h      ← A* 8-directional, obstacle inflation, breadcrumbs
├── debug.h           ← Ring buffer log, thread-safe
├── secrets.h         ← WiFi credentials, server address (gitignored)
└── robot_main.cpp    ← Setup, Core 1 loop, Core 0 task, sync, tests

server/
├── app.py            ← Flask API: sync, dashboard, control, AI, tunnel
├── ai_translator.py  ← Ollama LLM + rule-based fallback
├── templates/
│   └── dashboard.html ← Full dashboard UI + merged polling JS
├── start.bat         ← Start server (Windows)
├── restart.bat       ← Restart server (keeps tunnel alive)
└── logs/             ← telemetry.csv + debug.log (auto-created)
```
