# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

**Firmware (ESP32-S3):**
```bash
# PlatformIO CLI (may need full path on Windows)
~/.platformio/penv/Scripts/pio.exe run          # compile
~/.platformio/penv/Scripts/pio.exe run -t upload # flash
~/.platformio/penv/Scripts/pio.exe device monitor # serial 115200
```

**Server (Flask dashboard):**
```bash
cd server
pip install -r requirements.txt
python app.py                    # http://localhost:25565
```

**Secrets:** Copy `src/secrets.example.h` to `src/secrets.h` and fill in WiFi/server values. `secrets.h` is gitignored. A pre-push hook (`scripts/check_secrets.py`) scans for leaked credentials.

## Architecture

### Dual-Core FreeRTOS Split

- **Core 1 (`loop()`)** — Brain: 20Hz tick. Reads sweep data, integrates gyro heading, updates encoder odometry, marks obstacles on grid, runs navigator state machine, drives motors.
- **Core 0 (`core0Task`)** — Eyes + Comms: Owns the servo. Cycles through 5 sweep angles (20/55/90/125/160 deg), reads ultrasonic after each settle, then syncs with Flask server over HTTP.

### I2C Bus Safety

MPU6050 (Core 1) and WiFi HTTP (Core 0) both touch I2C timing. A `volatile bool wifiActive` flag gates MPU reads — Core 1 skips `mpu.getEvent()` whenever Core 0 is mid-HTTP. The Core 0 loop is ordered to keep I2C-safe: servo move -> settle delay -> ultrasonic GPIO read -> then HTTP (wifiActive=true).

### Inter-Core Data Flow

Three shared data channels, each with its own lock:
- **`sweepData` + `sweepMux` (spinlock):** Core 0 writes 5 ultrasonic distances; Core 1 reads via `updateFromSweep()`.
- **`telemetry` + `stateMutex` (mutex):** Core 1 snapshots all sensor/nav state; Core 0 reads it into JSON for HTTP sync.
- **`pendingCmd` + `cmdMux` (spinlock):** Core 0 writes commands from server response; Core 1 reads in `processCommand()`.

### Navigation

Reactive bearing-first (no A* pathfinding). Navigator state machine: `IDLE -> TURNING -> SCAN_AHEAD -> DRIVING -> AVOIDING -> REACHED`. Avoidance runs as a sub-state machine (brake -> reverse -> pick side -> heading-controlled turn -> recheck). In manual mode, avoidance still fires as a safety override.

### Sliding Grid

20x20 grid at 10cm/cell follows the robot. When the robot nears an edge (margin=4), the grid shifts and obstacle ages shift in lockstep. Obstacles decay after 30s. Grid is RLE-encoded for WiFi transmission.

### Slip Detection

Encoder-IMU cross-validation: if motors are active and encoders tick but accelerometer vibration variance is below threshold (no ground contact) for 8 consecutive cycles, traction factor (0.0-1.0) decays. Distance is multiplied by traction before updating position. Recovers when IMU detects real motion again.

### Server Communication

Single round-trip design: ESP32 POSTs to `/api/robot/sync` with debug logs + optional telemetry. Server responds with one pending command (then clears it). Dashboard polls `/api/dashboard` which merges status + debug + chat into one response.

## Key Conventions

- All firmware modules are **header-only** (`.h` files with inline implementations) except `robot_main.cpp`.
- Motor control priority: **stop > avoid > manual > autonomous**.
- Heading source is always **MPU6050 gyro**, never encoder differential.
- Config constants live in `config.h`; pin assignments, speeds, thresholds, timing are all there.
- `SWEEP_OFFSETS[]` in `robot_main.cpp` converts servo angles to heading-relative radians for obstacle mapping.
- Grid Y increases northward (world coords), but canvas Y=0 is top — all dashboard rendering flips Y.
