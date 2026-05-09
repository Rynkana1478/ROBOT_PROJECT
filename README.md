# ESP32-S3 4WD Robot Chassis

Autonomous 4WD robot with reactive obstacle avoidance, encoder + gyro odometry, and web dashboard control. Dual-core ESP32-S3 runs sensing and WiFi independently of the navigator.

## Features

- **Obstacle avoidance** — 5-angle ultrasonic sweep (20°/55°/90°/125°/160°) with directed-mode sub-patterns. Incremental-turn FSM: brake → pick side → turn 30° → check → drive past → re-bear toward target. Non-blocking, sensor-checked every tick.
- **Reactive bearing-first navigation** — Goes straight at the target, dodges around obstacles, then re-bears. Sweep mode locks to forward (`SWEEP_FRONT_LOCK`) while driving so the brake decision runs on fresh data, not stale.
- **Backtracking** — 100 breadcrumbs (20 m range) to retrace path home
- **Sliding 20×20 grid** — visualised on the dashboard as a live obstacle map (5 cm/cell); not used for path planning
- **Dual-core** — Core 0 fires ultrasonic + handles WiFi sync, Core 1 owns brain (avoidance + navigator + motors + servo)
- **Web dashboard** — Live sensor data, manual controls, destination input, grid map, hardware tests
- **Built-in charging** — 2S BMS + USB-C charger, never remove batteries
- **AI natural language control** — Ollama LLM translates "go forward 2m" to robot commands (offline, free)
- **Voice input** — Speak commands via phone mic (Chrome/Edge, EN/TH)
- **Multi-user chat** — Everyone sees all commands, usernames shown
- **Cloudflare tunnel** — Control from anywhere via public URL
- **WiFi debug logger** — Stream logs to dashboard, 9 hardware test commands

## Hardware

| Component | Purpose |
|-----------|---------|
| ESP32-S3 DevKitC-1 | Dual-core 240MHz, WiFi+BLE, 34 GPIO |
| TB6612FNG | Motor driver, 1.2A/ch, separate PWM+direction |
| 4WD Smart Car Chassis | Frame + 4 TT motors + wheels |
| HC-SR04 + SG90 Servo | Ultrasonic distance + 5-angle servo sweep (mode-switchable) |
| MPU6050 | Gyroscope heading (integrated over time) |
| Speed Encoders (x2) | Left + right wheel odometry |
| 2S 18650 (7.4V) | Rechargeable Li-ion power |
| 2S BMS 16A | Battery protection (overcharge/discharge/short) |
| USB-C 2S Charger | Built-in charging via phone charger cable |
| Buck Converter | 7.4V → 5V for ESP32 + sensors |

## Pin Wiring (ESP32-S3)

| GPIO | Connection |
|------|------------|
| 6 | TB6612 AIN1 (left dir) |
| 5 | TB6612 AIN2 (left dir) |
| 4 | TB6612 PWMA (left speed) |
| 15 | TB6612 BIN1 (right dir) |
| 16 | TB6612 BIN2 (right dir) |
| 17 | TB6612 PWMB (right speed) |
| 7 | TB6612 STBY |
| 18 | HC-SR04 Trigger |
| 3 | HC-SR04 Echo |
| 9 | Servo signal (50Hz LEDC) |
| 11 | I2C SDA (MPU6050) |
| 12 | I2C SCL (MPU6050) |
| 13 | Left encoder |
| 14 | Right encoder |
| 1 | Battery ADC (via divider) |

## Quick Setup

### Prerequisites

- [Python 3](https://www.python.org/downloads/)
- [PlatformIO CLI](https://platformio.org/install/cli) or [VS Code + PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- Phone with hotspot

### 1. Clone & Install

```bash
git clone https://github.com/Rynkana1478/robot-chassis.git
cd robot-chassis

# One-time: enable the pre-push secret scanner so credentials can't slip out
git config core.hooksPath .githooks

cd server
pip install -r requirements.txt
cd ..
```

### 2. Connect to Phone Hotspot

1. Turn on phone hotspot (2.4GHz)
2. Connect your PC to the same hotspot
3. Find your PC's IP:
   ```bash
   # Windows
   ipconfig
   # Look for "Wireless LAN" → IPv4 Address
   ```

### 3. Configure

**Create your local `src/secrets.h`** (gitignored — never committed):

```bash
cp src/secrets.example.h src/secrets.h
```

Then edit `src/secrets.h`:
```cpp
#define WIFI_SSID     "YourHotspot"       // Your phone hotspot name
#define WIFI_PASSWORD "YourPassword"      // Your hotspot password
#define SERVER_HOST   "192.168.43.100"    // Your PC's IP (or cloudflare URL)
#define SERVER_PORT   25565               // Match your server port
#define SERVER_HTTPS  false               // true if using cloudflare tunnel
#define CHASSIS_ID    "my-robot"          // Name shown on dashboard
```

### 4. Upload Firmware

```bash
pio run -t upload
```

### 5. Start Dashboard

```bash
cd server
python app.py
```

Open: **http://localhost:25565**

### 6. Drive

Dashboard shows **CONNECTED** → ready to go.

## Controls

| Action | Button | Keyboard |
|--------|--------|----------|
| Drive | FWD / BACK / LEFT / RIGHT | WASD or Arrow Keys |
| Stop | STOP | Space |
| Autonomous | AUTONOMOUS | — |
| Return home | BACKTRACK HOME | B |
| Reset position | RESET | R |
| Set destination | Type X,Y (cm) → Go To Target | — |
| Hardware tests | RUN ALL TESTS (debug panel) | — |

## Project Structure

```
robot_chassis/
├── src/                            # ESP32-S3 firmware
│   ├── config.h                    # Pins, speeds, WiFi, thresholds
│   ├── encoder.h                   # Wheel tick counting + gyro-based odometry
│   ├── motors.h                    # TB6612FNG with LEDC PWM
│   ├── sensors.h                   # Continuous sweep data + MPU6050 gyro heading
│   ├── navigator.h                 # FSM: turning / driving / avoid (incremental-turn)
│   ├── pathfinder.h                # World-coord target + 20x20 grid + breadcrumbs
│   ├── debug.h                     # WiFi debug log buffer
│   └── robot_main.cpp              # Dual-core main (Core 0 ultrasonic+WiFi, Core 1 brain+servo)
├── server/                         # PC dashboard server
│   ├── app.py                      # Flask REST API + AI command queue
│   ├── ai_translator.py            # Ollama LLM + rule-based fallback
│   ├── requirements.txt
│   ├── start.bat                   # Start server (Windows)
│   ├── restart.bat                 # Quick restart without killing tunnel
│   ├── Procfile                    # Cloud deployment (Render)
│   └── templates/
│       └── dashboard.html          # Dashboard + AI chat + voice + debug
├── test/                           # Standalone hardware tests
│   ├── 01_esp32_basic/             # Board, WiFi, GPIO
│   ├── 02_mpu6050/                 # Gyro + accelerometer
│   ├── 03_ultrasonic_servo/        # HC-SR04 + SG90 sweep
│   ├── 04_tb6612fng_motors/        # Motor driver + all 4 wheels
│   ├── 05_encoders/                # Dual wheel encoders
│   └── 06_battery_power/           # Voltage divider + charge level
├── platformio.ini                  # ESP32-S3 build config
├── component_list.md               # Parts list + wiring diagrams
├── WIRING_AND_ASSEMBLY.md          # Step-by-step build guide
└── GUIDE.md                        # Beginner explanation of everything
```

## Architecture

```
Phone Hotspot (2.4GHz WiFi)
  │
  ├── ESP32-S3 Robot
  │     ├── Core 0: Ultrasonic firing (Core 1 publishes target angle, Core 0 measures)
  │     │           + WiFi sync (independent task; never blocks sensors)
  │     │           Single /api/robot/sync endpoint (replaces 3 HTTP calls)
  │     └── Core 1: Brain — servo + navigator + avoidance + motors (20Hz)
  │                 Mode-switches sweep pattern (NORMAL / FRONT_LOCK / BYPASS)
  │                 to keep distFront fresh while driving forward
  │
  └── PC (Flask server + Ollama AI)
        ├── Single /api/dashboard endpoint (replaces 3 browser polls)
        ├── Chassis identified by X-Chassis-ID header (no auth token)
        ├── AI translator: natural language → robot commands
        ├── Voice input via Web Speech API
        ├── Shared chat log (all users see all commands)
        └── Cloudflare tunnel (auto-starts if cloudflared installed)

Priority: Safety (e-brake) > Manual command > Auto avoidance > Navigation
```

## Power System

```
USB-C charger cable
  └── [2S Charger Module] ── [2S BMS 16A] ── [2x 18650 cells]
                                    │
                              Protected 7.4V
                                    │
                    ┌───────────────┼────────────────┐
                    │               │                │
              [Buck → 5V]     TB6612 VM         [Voltage divider]
                    │          (motors)          → ESP32 ADC
              ┌─────┴─────┐
          ESP32 5V    HC-SR04
                      Servo
```

## Documentation

- **[GUIDE.md](GUIDE.md)** — How everything works (algorithms, concepts, accuracy)
- **[WIRING_AND_ASSEMBLY.md](WIRING_AND_ASSEMBLY.md)** — Build instructions + wire checklist
- **[component_list.md](component_list.md)** — Parts list + pin map + wiring diagrams
