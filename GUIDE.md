# ESP32-S3 4WD Robot Chassis - Beginner Guide

## What Is This Project?

A self-driving robot car that:
- **Avoids obstacles** using an ultrasonic sensor
- **Finds its way** to a destination you set (reactive bearing-first navigation)
- **Reports everything** to a dashboard on your computer
- **Drives itself** back home via breadcrumb backtracking
- Can be **manually controlled** from your browser or keyboard

You control it from a web dashboard running on your PC. Both the robot and your PC connect to your **phone's hotspot**.

---

## How The System Works

```
    Phone Hotspot (2.4GHz WiFi)
         │
    ┌────┴────┐
    │         │
[Your PC]  [ESP32-S3 Robot]
    │         │
    │ Flask   │ Core 1: Sensors + motors + reactive navigation (20Hz)
    │ server  │ Core 0: WiFi telemetry + commands (5Hz)
    │         │
    │ Robot POSTs sensor data → server stores it
    │ Robot GETs commands    ← server queues your clicks
    │
    │ Browser polls server for status (every 500ms)
    │ Browser sends your button clicks to server
```

The ESP32-S3 does NOT host a web page. It acts as a **client** that talks to your PC's Flask server. The two CPU cores run independently — WiFi never blocks the robot's sensors or motors.

---

## Your Components

| Part | What It Does |
|------|-------------|
| **ESP32-S3 DevKitC-1** | The brain. Dual-core 240MHz, WiFi built-in, 34 GPIO pins |
| **TB6612FNG** | Motor driver. Takes direction + speed signals from ESP32, sends power to motors. Separate direction pins + PWM speed pin per channel |
| **4WD Chassis + 4 Motors** | The frame and wheels. 4 TT gear motors wired as 2 pairs (left side + right side) |
| **HC-SR04 Ultrasonic** | Measures distance to objects. Sends a sound pulse, measures echo return time |
| **SG90 Servo** | Rotates the ultrasonic sensor left/center/right to scan for obstacles |
| **MPU6050** | Gyroscope + accelerometer. Gyro Z tracks rotation for heading. Accelerometer detects tilt |
| **Speed Encoders (x2)** | Slotted discs on left + right wheels. Count rotations to measure distance and detect turns |
| **2S 18650 Battery (7.4V)** | Two rechargeable Li-ion cells in series. Stable, high-current power |
| **2S BMS 16A** | Battery Management System. Protects against overcharge, over-discharge, and short circuits |
| **USB-C 2S Charger** | Built-in charging. Plug in a phone charger cable, never remove batteries |
| **Buck Converter** | Steps 7.4V down to 5V for ESP32 and sensors |

---

## How The Wiring Works

### Power System

```
USB-C charger cable
  └── [2S Charger] ── [2S BMS 16A] ── [18650 Cell 1 + Cell 2]
                            │
                      Protected 7.4V
                            │
              ┌─────────────┼─────────────┐
              │             │             │
        [Buck → 5V]   TB6612 VM     [Voltage divider]
              │        (motors)      → ESP32 ADC
         ┌────┴────┐
     ESP32 5V  HC-SR04
               Servo
```

The BMS protects the batteries automatically. The buck converter steps 7.4V down to 5V for the ESP32 and sensors. Motors get 7.4V directly through the TB6612FNG.

### Motor Wiring (TB6612FNG)

The TB6612FNG is different from simpler drivers. It has **separate pins for direction and speed**:

```
Direction (digital HIGH/LOW):     Speed (PWM):
  AIN1 + AIN2 → left motor dir     PWMA → left motor speed
  BIN1 + BIN2 → right motor dir    PWMB → right motor speed
```

| AIN1 | AIN2 | PWMA | Motor Action |
|------|------|------|-------------|
| HIGH | LOW | PWM value | Forward at speed |
| LOW | HIGH | PWM value | Backward at speed |
| HIGH | HIGH | any | Brake (locked) |
| LOW | LOW | any | Coast (free spin) |

PWM = Pulse Width Modulation. On ESP32-S3, we use LEDC hardware PWM channels. Value 0-1023 (10-bit): 0 = stop, 1023 = full speed.

The STBY (standby) pin must be HIGH for the driver to work. Setting it LOW puts the driver to sleep.

Each side has 2 motors wired **in parallel** — they share the same output and spin together.

### I2C Bus

The MPU6050 connects via I2C (2-wire protocol):
- **SDA** (data) = GPIO 11
- **SCL** (clock) = GPIO 12
- Address: `0x68`

**Important:** The MPU6050 VCC must connect **directly** to the ESP32's 3.3V pin with a short wire. Connecting through a breadboard rail causes voltage noise that corrupts I2C data.

### Ultrasonic Sensor (HC-SR04)

```
1. ESP32 sends a 10 microsecond pulse on TRIGGER pin
2. HC-SR04 sends an ultrasonic sound wave
3. Sound bounces off obstacle and returns
4. HC-SR04 sets ECHO pin HIGH for the round-trip duration
5. ESP32 measures that time
6. Distance = time / 58 (gives centimeters)
```

Max range: ~400cm. If no echo returns, it times out (we return 999 = no obstacle).

The sensor is mounted on a servo that sweeps left/center/right. This gives three distance readings from one sensor. The sweep is **non-blocking** — the servo moves one step per loop cycle, so the robot never freezes while scanning.

### Wheel Encoders

Two encoder discs (one on each side) count wheel rotations via hardware interrupts:
- 20 slots per revolution
- Wheel circumference = 20.4cm
- Each slot = ~1cm of distance
- Left encoder on GPIO 13, Right encoder on GPIO 14

With **two encoders**, the robot can detect:
- **Straight movement** — both counts increase equally
- **Turning** — one side counts more than the other
- **Spinning in place** — one counts up, the other counts up too (both wheels move)

This is called **differential odometry** and is much more accurate than a single encoder.

### ESP32-S3 Servo Control

ESP32 does **not** use the standard Arduino `Servo.h` library. It uses `ESP32Servo` which wraps the LEDC PWM hardware:

```cpp
#include <ESP32Servo.h>

servo.setPeriodHertz(50);          // 50Hz = standard servo frequency
servo.attach(pin, 500, 2400);     // min/max pulse width in microseconds
servo.write(90);                   // angle: 0-180 degrees
```

Our project allocates LEDC channels like this:
- Channel 0 = Left motor PWM (5kHz)
- Channel 1 = Right motor PWM (5kHz)
- Channel 2 = Servo (50Hz, auto-assigned by ESP32Servo)

---

## Understanding The Code Files

### `config.h` — All Settings

Every pin number, speed value, threshold, and WiFi credential lives here. The only file you need to edit before uploading.

Key settings:
```cpp
#define WIFI_SSID     "YourHotspot"
#define WIFI_PASSWORD "Password"
#define SERVER_HOST   "192.168.43.100"  // Your PC's IP on the hotspot
```

### `motors.h` — TB6612FNG Driver

Commands:
- `motors.forward(speed)` — both sides forward
- `motors.backward(speed)` — both sides backward
- `motors.turnLeft(speed)` — spin turn (left back, right forward)
- `motors.turnRight(speed)` — spin turn
- `motors.curveLeft(speed)` — gentle curve (left slow, right fast)
- `motors.stop()` — coast to stop
- `motors.brake()` — lock wheels immediately
- `motors.sleep()` / `motors.wake()` — standby mode

### `sensors.h` — Ultrasonic + MPU6050

**Ultrasonic sweep** (non-blocking state machine):
1. Servo moves to LEFT → read distance
2. Servo moves to CENTER → read distance
3. Servo moves to RIGHT → read distance
4. Servo returns to CENTER

Each step takes ~120ms. Total sweep ~480ms. Between steps, the main loop keeps running (checking front, updating encoders, etc).

**Heading** from MPU6050 gyroscope:
- Gyro Z measures rotation rate (degrees per second)
- Integrated over time: `heading += gyroRate * deltaTime`
- Auto-calibrated at boot (2 seconds, keep robot still)
- Dead zone filters out noise below 0.3 deg/s
- Drifts ~1-5 degrees per minute (acceptable for navigation)

### `encoder.h` — Dual Encoder Odometry

Both encoders use hardware interrupts (ESP32 supports interrupts on all GPIOs). Every 50ms:
1. Read how many ticks each encoder counted since last check
2. Average left + right ticks for distance
3. Use heading to calculate X,Y position change
4. Also compute heading change from encoder differential (sanity check)

Thread-safe: uses `portENTER_CRITICAL` to read volatile counters without race conditions.

### `avoidance.h` — Don't Hit Things

Non-blocking state machine with 6 states:

```
Front clear (>50cm)
┌───────────────────────┐
│       IDLE            │ ← Pathfinder controls motors
└───────┬───────────────┘
        │ Front < 30cm
┌───────▼───────────────┐
│     SLOWDOWN          │ ← Slow down, start sweep scan
└───────┬───────────────┘
        │ Front < 15cm
┌───────▼───────────────┐
│      BRAKE            │ ← Emergency stop, wait for sweep
└───────┬───────────────┘
        │ Sweep done
┌───────▼───────────────┐
│    REVERSING          │ ← Back up for 300ms
└───────┬───────────────┘
        │
┌───────▼───────────────┐
│     TURNING           │ ← Turn toward clearest direction
└───────┬───────────────┘
        │ Front clear?
        └──→ Back to IDLE
```

The front distance is checked **every loop cycle** (not just during avoidance). Even in manual mode, the safety system brakes if an obstacle is too close.

### `pathfinder.h` — Target + Grid + Breadcrumbs

The class is named "Pathfinder" historically, but in the current build it's a data structure, not a planner. Navigation is **reactive bearing-first** (`navigator.h`): the robot computes a straight-line bearing to the target and drives. When something blocks the path, the avoidance FSM dodges, then re-computes bearing from the new position.

**Target in world coordinates** — `targetWorldX` / `targetWorldY` are the goal point. `distToTarget(posX, posY)` returns straight-line distance; the navigator stops when that drops below `NAV_REACHED_CM`.

**Sliding 20×20 grid (5 cm/cell)** — visualised on the dashboard as a live obstacle map. When the robot approaches the edge of the visible window, the grid shifts so the robot stays roughly centered. **Not consulted by navigation** — the grid is for the human looking at the dashboard, not the robot's decisions.

**100 breadcrumbs** — waypoints dropped every 20 cm (covers ~20 m of travel). Backtracking pops them in reverse order, setting each as the next target.

**Why no A*** — the 5-angle ultrasonic sweep doesn't give the grid enough fidelity to plan against, and reactive avoidance with the right exit criteria gets the chassis past most obstacles. A planner could be added on top of the same grid if the sensor suite improves.

### `robot_main.cpp` — The Brain

**Dual-core operation:**
- **Core 1** (main loop): Sensors, encoders, servo, navigator, avoidance — runs at 20 Hz (every 50 ms)
- **Core 0** (ultrasonicTask + wifiTask): fires HC-SR04 when Core 1 publishes an angle; separate WiFi task POSTs telemetry/GETs commands

This is the biggest upgrade from v1. WiFi operations (which can block for up to 500ms on bad connections) **never** affect the robot's sensor loop or motor control.

**Thread safety:** Telemetry data is shared between cores via a mutex-protected struct. Core 1 writes sensor data, Core 0 reads it for HTTP POST.

### `debug.h` — WiFi Debug Logger

Since the ESP32-S3 keeps Serial alive (unlike ESP8266), you get debug output on both:
- **Serial Monitor** — prints sensor summary every second
- **WiFi dashboard** — streams log messages to the debug console panel

9 hardware test commands can be triggered from the dashboard to test each component individually.

---

## How Self-Tracking Works

### Dead Reckoning (Dual Encoders + Gyro)

```
Step 1: Both encoders count ticks
        Left: 5 ticks, Right: 5 ticks → "Moved 5.1cm straight"
        Left: 3 ticks, Right: 7 ticks → "Moved forward while turning right"

Step 2: Gyro reads rotation rate
        "Rotating at 12 deg/s"

Step 3: Integrate heading
        heading += 12 * deltaTime → new heading

Step 4: Convert distance + heading to X,Y
        deltaX = 5.1 * sin(heading)
        deltaY = 5.1 * cos(heading)

Step 5: Update position
        posX += deltaX
        posY += deltaY
```

This runs every **50ms** (20 times per second). Position is in **centimeters** relative to the start point (0, 0).

### Accuracy

**Dual encoders** are much better than single encoder:

| Configuration | Turn detection | Straight accuracy |
|--------------|----------------|-------------------|
| 1 encoder (v1) | Can't detect turns | ~1cm per tick |
| 2 encoders (v2) | Detects turns from L/R difference | Same, plus turn correction |

**Gyro drift:** ~1-5 degrees per minute. For short runs (< 2 minutes), this is negligible. For longer runs, the drift accumulates but the pathfinder constantly recalculates, so the robot still reaches its target.

**Target reached threshold:** 15cm. The robot considers the target "reached" when it gets within 15cm. This accounts for all accumulated errors.

---

## How To Run It

### Step 1: Phone Hotspot

Turn on hotspot (must be 2.4GHz). Note the name and password.

### Step 2: Connect PC to Hotspot

Join the hotspot from your laptop. Find your IP:
```bash
ipconfig
# Look for "Wireless LAN" → IPv4 Address (e.g., 192.168.43.100)
```

### Step 3: Install AI (Optional but Recommended)

```bash
# Install Ollama (local AI, free, offline)
winget install Ollama.Ollama
ollama pull llama3.2
```

Without Ollama, the AI chat still works using rule-based parsing (simpler but handles common commands).

### Step 4: Start Server

```bash
cd robot_chassis/server
pip install -r requirements.txt
start.bat
```

Dashboard: `http://localhost:25565`

### Step 5: Remote Access (Cloudflare Tunnel)

If your PC stays at home and robot goes to a contest:
```bash
winget install Cloudflare.cloudflared
cloudflared tunnel --url http://localhost:25565
```

Gives you a public URL like `https://random-words.trycloudflare.com`. Anyone can open it from anywhere.

### Step 6: Configure & Upload

Edit `src/config.h` with your hotspot name, password, and server URL. Then:
```bash
cd robot_chassis
pio run -t upload
```

### Step 7: Drive

1. Power on the robot (battery switch)
2. Wait ~3 seconds (gyro calibration — keep still!)
3. Dashboard shows **CONNECTED** in green
4. Set destination in cm → **Go To Target**
5. Or type in AI chat: "go forward 2 meters"
6. Press **BACKTRACK HOME** to return

---

## AI Natural Language Control

The dashboard has an AI chat panel where you type (or speak) commands in plain English. A local LLM (Ollama Llama 3.2) translates your words into robot commands.

### How It Works

```
You type:  "go forward 2 meters then come back"
    ↓
Ollama LLM (runs on your PC, offline, free)
    ↓
Translates to:
  [1] move_relative forward:200  (relative to robot facing)
  [2] backtrack                  (return to start)
    ↓
Server resolves relative using robot heading:
  Robot facing east (90°) → set_target x:+200, y:0
    ↓
ESP32 receives set_target → reactive bearing-first navigation → robot moves
```

### Relative vs Absolute Directions

| You Say | Type | What Happens |
|---------|------|-------------|
| "go forward 2m" | **Relative** | Moves in direction robot is facing |
| "go left 1m" | **Relative** | Moves to robot's left side |
| "go north 2m" | **Absolute** | Always moves toward world north |
| "go east 1m" | **Absolute** | Always moves toward world east |

This is important: "forward" depends on where the robot faces, "north" is always the same direction.

### Priority Commands (Instant, 0ms)

These words bypass the AI completely for instant response:
- **Stop:** stop, halt, freeze, emergency, quick stop, s
- **Return:** come back, go home, backtrack, return
- **Reset:** reset, restart

### Without Ollama

If Ollama is not running, the system falls back to rule-based parsing. It handles simple commands ("go forward 2m", "stop", "come back") but can't understand complex or ambiguous sentences.

---

## Voice Input

Click the microphone button or press it on phone to speak commands. The browser converts speech to text, then the AI translates it to robot commands.

- **EN mode:** Speak English — "go forward two meters"
- **TH mode:** Speak Thai — the dashboard translates Thai direction words to English before sending to AI

Voice works on Chrome and Edge (Android and desktop). Safari and Firefox don't support the Web Speech API.

**Tip:** Voice is cool for demos but text is more reliable. Use voice for simple commands ("stop", "forward"), text for precise ones ("go northwest 2.5m then come back").

---

## Multi-User Access

Anyone who opens the dashboard URL sees the same robot state and can send commands. Type your name in the "Name" field to identify yourself in the chat.

```
Dashboard chat log:
  [Referee] go north 3 meters
  [AI] Forward 300cm (absolute)
  [Staff] stop
  [AI] EMERGENCY STOP
  [Viewer] come back
  [AI] Returning to start
```

All commands go to the same robot. Everyone sees everything in real-time. No app install needed — just share the URL.

---

## Dashboard Reference

| Action | Button | Keyboard |
|--------|--------|----------|
| Drive forward | FWD | W or Arrow Up |
| Drive backward | BACK | S or Arrow Down |
| Turn left | LEFT | A or Arrow Left |
| Turn right | RIGHT | D or Arrow Right |
| Stop | STOP | Space |
| Autonomous mode | AUTONOMOUS | — |
| Return home | BACKTRACK HOME | B |
| Reset position | RESET | R |
| Set destination | Type X,Y in cm → Go To Target | — |
| Run hardware tests | Test buttons in debug panel | — |

---

## Hardware Test Sequence

Before assembling, test each component individually using the standalone tests in `test/`:

| # | Test | What To Connect | Command |
|---|------|----------------|---------|
| 01 | ESP32 Basic | Just the board | `cd test/01_esp32_basic && pio run -t upload -t monitor` |
| 02 | MPU6050 | + MPU6050 (SDA=11, SCL=12, 3.3V direct) | `cd test/02_mpu6050 && pio run -t upload -t monitor` |
| 03 | Ultrasonic+Servo | + HC-SR04 + SG90 (needs 5V) | `cd test/03_ultrasonic_servo && ...` |
| 04 | Motors | + TB6612FNG + motors (needs 7.4V) | `cd test/04_tb6612fng_motors && ...` |
| 05 | Encoders | + 2 encoder modules (push by hand) | `cd test/05_encoders && ...` |
| 06 | Battery | + voltage divider on GPIO1 | `cd test/06_battery_power && ...` |

Each test is fully standalone with its own `platformio.ini`. Pass all tests before assembling the full robot.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Dashboard says DISCONNECTED | Check WiFi SSID/password in config.h. Check SERVER_HOST = your PC's IP. Both on same hotspot? |
| Motors don't move | Check TB6612 wiring. STBY pin → GPIO 17 (must be HIGH). VM → 7.4V. VCC → 3.3V |
| Motors wrong direction | Swap AIN1/AIN2 wires (left) or BIN1/BIN2 (right) at TB6612 |
| Left/right swapped | Swap AIN/BIN channel wires at TB6612 |
| Distance shows 999 | HC-SR04 needs 5V (from buck converter). Check TRIG→GPIO18, ECHO→GPIO8 |
| Heading doesn't change | MPU6050 not responding. Check I2C: SDA→GPIO11, SCL→GPIO12, VCC→3.3V **direct** |
| Encoder stays at 0 | Check VCC→3.3V, OUT→GPIO13/14. Encoder disc must be on shaft, sensor facing slots |
| Battery voltage wrong | Check voltage divider: 220K top + 33K bottom. Tap point → GPIO 1 |
| Robot drifts off course | Normal for gyro-only heading. Recalibrate by keeping still at boot |
| Servo doesn't move | ESP32 uses ESP32Servo library, not standard Servo. Check GPIO 9, needs 5V power |
| WiFi drops frequently | Phone hotspot may auto-sleep. Disable auto-off in hotspot settings |

---

## Key Concepts

### PWM (Pulse Width Modulation)
Rapidly switches a pin on/off. The on-time ratio controls motor speed. 0 = always off, 1023 = always on, 512 = half speed. On ESP32-S3, this is done by dedicated LEDC hardware — no CPU overhead.

### LEDC (LED Control)
ESP32's hardware PWM peripheral. Has 8 independent channels, each with configurable frequency and resolution. We use 3: two for motors (5kHz, 10-bit) and one for servo (50Hz, auto-assigned).

### Interrupt
Hardware feature that runs a tiny function instantly when a pin changes state. Used for encoders — the ESP32 counts every tick even while doing other work. All ESP32 GPIO pins support interrupts (unlike ESP8266 which had restrictions).

### FreeRTOS Dual-Core
The ESP32-S3 has two CPU cores. FreeRTOS (built into ESP32 Arduino) lets us run separate tasks on each core. `xTaskCreatePinnedToCore()` assigns a function to a specific core. We use this to run WiFi on Core 0 while the robot logic runs on Core 1.

### Mutex
A lock that prevents two cores from accessing shared data at the same time. When Core 1 writes sensor data and Core 0 reads it for WiFi, the mutex ensures one waits for the other. Prevents corrupted data.

### Dead Reckoning
Estimating position by measuring movement from a known starting point. Like walking blindfolded: count steps, remember turns. Drifts over time because small errors accumulate. Our system uses dual encoders + gyro to minimize drift.

### Reactive Bearing-First Navigation
Compute a straight-line bearing from current position to target. Turn until the chassis faces that bearing, then drive forward. If something blocks the path, the avoidance FSM dodges; on exit the bearing is recomputed from the new position and the robot keeps going. No A*, no plan ahead — every decision is based on what the sensors see right now.

### Incremental-Turn Avoidance
Front cone sees an obstacle → brake → pick the more open side from `distLeft`/`distRight` → turn 30° toward it → check if the new heading is clear → if yes, drive 50 cm forward and exit; if no, turn another 30°, repeat. Hard cap at 120° total before giving up (`NAV_FAILED`). The sweep is locked to forward (`SWEEP_FRONT_LOCK`) while driving so the brake isn't acting on stale data.

### Sliding Window Grid (visualization)
A 20×20 grid at 5 cm/cell that follows the robot. When the robot approaches the edge, the grid shifts — old data scrolls off, new "unknown" cells scroll in. Used by the dashboard to show a live obstacle map; not consulted by navigation.

### Breadcrumb Trail
The robot drops position markers every 20 cm as it moves (up to 100 = 20 m range). To return home, it follows the markers in reverse — each crumb becomes the next target, and the navigator drives toward it. Works even if new obstacles appeared after the outbound trip, because avoidance still fires.

### Complementary Filter
Blends two sensors: gyro (fast, accurate short-term, drifts long-term) with compass (slow, accurate long-term, noisy short-term). Our project is currently gyro-only, but the code supports adding a compass later with the formula:
`heading = 0.98 * gyro_prediction + 0.02 * compass_reading`

### Client-Server Architecture
The robot is a **client** — sends data to and receives commands from the server. Your browser is also a client. The Flask **server** sits in the middle, storing state and relaying messages. All three connect through the phone hotspot.
