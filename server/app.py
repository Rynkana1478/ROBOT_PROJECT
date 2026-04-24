"""
ESP32-S3 4WD Robot Server

Can run locally (LAN) or deployed to cloud (internet).
No authentication — intended for trusted local network use.

Local:  python app.py
Cloud:  gunicorn app:app (Render/Railway auto-detect this)

Dashboard: http://localhost:5000  (or your cloud URL)
"""

import os
import json
from flask import Flask, render_template, request, jsonify
from collections import deque
import time
from ai_translator import translate as ai_translate, check_ollama

app = Flask(__name__)

# --- Chassis tracking ---
chassis_info = {
    "id": None,
    "last_seen": 0,
}

# --- Debug log buffer ---
debug_logs = deque(maxlen=200)

# --- Robot state ---
robot_state = {
    "front": 0, "left": 0, "right": 0,
    "heading": 0, "compass_heading": -1, "gyro_rate": 0,
    "pos_x": 0, "pos_y": 0,
    "distance": 0, "enc_l": 0, "enc_r": 0,
    "battery": 0,
    "state": 0, "state_name": "IDLE",
    "path_length": 0, "auto": True,
    "grid_x": 20, "grid_y": 20,
    "target_x": 20, "target_y": 20,
    "target_wx": 0, "target_wy": 0,
    "has_target": False,
    "target_reached": False,
    "backtracking": False,
    "crumbs": 0,
    "compass_ok": False, "mpu_ok": False, "enc_healthy": True,
    "grid": "",
    "connected": False,
    "last_update": 0,
    "debug_mode": False,
    "wifi_rssi": 0,
    "free_heap": 0,
    "uptime": 0,
}

pending_command = {"cmd": "none"}

STATE_NAMES = {
    0: "IDLE", 1: "SLOWDOWN", 2: "BRAKE",
    3: "REVERSING", 4: "TURNING",
}

GRID_SIZE = 40


# ============================================
# Dashboard (serves HTML)
# ============================================
@app.route("/")
def dashboard():
    return render_template("dashboard.html", grid_size=GRID_SIZE)


# ============================================
# Robot <-> Server (single sync endpoint)
# ============================================
@app.route("/api/robot/sync", methods=["POST"])
def robot_sync():
    """Single round-trip endpoint for ESP32.

    Accepts JSON with optional telemetry fields at root level
    (when includeTelemetry=true on ESP32 side) and optional
    "debug" array of log strings.

    Returns the pending command (and clears it).
    """
    global pending_command

    # Track chassis identity
    cid = request.headers.get("X-Chassis-ID")
    if cid:
        chassis_info["id"] = cid
        chassis_info["last_seen"] = time.time()

    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "no json"}), 400

    # --- Debug logs ---
    if "debug" in data:
        ts = time.strftime("%H:%M:%S")
        for msg in data["debug"]:
            debug_logs.append(f"[{ts}] {msg}")
            print(f"  [ROBOT] {msg}")

    # --- Telemetry (fields at root level; check for "front" key) ---
    if "front" in data:
        prev_reached = bool(robot_state.get("target_reached"))

        robot_state.update({k: data.get(k, robot_state.get(k, 0)) for k in data if k != "debug"})
        robot_state["state_name"] = STATE_NAMES.get(data.get("state", 0), "UNKNOWN")
        robot_state["connected"] = True
        robot_state["last_update"] = time.time()

        # Drain the multi-step AI queue on rising edge of target_reached.
        now_reached = bool(robot_state.get("target_reached"))
        if now_reached and not prev_reached and ai_command_queue:
            cmd = ai_command_queue.popleft()
            _execute_ai_command(cmd)
            ts = time.strftime("%H:%M:%S")
            debug_logs.append(f"[{ts}] [AI] Auto-drain: {json.dumps(cmd)} ({len(ai_command_queue)} left)")

    # --- Return pending command (and clear it) ---
    cmd = pending_command.copy()
    pending_command = {"cmd": "none"}
    return jsonify(cmd)


# ============================================
# Dashboard -> Server (single dashboard endpoint)
# ============================================
@app.route("/api/dashboard", methods=["GET"])
def get_dashboard():
    """Merged status + debug + chat for the dashboard.

    Query params:
        debug_since (int): return debug logs from this index onward
        chat_since  (int): return chat messages from this index onward
    """
    # Status
    state = robot_state.copy()
    if time.time() - state["last_update"] > 3:
        state["connected"] = False

    # Debug logs
    debug_since = int(request.args.get("debug_since", 0))
    all_debug = list(debug_logs)
    if debug_since < len(all_debug):
        debug_slice = all_debug[debug_since:]
    else:
        debug_slice = []

    # Chat messages
    chat_since = int(request.args.get("chat_since", 0))
    all_chat = list(chat_log)
    if chat_since < len(all_chat):
        chat_slice = all_chat[chat_since:]
    else:
        chat_slice = []

    return jsonify({
        "status": state,
        "debug": {"logs": debug_slice, "total": len(all_debug)},
        "chat": {"messages": chat_slice, "total": len(all_chat)},
    })


# ============================================
# Dashboard -> Server (control endpoints)
# ============================================
@app.route("/api/control", methods=["POST"])
def send_control():
    global pending_command
    data = request.get_json(silent=True)
    if not data or "cmd" not in data:
        return jsonify({"error": "missing cmd"}), 400

    cmd = data["cmd"]
    valid = ("forward", "back", "left", "right", "stop", "auto",
             "backtrack", "reset",
             "test_all", "test_i2c", "test_ultrasonic", "test_servo",
             "test_mpu", "test_compass", "test_motors", "test_encoder",
             "test_battery")
    if cmd not in valid:
        return jsonify({"error": "invalid cmd"}), 400

    pending_command = {"cmd": cmd}
    return jsonify({"ok": True, "cmd": cmd})


@app.route("/api/target", methods=["POST"])
def set_target():
    global pending_command, robot_state
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "no json"}), 400

    x = data.get("x")
    y = data.get("y")
    if x is None or y is None:
        return jsonify({"error": "missing x or y"}), 400

    pending_command = {"cmd": "set_target", "x": float(x), "y": float(y)}
    robot_state["target_wx"] = float(x)
    robot_state["target_wy"] = float(y)
    return jsonify({"ok": True, "x": x, "y": y})


@app.route("/api/debug/clear", methods=["POST"])
def clear_debug_logs():
    debug_logs.clear()
    return jsonify({"ok": True})


# ============================================
# AI Command Translator
# ============================================
ai_command_queue = deque()  # Multi-step commands from AI
ai_log = deque(maxlen=50)   # AI decision history
chat_log = deque(maxlen=100)  # Shared chat: everyone sees all messages

@app.route("/api/ai/translate", methods=["POST"])
def ai_translate_endpoint():
    """Translate natural language to robot commands via Ollama/rules."""
    global pending_command
    data = request.get_json(silent=True)
    if not data or "text" not in data:
        return jsonify({"error": "missing text"}), 400

    user_text = data["text"].strip()
    username = data.get("user", "Anonymous").strip() or "Anonymous"
    if not user_text:
        return jsonify({"error": "empty text"}), 400

    # Translate
    result = ai_translate(user_text)
    result["user"] = username

    # Log to shared chat
    ts = time.strftime("%H:%M:%S")
    chat_log.append({"time": ts, "user": username, "type": "user", "text": user_text})

    # Log to AI history
    ai_log.append(result)

    # Log to debug console
    debug_logs.append(f"[{ts}] [{username}] \"{user_text}\"")
    debug_logs.append(f"[{ts}] [AI] Method: {result['method']} ({result['duration_ms']}ms)")
    print(f"  [{username}] \"{user_text}\" -> {result['method']} ({result['duration_ms']}ms)")

    if result.get("error"):
        chat_log.append({"time": ts, "user": "AI", "type": "error", "text": result['error']})
        debug_logs.append(f"[{ts}] [AI] ERROR: {result['error']}")
        print(f"  [AI] ERROR: {result['error']}")
        return jsonify(result)

    chat_log.append({"time": ts, "user": "AI", "type": "ai", "text": result['explanation']})
    for cmd in result.get("commands", []):
        chat_log.append({"time": ts, "user": "AI", "type": "cmd", "text": json.dumps(cmd)})

    debug_logs.append(f"[{ts}] [AI] {result['explanation']}")
    print(f"  [AI] {result['explanation']}")

    # Queue commands
    commands = result.get("commands", [])
    for i, cmd in enumerate(commands):
        debug_logs.append(f"[{ts}] [AI] Command [{i+1}/{len(commands)}]: {json.dumps(cmd)}")
        print(f"  [AI] Command [{i+1}/{len(commands)}]: {cmd}")

    # Show full JSON that will be sent to chassis
    if commands:
        full_json = json.dumps(commands)
        debug_logs.append(f"[{ts}] [AI] >>> JSON to chassis: {full_json}")
        print(f"  [AI] >>> JSON to chassis: {full_json}")

        # Execute first command immediately, queue the rest
        first = commands[0]
        pending_json = json.dumps(_to_pending(first))
        debug_logs.append(f"[{ts}] [AI] >>> pending_command = {pending_json}")
        print(f"  [AI] >>> pending_command = {pending_json}")
        _execute_ai_command(first)

        for cmd in commands[1:]:
            ai_command_queue.append(cmd)

        if len(commands) > 1:
            debug_logs.append(f"[{ts}] [AI] Queued {len(commands)-1} more command(s)")

    return jsonify(result)


@app.route("/api/ai/status", methods=["GET"])
def ai_status():
    """Get AI system status and recent history."""
    return jsonify({
        "ollama_available": check_ollama(),
        "queue_length": len(ai_command_queue),
        "history": list(ai_log),
    })


@app.route("/api/ai/next", methods=["POST"])
def ai_next_command():
    """Execute next queued AI command (called when robot reaches target)."""
    if ai_command_queue:
        cmd = ai_command_queue.popleft()
        _execute_ai_command(cmd)
        ts = time.strftime("%H:%M:%S")
        debug_logs.append(f"[{ts}] [AI] Next queued command: {json.dumps(cmd)}")
        return jsonify({"ok": True, "cmd": cmd, "remaining": len(ai_command_queue)})
    return jsonify({"ok": True, "cmd": None, "remaining": 0})


import math

def _resolve_relative(cmd):
    """Convert move_relative to set_target using robot's current heading and position."""
    fwd = cmd.get("forward", 0)
    right = cmd.get("right", 0)

    heading_deg = robot_state.get("heading", 0)
    heading_rad = math.radians(heading_deg)
    pos_x = robot_state.get("pos_x", 0)
    pos_y = robot_state.get("pos_y", 0)

    # Forward = along heading, Right = 90 degrees clockwise from heading
    dx = fwd * math.sin(heading_rad) + right * math.cos(heading_rad)
    dy = fwd * math.cos(heading_rad) - right * math.sin(heading_rad)

    return {
        "action": "set_target",
        "x": round(pos_x + dx, 1),
        "y": round(pos_y + dy, 1)
    }

def _to_pending(cmd):
    """Convert AI command to the format ESP32 expects."""
    action = cmd.get("action", "")
    if action == "move_relative":
        cmd = _resolve_relative(cmd)
        action = "set_target"
    if action == "set_target":
        return {"cmd": "set_target", "x": float(cmd.get("x", 0)), "y": float(cmd.get("y", 0))}
    elif action == "backtrack":
        return {"cmd": "backtrack"}
    elif action == "stop":
        return {"cmd": "stop"}
    elif action == "reset":
        return {"cmd": "reset"}
    return {"cmd": "none"}

def _execute_ai_command(cmd):
    """Convert an AI command dict to a pending robot command."""
    global pending_command
    action = cmd.get("action", "")

    # Resolve relative to absolute first
    if action == "move_relative":
        resolved = _resolve_relative(cmd)
        ts = time.strftime("%H:%M:%S")
        debug_logs.append(f"[{ts}] [AI] Relative -> Absolute: heading={robot_state.get('heading',0):.0f} pos=({robot_state.get('pos_x',0):.0f},{robot_state.get('pos_y',0):.0f}) -> target=({resolved['x']},{resolved['y']})")
        print(f"  [AI] Relative -> Absolute: fwd={cmd.get('forward',0)} right={cmd.get('right',0)} -> x={resolved['x']} y={resolved['y']}")
        cmd = resolved
        action = "set_target"

    if action == "set_target":
        x = cmd.get("x", 0)
        y = cmd.get("y", 0)
        pending_command = {"cmd": "set_target", "x": float(x), "y": float(y)}
        robot_state["target_wx"] = float(x)
        robot_state["target_wy"] = float(y)
    elif action == "backtrack":
        pending_command = {"cmd": "backtrack"}
    elif action == "stop":
        pending_command = {"cmd": "stop"}
    elif action == "reset":
        pending_command = {"cmd": "reset"}


# ============================================
# Health check (for cloud platforms)
# ============================================
@app.route("/health")
def health():
    return jsonify({"status": "ok", "connected": robot_state["connected"]})


def _lan_ips():
    """Every non-loopback IPv4 the host has, so the phone knows which to use."""
    import socket
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                ips.add(ip)
    except socket.gaierror:
        pass
    return sorted(ips)


def _public_ip(timeout=1.5):
    """Best-effort public IP. Returns None if offline or the endpoint is slow."""
    import requests
    for url in ("https://api.ipify.org", "https://ifconfig.me/ip"):
        try:
            r = requests.get(url, timeout=timeout)
            if r.status_code == 200 and r.text.strip():
                return r.text.strip()
        except requests.RequestException:
            continue
    return None


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 25565))
    ddns = os.environ.get("DDNS_HOST", "blackwise.thddns.net")
    ddns_port = os.environ.get("DDNS_PORT", "5570")

    print("=" * 60)
    print("  4WD Robot Server")
    cid = chassis_info.get("id")
    print(f"  Chassis:    {cid if cid else '<waiting for first sync>'}")
    print(f"  Local:      http://localhost:{port}")
    for ip in _lan_ips():
        print(f"  LAN:        http://{ip}:{port}")
    print(f"  DDNS:       http://{ddns}:{ddns_port}")
    pub = _public_ip()
    if pub:
        print(f"  Public IP:  {pub}")
    print("=" * 60)

    app.run(host="0.0.0.0", port=port, debug=os.environ.get("FLASK_DEBUG") == "1", threaded=True)
