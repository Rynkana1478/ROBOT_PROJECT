"""
ESP32-S3 4WD Robot Server

Local:  python app.py
Cloud:  gunicorn app:app (Render/Railway auto-detect this)

Dashboard: http://localhost:5000  (or your cloud URL)
"""

import os
import json
import math
import re
import subprocess
import threading
import csv
from datetime import datetime
from flask import Flask, render_template, request, jsonify
from collections import deque
import time
from ai_translator import translate as ai_translate, check_ollama

app = Flask(__name__)

# --- Debug log buffer ---
debug_logs = deque(maxlen=200)

# --- Robot state ---
robot_state = {
    "front": 0, "left": 0, "right": 0,
    "heading": 0, "gyro_rate": 0,
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
    "mpu_ok": False,
    "connected": False,
    "last_update": 0,
    "debug_mode": False,
    "wifi_rssi": 0,
    "free_heap": 0,
    "uptime": 0,
    "accel_x": 0, "accel_y": 0, "accel_z": 0,
    "gyro_x": 0, "gyro_y": 0,
    "mpu_temp": 0,
}

pending_command = {"cmd": "none"}

# --- Telemetry logging (background) ---
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(LOG_DIR, exist_ok=True)
_last_log_time = 0

TELEMETRY_FIELDS = [
    "timestamp", "front", "left", "right", "heading", "gyro_rate",
    "pos_x", "pos_y", "distance", "enc_l", "enc_r",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y",
    "mpu_temp", "battery", "state", "state_name", "auto",
]

def _bg_write_telemetry(data):
    global _last_log_time
    now = time.time()
    if now - _last_log_time < 1.0:
        return
    _last_log_time = now
    threading.Thread(target=_write_csv, args=(data.copy(),), daemon=True).start()

def _write_csv(data):
    log_file = os.path.join(LOG_DIR, "telemetry.csv")
    exists = os.path.exists(log_file) and os.path.getsize(log_file) > 0
    try:
        with open(log_file, "a", newline="") as f:
            writer = csv.writer(f)
            if not exists:
                writer.writerow(TELEMETRY_FIELDS)
            writer.writerow([
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                *[data.get(k, "") for k in TELEMETRY_FIELDS[1:]]
            ])
    except Exception:
        pass

STATE_NAMES = {
    0: "IDLE", 1: "SLOWDOWN", 2: "BRAKE",
    3: "REVERSING", 4: "TURNING",
}

GRID_SIZE = 40


# ============================================
# Dashboard
# ============================================
@app.route("/")
def dashboard():
    return render_template("dashboard.html", grid_size=GRID_SIZE)


# ============================================
# Robot <-> Server: Single sync endpoint
# ESP32 POSTs telemetry + debug, gets command back
# ============================================
@app.route("/api/robot/sync", methods=["POST"])
def robot_sync():
    global robot_state, pending_command
    data = request.get_json(silent=True) or {}

    # Update telemetry if included (has sensor fields)
    if "front" in data:
        skip = {"debug"}
        robot_state.update({k: data[k] for k in data if k not in skip})
        robot_state["state_name"] = STATE_NAMES.get(data.get("state", 0), "UNKNOWN")
        robot_state["connected"] = True
        robot_state["last_update"] = time.time()
        _bg_write_telemetry(robot_state)

        # AI queue drain on target_reached rising edge
        if data.get("target_reached") and ai_command_queue:
            cmd = ai_command_queue.popleft()
            _execute_ai_command(cmd)
            ts = time.strftime("%H:%M:%S")
            debug_logs.append(f"[{ts}] [AI] Auto-drain: {json.dumps(cmd)} ({len(ai_command_queue)} left)")

    # Process debug logs if included
    if "debug" in data:
        ts = time.strftime("%H:%M:%S")
        debug_log_file = os.path.join(LOG_DIR, "debug.log")
        for msg in data["debug"]:
            line = f"[{ts}] {msg}"
            debug_logs.append(line)
            print(f"  [ROBOT] {msg}")
        threading.Thread(
            target=_write_debug_lines,
            args=([f"[{ts}] {m}" for m in data["debug"]],),
            daemon=True
        ).start()

    # Return and consume pending command
    cmd = pending_command.copy()
    pending_command = {"cmd": "none"}
    return jsonify(cmd)


def _write_debug_lines(lines):
    try:
        with open(os.path.join(LOG_DIR, "debug.log"), "a") as f:
            for line in lines:
                f.write(line + "\n")
    except Exception:
        pass


# ============================================
# Dashboard: Single merged poll endpoint
# Returns status + debug + chat in one response
# ============================================
@app.route("/api/dashboard", methods=["GET"])
def dashboard_data():
    state = robot_state.copy()
    if time.time() - state["last_update"] > 3:
        state["connected"] = False

    since_debug = int(request.args.get("d", 0))
    since_chat = int(request.args.get("c", 0))

    logs = list(debug_logs)
    messages = list(chat_log)

    return jsonify({
        "status": state,
        "debug": {
            "logs": logs[since_debug:] if since_debug < len(logs) else [],
            "total": len(logs),
        },
        "chat": {
            "messages": messages[since_chat:] if since_chat < len(messages) else [],
            "total": len(messages),
        },
    })


# ============================================
# Dashboard -> Server (browser control endpoints)
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
ai_command_queue = deque()
ai_log = deque(maxlen=50)
chat_log = deque(maxlen=100)

@app.route("/api/ai/translate", methods=["POST"])
def ai_translate_endpoint():
    global pending_command
    data = request.get_json(silent=True)
    if not data or "text" not in data:
        return jsonify({"error": "missing text"}), 400

    user_text = data["text"].strip()
    username = data.get("user", "Anonymous").strip() or "Anonymous"
    if not user_text:
        return jsonify({"error": "empty text"}), 400

    result = ai_translate(user_text)
    result["user"] = username

    ts = time.strftime("%H:%M:%S")
    chat_log.append({"time": ts, "user": username, "type": "user", "text": user_text})
    ai_log.append(result)
    debug_logs.append(f"[{ts}] [{username}] \"{user_text}\"")
    debug_logs.append(f"[{ts}] [AI] Method: {result['method']} ({result['duration_ms']}ms)")
    print(f"  [{username}] \"{user_text}\" -> {result['method']} ({result['duration_ms']}ms)")

    if result.get("error"):
        chat_log.append({"time": ts, "user": "AI", "type": "error", "text": result['error']})
        debug_logs.append(f"[{ts}] [AI] ERROR: {result['error']}")
        return jsonify(result)

    chat_log.append({"time": ts, "user": "AI", "type": "ai", "text": result['explanation']})
    for cmd in result.get("commands", []):
        chat_log.append({"time": ts, "user": "AI", "type": "cmd", "text": json.dumps(cmd)})

    debug_logs.append(f"[{ts}] [AI] {result['explanation']}")

    commands = result.get("commands", [])
    for i, cmd in enumerate(commands):
        debug_logs.append(f"[{ts}] [AI] Command [{i+1}/{len(commands)}]: {json.dumps(cmd)}")

    if commands:
        _execute_ai_command(commands[0])
        for cmd in commands[1:]:
            ai_command_queue.append(cmd)
        if len(commands) > 1:
            debug_logs.append(f"[{ts}] [AI] Queued {len(commands)-1} more command(s)")

    return jsonify(result)


@app.route("/api/ai/status", methods=["GET"])
def ai_status():
    return jsonify({
        "ollama_available": check_ollama(),
        "queue_length": len(ai_command_queue),
        "history": list(ai_log),
    })


@app.route("/api/ai/next", methods=["POST"])
def ai_next_command():
    if ai_command_queue:
        cmd = ai_command_queue.popleft()
        _execute_ai_command(cmd)
        ts = time.strftime("%H:%M:%S")
        debug_logs.append(f"[{ts}] [AI] Next queued command: {json.dumps(cmd)}")
        return jsonify({"ok": True, "cmd": cmd, "remaining": len(ai_command_queue)})
    return jsonify({"ok": True, "cmd": None, "remaining": 0})


def _resolve_relative(cmd):
    fwd = cmd.get("forward", 0)
    right = cmd.get("right", 0)
    heading_rad = math.radians(robot_state.get("heading", 0))
    pos_x = robot_state.get("pos_x", 0)
    pos_y = robot_state.get("pos_y", 0)
    dx = fwd * math.sin(heading_rad) + right * math.cos(heading_rad)
    dy = fwd * math.cos(heading_rad) - right * math.sin(heading_rad)
    return {"action": "set_target", "x": round(pos_x + dx, 1), "y": round(pos_y + dy, 1)}

def _to_pending(cmd):
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
    global pending_command
    action = cmd.get("action", "")
    if action == "move_relative":
        resolved = _resolve_relative(cmd)
        ts = time.strftime("%H:%M:%S")
        debug_logs.append(f"[{ts}] [AI] Relative -> Absolute: heading={robot_state.get('heading',0):.0f} pos=({robot_state.get('pos_x',0):.0f},{robot_state.get('pos_y',0):.0f}) -> target=({resolved['x']},{resolved['y']})")
        cmd = resolved
        action = "set_target"

    if action == "set_target":
        pending_command = {"cmd": "set_target", "x": float(cmd.get("x", 0)), "y": float(cmd.get("y", 0))}
        robot_state["target_wx"] = float(cmd.get("x", 0))
        robot_state["target_wy"] = float(cmd.get("y", 0))
    elif action == "backtrack":
        pending_command = {"cmd": "backtrack"}
    elif action == "stop":
        pending_command = {"cmd": "stop"}
    elif action == "reset":
        pending_command = {"cmd": "reset"}


# ============================================
# Health check
# ============================================
@app.route("/health")
def health():
    return jsonify({"status": "ok", "connected": robot_state["connected"]})


def _lan_ips():
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
    import requests
    for url in ("https://api.ipify.org", "https://ifconfig.me/ip"):
        try:
            r = requests.get(url, timeout=timeout)
            if r.status_code == 200 and r.text.strip():
                return r.text.strip()
        except requests.RequestException:
            continue
    return None


_TUNNEL_INFO_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".tunnel_info")

def _cloudflared_running():
    try:
        out = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe", "/NH"],
            capture_output=True, text=True, timeout=5
        )
        return "cloudflared" in out.stdout.lower()
    except Exception:
        try:
            out = subprocess.run(["pgrep", "-x", "cloudflared"],
                                 capture_output=True, timeout=5)
            return out.returncode == 0
        except Exception:
            return False

def _get_existing_tunnel():
    try:
        with open(_TUNNEL_INFO_FILE, "r") as f:
            data = json.load(f)
        url = data.get("url")
        if url and _cloudflared_running():
            return url
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        pass
    return None

def _save_tunnel_info(pid, url):
    with open(_TUNNEL_INFO_FILE, "w") as f:
        json.dump({"pid": pid, "url": url}, f)

def _start_cloudflare_tunnel(port, timeout=20):
    import platform
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".cloudflared.log")
    try:
        log_file = open(log_path, "w")
        kwargs = {"stdout": subprocess.DEVNULL, "stderr": log_file}
        if platform.system() == "Windows":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        proc = subprocess.Popen(
            ["cloudflared", "tunnel", "--url", f"http://localhost:{port}"],
            **kwargs,
        )
        log_file.close()
    except FileNotFoundError:
        return None

    pattern = re.compile(r'https://[a-z0-9-]+\.trycloudflare\.com')
    url = None
    for _ in range(timeout * 10):
        time.sleep(0.1)
        try:
            with open(log_path, "r") as f:
                m = pattern.search(f.read())
            if m:
                url = m.group(0)
                break
        except Exception:
            pass

    if url is None:
        proc.terminate()
        return None

    _save_tunnel_info(proc.pid, url)
    return url


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 25565))

    cloudflare_url = os.environ.get("CLOUDFLARE_URL")
    if not cloudflare_url:
        cloudflare_url = _get_existing_tunnel()
        if cloudflare_url:
            print("  Reusing existing Cloudflare tunnel.")
        else:
            print("  Starting Cloudflare tunnel (up to 20s)...")
            cloudflare_url = _start_cloudflare_tunnel(port)

    print("=" * 60)
    print("  4WD Robot Server")
    print(f"  Local:      http://localhost:{port}")
    for ip in _lan_ips():
        print(f"  LAN:        http://{ip}:{port}")
    pub = _public_ip()
    if pub:
        print(f"  Public IP:  http://{pub}:{port}  (needs port-forward on your router)")
    else:
        print(f"  Public IP:  <offline or lookup failed>")
    if cloudflare_url:
        print(f"  Cloudflare: {cloudflare_url}")
    else:
        print(f"  Cloudflare: <not available>")
    print("=" * 60)

    app.run(host="0.0.0.0", port=port, threaded=True,
            debug=os.environ.get("FLASK_DEBUG") == "1")
