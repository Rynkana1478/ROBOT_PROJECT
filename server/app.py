"""
ESP32-S3 4WD Robot Server

Manual = immediate hard override (single var, no queue).
Auto   = queued targets/commands with id-based ack.
"""

import os
import json
import time
import math
import threading
from collections import deque
from flask import Flask, render_template, request, jsonify
from ai_translator import translate as ai_translate, check_ollama

app = Flask(__name__)

# --- Chassis tracking ---
chassis_info = {"id": None, "last_seen": 0}

# --- Debug log buffer ---
debug_logs = deque(maxlen=200)

# --- Robot state (latest telemetry from sync) ---
robot_state = {
    "front": 0, "left": 0, "right": 0,
    "near_left": 0, "near_right": 0,
    "heading": 0, "gyro_rate": 0,
    "accel_x": 0, "accel_y": 0,
    "pos_x": 0, "pos_y": 0,
    "distance": 0, "enc_l": 0, "enc_r": 0,
    "battery": 0,
    "mode": "manual", "auto": False,
    "grid_x": 10, "grid_y": 10,
    "target_x": 10, "target_y": 10,
    "target_wx": 0, "target_wy": 0,
    "has_target": False, "target_reached": False,
    "backtracking": False, "crumbs": 0,
    "mpu_ok": False, "enc_healthy": True,
    "grid": "",
    "connected": False, "last_update": 0,
    "wifi_rssi": 0, "free_heap": 0, "uptime": 0,
    "nav_state": "IDLE", "control": "manual",
    "avoid_active": False,
    "batt_low": False, "stuck": False,
    "slipping": False, "stalled": False, "traction": 1.0,
    "path_length": 0,
    "avoid_dx": 0, "avoid_dy": 0, "avoid_dheading": 0,
    "ingested_through": 0, "current_target_id": 0,
    "last_completed_id": 0, "last_completion_status": "NONE",
    "queue_size": 0, "fresh_boot": True,
}

GRID_SIZE = 20

# ----- Master queue (auto-mode only) -----
_state_lock = threading.Lock()
master_queue = deque()        # list of {id, type, x, y, ...}
next_cmd_id = 1               # monotonic id

# ----- Manual hard-override state (single value, overwrites) -----
manual_state = "stop"         # forward / back / left / right / stop / test_*

# ----- Robot mode (sent in every sync response) -----
robot_mode = "manual"         # manual | auto

# ----- One-shot directives (drained on next sync) -----
queue_clear_pending = False
heading_override_pending = None  # float or None

# ----- AI history -----
ai_command_queue = deque()  # legacy AI multi-step (kept for compat)
ai_log = deque(maxlen=50)
chat_log = deque(maxlen=100)

# ----- Request dedupe (prevents double-clicks / accidental re-submits from
#       generating duplicate state changes or duplicate queue entries) -----
_last_control = {"cmd": None, "ts": 0.0}
_last_target  = {"x": None, "y": None, "ts": 0.0}
CONTROL_DEDUPE_MS = 100   # consecutive identical /api/control within this is dropped
TARGET_DEDUPE_MS  = 1000  # consecutive identical /api/target within this is dropped


def _alloc_id():
    global next_cmd_id
    with _state_lock:
        cid = next_cmd_id
        next_cmd_id += 1
        return cid


def _enqueue(cmd_dict):
    """Push an auto-mode command into the master queue with a fresh id."""
    cmd_dict.setdefault("id", _alloc_id())
    with _state_lock:
        master_queue.append(cmd_dict)
    return cmd_dict["id"]


# ============================================
# Dashboard
# ============================================
@app.route("/")
def dashboard():
    port = int(os.environ.get("PORT", 25565))
    return render_template("dashboard.html", grid_size=GRID_SIZE,
                           lan_ips=_lan_ips(), server_port=port)


# ============================================
# Robot <-> Server (single sync endpoint)
# ============================================
@app.route("/api/robot/sync", methods=["POST"])
def robot_sync():
    """ESP32 POSTs telemetry/debug; we respond with mode + manual_state +
       unacked commands + one-shot directives."""
    global queue_clear_pending, heading_override_pending

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

    # --- Telemetry ---
    if "front" in data:
        prev_completed = int(robot_state.get("last_completed_id", 0))
        robot_state.update({k: data.get(k, robot_state.get(k, 0))
                            for k in data if k != "debug"})
        robot_state["connected"] = True
        robot_state["last_update"] = time.time()

        # Prune master queue: drop items with id <= ingested_through
        # (robot has them in its ring buffer now).
        ingested = int(data.get("ingested_through", 0))
        with _state_lock:
            while master_queue and master_queue[0]["id"] <= ingested:
                master_queue.popleft()

        # Log completion edges
        now_completed = int(data.get("last_completed_id", 0))
        if now_completed and now_completed != prev_completed:
            status = data.get("last_completion_status", "?")
            ts = time.strftime("%H:%M:%S")
            debug_logs.append(f"[{ts}] [QUEUE] cmd #{now_completed} -> {status}")

        # If robot just freshly booted, freeze the queue until user explicitly
        # clears it or recalibrates. Robot will send fresh_boot=false next sync.
        if data.get("fresh_boot"):
            ts = time.strftime("%H:%M:%S")
            debug_logs.append(f"[{ts}] [BOOT] robot reports fresh boot")

    # --- Build response ---
    resp = {
        "mode": robot_mode,
        "manual_state": manual_state if robot_mode == "manual" else None,
    }

    # Send only commands the robot hasn't ingested yet
    ingested = int(data.get("ingested_through", 0)) if data else 0
    out_cmds = []
    with _state_lock:
        for c in list(master_queue):
            if c["id"] > ingested:
                out_cmds.append(c)
        # Trim to what fits in robot's ring (8)
        out_cmds = out_cmds[:8]
    if out_cmds:
        resp["cmds"] = out_cmds

    if queue_clear_pending:
        resp["queue_clear"] = True
        queue_clear_pending = False

    if heading_override_pending is not None:
        resp["set_heading"] = float(heading_override_pending)
        heading_override_pending = None

    return jsonify(resp)


# ============================================
# Dashboard endpoint (status + debug + chat merged)
# ============================================
@app.route("/api/dashboard", methods=["GET"])
def get_dashboard():
    state = robot_state.copy()
    if time.time() - state["last_update"] > 3:
        state["connected"] = False

    state["chassis_id"] = chassis_info.get("id")
    state["robot_mode"] = robot_mode
    state["manual_state"] = manual_state
    state["server_queue_size"] = len(master_queue)
    state["server_queue"] = [{"id": c["id"], "type": c["type"]}
                             for c in list(master_queue)[:10]]

    debug_since = int(request.args.get("debug_since", 0))
    all_debug = list(debug_logs)
    debug_slice = all_debug[debug_since:] if debug_since < len(all_debug) else []

    chat_since = int(request.args.get("chat_since", 0))
    all_chat = list(chat_log)
    chat_slice = all_chat[chat_since:] if chat_since < len(all_chat) else []

    return jsonify({
        "status": state,
        "debug": {"logs": debug_slice, "total": len(all_debug)},
        "chat": {"messages": chat_slice, "total": len(all_chat)},
    })


# ============================================
# Manual (immediate hard override — NOT queued)
# ============================================
@app.route("/api/control", methods=["POST"])
def send_control():
    """Manual button press. Sets manual_state directly; if non-stop, also
       flips robot mode to manual (hard override of any auto plan)."""
    global manual_state, robot_mode, queue_clear_pending
    data = request.get_json(silent=True)
    if not data or "cmd" not in data:
        return jsonify({"error": "missing cmd"}), 400
    cmd = data["cmd"]

    # Dedupe identical consecutive control requests within a short window.
    # Catches double-clicks, key-repeat bursts, and rapid network retries.
    now_ms = time.time() * 1000.0
    if cmd == _last_control["cmd"] and (now_ms - _last_control["ts"]) < CONTROL_DEDUPE_MS:
        return jsonify({"ok": True, "deduped": True, "cmd": cmd})
    _last_control["cmd"] = cmd
    _last_control["ts"]  = now_ms

    valid_manual = ("forward", "back", "left", "right", "stop")
    valid_special = ("auto", "reset", "calibrate", "clear_map",
                     "test_all", "test_i2c", "test_ultrasonic", "test_servo",
                     "test_mpu", "test_motors", "test_encoder", "test_battery")

    if cmd in valid_manual:
        manual_state = cmd
        # Any manual button — including stop — is a hard override. Stop in
        # particular must yank an auto plan, so we also flush the queue.
        robot_mode = "manual"
        if cmd == "stop":
            with _state_lock:
                master_queue.clear()
            queue_clear_pending = True
        return jsonify({"ok": True, "manual_state": cmd, "mode": robot_mode})

    if cmd == "auto":
        robot_mode = "auto"
        manual_state = "stop"
        return jsonify({"ok": True, "mode": "auto"})

    if cmd in ("reset", "calibrate", "clear_map"):
        # Sent as a queued one-shot but executed immediately by the robot
        # (the robot recognizes these types and runs them out of order).
        _enqueue({"type": cmd})
        if cmd == "reset" or cmd == "clear_map":
            queue_clear_pending = True
        return jsonify({"ok": True, "queued": cmd})

    if cmd.startswith("test_"):
        # Tests fire as a one-off via manual channel (immediate)
        manual_state = cmd
        robot_mode = "manual"
        return jsonify({"ok": True, "test": cmd})

    if cmd == "backtrack":
        cid = _enqueue({"type": "backtrack"})
        robot_mode = "auto"
        return jsonify({"ok": True, "queued": "backtrack", "id": cid})

    return jsonify({"error": "invalid cmd"}), 400


# ============================================
# Auto target (queued)
# ============================================
@app.route("/api/target", methods=["POST"])
def set_target():
    global robot_mode
    data = request.get_json(silent=True)
    if not data:
        return jsonify({"error": "no json"}), 400
    x = data.get("x"); y = data.get("y")
    if x is None or y is None:
        return jsonify({"error": "missing x or y"}), 400
    x = float(x); y = float(y)

    # Dedupe identical consecutive target requests — prevents the master_queue
    # from getting two identical Go-To-Target enqueues from a double-click.
    now_ms = time.time() * 1000.0
    if (_last_target["x"] == x and _last_target["y"] == y
            and (now_ms - _last_target["ts"]) < TARGET_DEDUPE_MS):
        return jsonify({"ok": True, "deduped": True, "x": x, "y": y})
    _last_target["x"] = x; _last_target["y"] = y; _last_target["ts"] = now_ms

    cid = _enqueue({"type": "set_target", "x": x, "y": y})
    robot_mode = "auto"
    robot_state["target_wx"] = float(x)
    robot_state["target_wy"] = float(y)
    return jsonify({"ok": True, "id": cid, "x": x, "y": y})


# ============================================
# Heading override (north-pin)
# ============================================
@app.route("/api/heading_set", methods=["POST"])
def heading_set():
    """Manual snap of robot heading (e.g. 'I'm facing north now → 0°')."""
    global heading_override_pending
    data = request.get_json(silent=True)
    if not data or "deg" not in data:
        return jsonify({"error": "missing deg"}), 400
    heading_override_pending = float(data["deg"])
    return jsonify({"ok": True, "deg": heading_override_pending})


# ============================================
# Queue management
# ============================================
@app.route("/api/queue/clear", methods=["POST"])
def queue_clear():
    global queue_clear_pending
    with _state_lock:
        master_queue.clear()
    queue_clear_pending = True
    return jsonify({"ok": True})


@app.route("/api/queue", methods=["GET"])
def queue_get():
    with _state_lock:
        items = list(master_queue)
    return jsonify({"queue": items, "size": len(items)})


@app.route("/api/debug/clear", methods=["POST"])
def clear_debug_logs():
    debug_logs.clear()
    return jsonify({"ok": True})


# ============================================
# AI Command Translator (now enqueues into master_queue)
# ============================================
@app.route("/api/ai/translate", methods=["POST"])
def ai_translate_endpoint():
    global robot_mode
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

    if result.get("error"):
        chat_log.append({"time": ts, "user": "AI", "type": "error", "text": result['error']})
        return jsonify(result)

    chat_log.append({"time": ts, "user": "AI", "type": "ai", "text": result['explanation']})

    commands = result.get("commands", [])
    enqueued_ids = []
    for cmd in commands:
        action = cmd.get("action", "")
        if action == "set_target":
            cid = _enqueue({"type": "set_target",
                            "x": float(cmd.get("x", 0)), "y": float(cmd.get("y", 0))})
            enqueued_ids.append(cid)
        elif action == "move_relative":
            cid = _enqueue({"type": "move_relative",
                            "x": float(cmd.get("right", 0)),
                            "y": float(cmd.get("forward", 0))})
            enqueued_ids.append(cid)
        elif action == "backtrack":
            cid = _enqueue({"type": "backtrack"})
            enqueued_ids.append(cid)
        elif action == "stop":
            # Hard stop: switch to manual + clear queue
            global manual_state, queue_clear_pending
            manual_state = "stop"
            robot_mode = "manual"
            with _state_lock:
                master_queue.clear()
            queue_clear_pending = True
        chat_log.append({"time": ts, "user": "AI", "type": "cmd", "text": json.dumps(cmd)})

    if enqueued_ids:
        robot_mode = "auto"
        debug_logs.append(f"[{ts}] [AI] enqueued {len(enqueued_ids)} cmd(s) ids={enqueued_ids}")

    result["enqueued_ids"] = enqueued_ids
    return jsonify(result)


@app.route("/api/ai/status", methods=["GET"])
def ai_status():
    return jsonify({
        "ollama_available": check_ollama(),
        "queue_length": len(master_queue),
        "history": list(ai_log),
    })


# ============================================
# Health + LAN helpers
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


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 25565))
    # DDNS shown in startup banner only — set DDNS_HOST / DDNS_PORT env vars
    # (e.g. via your local start.bat / restart.bat) to display them.
    ddns = os.environ.get("DDNS_HOST", "")
    ddns_port = os.environ.get("DDNS_PORT", "")

    print("=" * 60)
    print("  4WD Robot Server")
    cid = chassis_info.get("id")
    print(f"  Chassis:    {cid if cid else '<waiting for first sync>'}")
    print(f"  Local:      http://localhost:{port}")
    for ip in _lan_ips():
        print(f"  LAN:        http://{ip}:{port}")
    if ddns and ddns_port:
        print(f"  DDNS:       http://{ddns}:{ddns_port}")
    pub = _public_ip()
    if pub:
        print(f"  Public IP:  {pub}")
    print("=" * 60)

    app.run(host="0.0.0.0", port=port,
            debug=os.environ.get("FLASK_DEBUG") == "1", threaded=True)
