"""
AI Command Translator
Converts natural language → robot commands.

Two backends are supported, picked by AI_BACKEND in config_local.py:
  - "ollama" (default): local Ollama instance. No rate limits, runs on the
    same box, supports system prompts properly. Good for contests / heavy use.
  - "google": Gemini / Gemma via Google AI Studio. Generous quality on Flash
    but the free tier caps daily requests.

Falls back to rule-based regex parsing when the chosen backend is unreachable.
"""

import json
import math
import os
import re
import requests
import time

try:
    import config_local as _cfg
except ImportError:
    _cfg = None


def _cfg_get(name, default):
    """Read a field from config_local.py first, then env var, then default.
    Each field is optional — a partially-filled config_local.py still works."""
    if _cfg is not None and hasattr(_cfg, name):
        return getattr(_cfg, name)
    return os.environ.get(name, default)


AI_BACKEND        = _cfg_get("AI_BACKEND",        "ollama")
OLLAMA_URL        = _cfg_get("OLLAMA_URL",        "http://localhost:11434")
OLLAMA_MODEL      = _cfg_get("OLLAMA_MODEL",      "qwen2.5:7b")
GOOGLE_AI_API_KEY = _cfg_get("GOOGLE_AI_API_KEY", "")
GOOGLE_AI_MODEL   = _cfg_get("GOOGLE_AI_MODEL",   "gemini-2.5-flash")

GOOGLE_AI_BASE = "https://generativelanguage.googleapis.com/v1beta/models"

SYSTEM_PROMPT = """You translate English or Thai instructions into JSON robot commands. Output ONLY valid JSON.

Units: cm. World axes: Forward=+Y, Back=-Y, Right=+X, Left=-X. 1m=100cm.
Heading: 0°=north (+Y), 90°=east (+X), 180°=south, 270°=west.

Diagonals — set BOTH x and y (each = distance * 0.707, signs as shown):
- Northeast: x positive, y positive
- Northwest: x negative, y positive
- Southeast: x positive, y negative
- Southwest: x negative, y negative

Actions (use ONLY these):
- set_target {x,y}              ABSOLUTE world coords. For north/south/east/west/diagonals.
- move_relative {forward,right} RELATIVE to robot facing. forward>0=ahead, forward<0=back, right>0=strafe right, right<0=strafe left.
- turn_relative {degrees}       PURE ROTATION in place. degrees<0 = LEFT (CCW). degrees>0 = RIGHT (CW). 180 = spin around. 360 = one full revolution.
- backtrack                     Return to starting point.
- stop                          Emergency stop.
- reset                         Reset position to origin.
- reply                         TEXT-ONLY response. No movement. Use for questions
                                ("where are you?", "is the path clear?", "battery?")
                                or chit-chat. Put the answer in "explanation".

Rules:
- forward / ahead / front / straight / advance = move_relative with forward>0
- backward / back / reverse / retreat = move_relative with forward<0
- "left/right N cm" = strafe = move_relative with right=±N (sideways MOVE, not a turn)
- turn / rotate / spin = turn_relative (robot rotates in place)
- "go back N meters" = move BACKWARD. "come back" / "return" / "go home" = backtrack. They are DIFFERENT.

Context use:
- [ROBOT STATE] block shows the robot's CURRENT sensor data. Use it to answer
  spatial/status questions with a `reply` action.
- [HISTORY] is the recent conversation. Use it to resolve references like
  "now", "another", "again", "same direction", "keep going". Example:
  history says user said "forward 1m", new input "another 50cm" → that means
  "forward 50cm" (same direction).
- If the user's input is a question or statement (not a movement command),
  emit ONE `reply` with the answer in "explanation". Do NOT invent movement.

Thai vocabulary:
- เดินหน้า / ไปข้างหน้า = forward; ถอยหลัง = backward
- เลี้ยวซ้าย = turn left (negative degrees); เลี้ยวขวา = turn right (positive degrees)
- หมุน = spin/rotate; รอบ = full revolution (1 รอบ = 360°)
- กลับบ้าน / กลับจุดเริ่ม = backtrack; หยุด = stop
- เมตร = meter; เซนติเมตร = cm; องศา = degrees
- อยู่ไหน / อยู่ตรงไหน = where are you (use reply)
- ข้างหน้ามีอะไร / โล่งไหม = what's ahead / is it clear (use reply)
- แบตเหลือเท่าไร = battery level (use reply)

Output format: {"commands":[...], "explanation":"short"}

Examples:
"go forward 1 meter" → {"commands":[{"action":"move_relative","forward":100,"right":0}],"explanation":"Forward 100cm"}
"go back 1 meter" → {"commands":[{"action":"move_relative","forward":-100,"right":0}],"explanation":"Backward 100cm"}
"go left 50cm" → {"commands":[{"action":"move_relative","forward":0,"right":-50}],"explanation":"Strafe left 50cm"}
"go south 2m" → {"commands":[{"action":"set_target","x":0,"y":-200}],"explanation":"South 200cm"}
"go northeast 2m" → {"commands":[{"action":"set_target","x":141,"y":141}],"explanation":"Northeast 200cm: x=+141, y=+141"}
"go southwest 1m" → {"commands":[{"action":"set_target","x":-71,"y":-71}],"explanation":"Southwest 100cm: x=-71, y=-71"}
"turn left" → {"commands":[{"action":"turn_relative","degrees":-90}],"explanation":"Left 90"}
"turn right 45" → {"commands":[{"action":"turn_relative","degrees":45}],"explanation":"Right 45"}
"spin around" → {"commands":[{"action":"turn_relative","degrees":180}],"explanation":"180 spin"}
"come back" → {"commands":[{"action":"backtrack"}],"explanation":"Return to start"}
"forward 1m then turn right then forward 50cm" → {"commands":[{"action":"move_relative","forward":100,"right":0},{"action":"turn_relative","degrees":90},{"action":"move_relative","forward":50,"right":0}],"explanation":"Fwd, turn right, fwd"}
"where are you?" (state pos=(50,100), heading=90) → {"commands":[{"action":"reply"}],"explanation":"At (50, 100) facing east."}
"is the path clear?" (state distFront=180) → {"commands":[{"action":"reply"}],"explanation":"Front clear: 180cm ahead."}
"what's the battery?" (state battery=72) → {"commands":[{"action":"reply"}],"explanation":"Battery at 72%."}
"another 50cm" (history: user said "forward 1m") → {"commands":[{"action":"move_relative","forward":50,"right":0}],"explanation":"Forward 50cm more"}
"now turn left" (history: any move) → {"commands":[{"action":"turn_relative","degrees":-90}],"explanation":"Left 90"}
"เดินหน้า 1 เมตร" → {"commands":[{"action":"move_relative","forward":100,"right":0}],"explanation":"Forward 100cm"}
"ถอยหลัง 50 เซนติเมตร" → {"commands":[{"action":"move_relative","forward":-50,"right":0}],"explanation":"Backward 50cm"}
"เลี้ยวขวา 45 องศา" → {"commands":[{"action":"turn_relative","degrees":45}],"explanation":"Right 45"}
"เลี้ยวซ้าย" → {"commands":[{"action":"turn_relative","degrees":-90}],"explanation":"Left 90"}
"หมุน 1 รอบ" → {"commands":[{"action":"turn_relative","degrees":360}],"explanation":"Full 360 spin"}
"กลับบ้าน" → {"commands":[{"action":"backtrack"}],"explanation":"Return to start"}
"หยุด" → {"commands":[{"action":"stop"}],"explanation":"Stop"}
"อยู่ไหน" (state pos=(20,80)) → {"commands":[{"action":"reply"}],"explanation":"อยู่ที่ (20, 80)"}
"""


# ---- Backend selection helpers ------------------------------------------------

def _backend():
    return (AI_BACKEND or "ollama").lower()


def _active_model():
    return OLLAMA_MODEL if _backend() == "ollama" else GOOGLE_AI_MODEL


def check_ai():
    """Is the active backend usable right now?

    Ollama path actually pings /api/tags (cheap, <1s) so we know the daemon
    is up. Google path just verifies a key is configured — we don't ping
    Google on every dashboard poll because it burns quota.
    """
    if _backend() == "ollama":
        try:
            r = requests.get(f"{OLLAMA_URL}/api/tags", timeout=0.5)
            return r.status_code == 200
        except Exception:
            return False
    return bool(GOOGLE_AI_API_KEY)


# Backwards-compat alias for older imports and the dashboard JS.
check_ollama = check_ai


# ---- Ollama backend -----------------------------------------------------------

def _ask_ollama(user_input):
    """Ollama supports a real system prompt and JSON format mode — use both."""
    try:
        r = requests.post(f"{OLLAMA_URL}/api/generate", json={
            "model": OLLAMA_MODEL,
            "prompt": user_input,
            "system": SYSTEM_PROMPT,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1},
        }, timeout=60)  # first call loads model into VRAM (~10-30s); later calls are fast
        if r.status_code != 200:
            return {"error": f"HTTP {r.status_code}: {r.text[:200]}"}
        text = r.json().get("response", "").strip()
        if not text:
            return {"error": "empty model output"}
        return json.loads(text)
    except json.JSONDecodeError as e:
        return {"error": f"bad JSON from model: {e}"}
    except Exception as e:
        return {"error": str(e)}


# ---- Google AI backend --------------------------------------------------------

def _gemma_payload(user_input):
    """Gemma's chat endpoint rejects systemInstruction; fold it into the user turn."""
    return {
        "contents": [{
            "role": "user",
            "parts": [{"text": f"{SYSTEM_PROMPT}\n\nInput: {user_input}\nOutput:"}],
        }],
        "generationConfig": {
            "temperature": 0.1,
            "responseMimeType": "application/json",
        },
    }


def _gemini_payload(user_input):
    return {
        "systemInstruction": {"parts": [{"text": SYSTEM_PROMPT}]},
        "contents": [{"role": "user", "parts": [{"text": user_input}]}],
        "generationConfig": {
            "temperature": 0.1,
            "responseMimeType": "application/json",
        },
    }


def _ask_google(user_input):
    if not GOOGLE_AI_API_KEY:
        return {"error": "no API key configured"}

    url = f"{GOOGLE_AI_BASE}/{GOOGLE_AI_MODEL}:generateContent?key={GOOGLE_AI_API_KEY}"
    body = (_gemma_payload(user_input)
            if GOOGLE_AI_MODEL.lower().startswith("gemma")
            else _gemini_payload(user_input))

    try:
        r = requests.post(url, json=body, timeout=15)
        if r.status_code != 200:
            return {"error": f"HTTP {r.status_code}: {r.text[:200]}"}

        data = r.json()
        candidates = data.get("candidates") or []
        if not candidates:
            return {"error": f"no candidates: {data.get('promptFeedback', {})}"}

        parts = candidates[0].get("content", {}).get("parts") or []
        text = "".join(p.get("text", "") for p in parts).strip()
        if not text:
            return {"error": "empty model output"}

        return json.loads(text)
    except json.JSONDecodeError as e:
        return {"error": f"bad JSON from model: {e}"}
    except Exception as e:
        return {"error": str(e)}


# ---- Context (state + history) preamble --------------------------------------

# Fields lifted from robot_state for the [ROBOT STATE] block. Kept small so we
# don't blow the LLM's context with every JSON field — just the ones the model
# needs to answer spatial/status questions.
_STATE_FIELDS_TEMPLATE = (
    "pos: ({pos_x}, {pos_y}) cm\n"
    "heading: {heading}° (0=north +Y, 90=east +X)\n"
    "distFront: {front} cm  distLeft: {left} cm  distRight: {right} cm\n"
    "nav: {nav_state}  mode: {mode}\n"
    "target: {target_str}\n"
    "battery: {battery}%"
)


def _format_state(state):
    """Render the [ROBOT STATE] block from a robot_state dict. Returns '' if
    no state was supplied (then the prompt skips the block entirely)."""
    if not state:
        return ""
    target_str = "none"
    if state.get("has_target"):
        target_str = f"({int(state.get('target_wx', 0))}, {int(state.get('target_wy', 0))}) cm"
    return _STATE_FIELDS_TEMPLATE.format(
        pos_x=int(state.get("pos_x", 0)),
        pos_y=int(state.get("pos_y", 0)),
        heading=int(state.get("heading", 0)),
        front=int(state.get("front", 0)),
        left=int(state.get("left", 0)),
        right=int(state.get("right", 0)),
        nav_state=state.get("nav_state", "IDLE"),
        mode=state.get("mode", "manual"),
        target_str=target_str,
        battery=int(state.get("battery", 0)),
    )


def _format_history(history):
    """Render the [HISTORY] block from a list of {role, text} turns. Skips
    the block entirely on empty / None."""
    if not history:
        return ""
    lines = []
    for turn in history:
        role = turn.get("role", "user")
        text = (turn.get("text") or "").strip()
        if not text:
            continue
        lines.append(f"{role}: {text}")
    return "\n".join(lines)


def _augment_input(user_input, history, state):
    """Wrap the raw user input in optional [ROBOT STATE] and [HISTORY] blocks
    so the model can resolve references like 'now', 'another', and answer
    spatial questions. If both are empty we return user_input unchanged so
    we don't waste tokens on empty headers."""
    state_block = _format_state(state)
    history_block = _format_history(history)
    if not state_block and not history_block:
        return user_input
    parts = []
    if state_block:
        parts.append("[ROBOT STATE]\n" + state_block)
    if history_block:
        parts.append("[HISTORY]\n" + history_block)
    parts.append("[NEW INPUT]\n" + user_input)
    return "\n\n".join(parts)


# ---- Public entry -------------------------------------------------------------

def ask_ai(user_input):
    """Route to whichever backend is configured."""
    if _backend() == "ollama":
        return _ask_ollama(user_input)
    return _ask_google(user_input)


# Backwards-compat alias.
ask_ollama = ask_ai


# Phrases that mean "go back to origin" on their own. Used by _is_pure_backtrack.
# Listed so we deterministically route them to {action: backtrack} instead of
# letting the LLM split "come back" into [turn_relative 180, move_relative ...]
# (which legitimately acks REACHED mid-trajectory and confuses the user).
_BACKTRACK_PHRASES = (
    "come back", "comeback", "go home", "back home", "go to start",
    "back to start", "go to origin", "back to origin", "return home",
    "return to start", "return to origin", "backtrack", "go back home",
    "head home", "head back",
    # Thai
    "กลับบ้าน", "กลับจุดเริ่ม", "กลับจุดเริ่มต้น", "กลับมา", "กลับฐาน",
)

# Tokens that indicate the message is a COMPOUND command (something else
# alongside the backtrack), so we must fall through to the LLM for proper
# multi-step planning. e.g. "north 200cm and come back" -> [set_target,
# backtrack], NOT a bare backtrack. Without this gate, the substring "come
# back" would short-circuit and the "north 200cm" half would be dropped.
_COMPOUND_TOKENS = (
    "forward", "ahead", "straight", "advance",
    "backward", "reverse", "retreat",
    "left", "right",
    "north", "south", "east", "west",
    "turn", "rotate", "spin",
    " then ", " and ", " after ", "first ", "next ",
    "เดินหน้า", "ไปข้างหน้า", "ถอยหลัง",
    "เลี้ยว", "หมุน", "ซ้าย", "ขวา",
    "เหนือ", "ใต้", "ตะวันออก", "ตะวันตก",
)


def _is_pure_backtrack(text):
    """True only when the message is essentially nothing but a backtrack
    request. Compound sentences ("north 200cm and come back", "go forward
    50cm then home") return False and get handled by the LLM as multi-step
    plans.

    Returns True for:
      "come back", "comeback", "please come back", "go home now"
    Returns False for:
      "north 200cm and comeback", "go 50cm then come back",
      "back" (ambiguous — could be 'back forward 50cm'),
      "no north 200cm and comeback" (compound).
    """
    t = text.strip()

    # Compound detected: another direction/movement word + a backtrack phrase
    # in the same message. The LLM must plan both halves.
    has_backtrack = any(p in t for p in _BACKTRACK_PHRASES)
    if not has_backtrack:
        return False
    if any(tok in t for tok in _COMPOUND_TOKENS):
        return False
    # Digits usually mean a distance/angle — also compound.
    if re.search(r'\d', t):
        return False
    return True


def rule_based_parse(text):
    """Fallback: simple regex parser when the LLM is not available."""
    text = text.lower().strip()

    # Priority / emergency commands (no AI needed).
    if text in ("stop", "halt", "freeze", "quick stop", "force stop",
                "emergency stop", "emergency", "quick sto", "stp", "s"):
        return {"commands": [{"action": "stop"}], "explanation": "EMERGENCY STOP", "method": "priority"}
    if _is_pure_backtrack(text):
        return {"commands": [{"action": "backtrack"}], "explanation": "Returning to (0,0)", "method": "priority"}
    if text in ("reset", "reset position", "restart"):
        return {"commands": [{"action": "reset"}], "explanation": "Position reset", "method": "priority"}

    # Parse direction + distance
    # Matches: "go forward 2 meters", "move left 50cm", "north 100", etc.
    # Relative directions (depend on robot heading) → move_relative
    # NOTE: longer synonyms first so e.g. "advance" wins over a partial match.
    relative_dirs = [
        # forward family
        ("forward", 1, 0), ("ahead", 1, 0), ("straight", 1, 0),
        ("advance", 1, 0), ("front", 1, 0), ("up", 1, 0),
        # backward family
        ("backward", -1, 0), ("reverse", -1, 0), ("retreat", -1, 0),
        ("back", -1, 0), ("down", -1, 0),
        # lateral (note: lateral move is different from a turn — these still strafe)
        ("left", 0, -1),
        ("right", 0, 1),
    ]
    # Absolute directions (world coordinates) → set_target
    absolute_dirs = [
        ("northwest", -0.707, 0.707), ("northeast", 0.707, 0.707),
        ("southwest", -0.707, -0.707), ("southeast", 0.707, -0.707),
        ("north", 0, 1), ("south", 0, -1),
        ("west", -1, 0), ("east", 1, 0),
    ]

    commands = []
    # Split by "then", "and then", ","
    parts = re.split(r'\s+then\s+|\s+and\s+then\s+|,\s*then\s*|,\s*', text)

    for part in parts:
        part = part.strip()
        if not part:
            continue

        matched = False

        # ---- TURN commands (pure rotation, not strafe) ----
        # "turn left", "turn right" → ±90°
        # "turn 45", "rotate 90 deg" → signed degrees as written
        # "turn left 45" / "turn right 90" → direction sets sign
        # "spin around" / "turn around" / "180" → 180°
        if re.search(r'\b(spin\s+around|turn\s+around|turnaround)\b', part):
            commands.append({"action": "turn_relative", "degrees": 180})
            matched = True
        if not matched:
            tm = re.search(
                r'(?:turn|rotate|spin)\s*(left|right)?\s*(-?\d+\.?\d*)?\s*(?:deg|degrees|°)?',
                part)
            if tm and (tm.group(1) or tm.group(2)):
                word = tm.group(1)            # "left", "right", or None
                deg_str = tm.group(2)         # number or None
                if word == "left" and not deg_str:
                    degrees = -90.0
                elif word == "right" and not deg_str:
                    degrees = 90.0
                elif deg_str is not None:
                    degrees = float(deg_str)
                    if word == "left":
                        degrees = -abs(degrees)
                    elif word == "right":
                        degrees = abs(degrees)
                    # else: signed value as written (negative = left, positive = right)
                else:
                    degrees = None
                if degrees is not None:
                    commands.append({"action": "turn_relative",
                                     "degrees": round(degrees, 1)})
                    matched = True

        if matched:
            continue

        # Try relative directions first (forward/backward/left/right)
        for dirname, fwd_mul, right_mul in relative_dirs:
            pattern = rf'(?:go\s+|move\s+)?{dirname}\s+(\d+\.?\d*)\s*(m|meter|meters|cm|centimeter|centimeters)?'
            m = re.search(pattern, part)
            if m:
                dist = float(m.group(1))
                unit = m.group(2) or "cm"
                if unit in ("m", "meter", "meters"):
                    dist *= 100
                commands.append({"action": "move_relative", "forward": round(fwd_mul * dist), "right": round(right_mul * dist)})
                matched = True
                break

        # Try absolute directions (north/south/east/west)
        if not matched:
            for dirname, dx, dy in absolute_dirs:
                pattern = rf'(?:go\s+|move\s+)?{dirname}\s+(\d+\.?\d*)\s*(m|meter|meters|cm|centimeter|centimeters)?'
                m = re.search(pattern, part)
                if m:
                    dist = float(m.group(1))
                    unit = m.group(2) or "cm"
                    if unit in ("m", "meter", "meters"):
                        dist *= 100
                    commands.append({"action": "set_target", "x": round(dx * dist), "y": round(dy * dist)})
                    matched = True
                    break

        # Check for backtrack phrases
        if not matched and re.search(r'come\s+back|return|go\s+home|backtrack|go\s+back\s+home', part):
            commands.append({"action": "backtrack"})
            matched = True

        # Bare direction without distance
        if not matched:
            for dirname, fwd_mul, right_mul in relative_dirs:
                if dirname in part:
                    commands.append({"action": "move_relative", "forward": round(fwd_mul * 100), "right": round(right_mul * 100)})
                    matched = True
                    break
        if not matched:
            for dirname, dx, dy in absolute_dirs:
                if dirname in part:
                    commands.append({"action": "set_target", "x": round(dx * 100), "y": round(dy * 100)})
                    matched = True
                    break

        if not matched and ("home" in part or "return" in part):
            commands.append({"action": "backtrack"})
            matched = True

    if commands:
        explanation = f"Parsed {len(commands)} command(s) from text"
        return {"commands": commands, "explanation": explanation, "method": "rule"}

    return {"error": "Could not understand. Try: 'go forward 2 meters' or 'come back'", "method": "rule"}


def translate(user_input, history=None, state=None):
    """
    Main entry: translate natural language to robot commands.
    Uses the configured backend (Ollama by default); falls back to rule-based.

    Args:
        user_input: raw text from the user.
        history: optional list of {"role": "user"|"ai", "text": "..."} turns,
                 most recent last. Lets the model resolve references like
                 "now", "another", "same direction".
        state:   optional robot_state dict (or any subset). Lets the model
                 answer questions like "where are you?" via a `reply` action.

    Returns dict with: commands, explanation, method, debug info.
    """
    start_time = time.time()
    backend = _backend()
    available = check_ai()
    result = {
        "input": user_input,
        "timestamp": time.strftime("%H:%M:%S"),
        "method": "unknown",
        "ai_available": available,
        "backend": backend,
        "model": _active_model() if available else "",
        "commands": [],
        "explanation": "",
        "error": None,
        "duration_ms": 0,
    }
    # Keep the legacy key for dashboards that still read it.
    result["ollama_available"] = available

    # Check priority commands first (instant, no AI). These bypass history/state
    # by design — "stop" must work even if the AI is wedged or context is junk.
    priority = rule_based_parse(user_input)
    if priority.get("method") == "priority":
        result["commands"] = priority["commands"]
        result["explanation"] = priority["explanation"]
        result["method"] = "priority (instant)"
        result["duration_ms"] = round((time.time() - start_time) * 1000)
        return result

    if available:
        result["method"] = f"{backend} ({_active_model()})"

        # Wrap with state + history so the model can resolve "now", "another",
        # and questions like "where are you?". Falls through to plain text when
        # both are empty.
        augmented = _augment_input(user_input, history, state)
        ai_response = ask_ai(augmented)

        if "error" not in ai_response:
            commands = ai_response.get("commands", [])
            # Validate: only allow known actions. `reply` is text-only — it
            # carries no payload, the answer text lives in `explanation`.
            valid_actions = {"set_target", "move_relative", "turn_relative",
                             "backtrack", "stop", "reset", "reply"}
            validated = []
            for cmd in commands:
                action = cmd.get("action", "")
                if action in valid_actions:
                    validated.append(cmd)
                elif "x" in cmd or "y" in cmd or "distance" in cmd:
                    # LLM used wrong action name but has coordinates — fix it
                    x = cmd.get("x", 0)
                    y = cmd.get("y", 0)
                    validated.append({"action": "set_target", "x": x, "y": y})
                # else: skip unknown action

            if validated:
                # Post-validate: catch model mistakes on diagonals.
                # Two failure modes seen in practice:
                #   (a) only one axis set (e.g., "northeast" → x=200, y=0)
                #   (b) both axes set but a sign is wrong (e.g., "northeast" → x=141, y=-141 == southeast)
                # Fix by stamping the expected magnitude + signs onto the first
                # set_target that follows a diagonal word.
                input_lower = user_input.lower()
                diag_dirs = {
                    "northeast": (1, 1), "northwest": (-1, 1),
                    "southeast": (1, -1), "southwest": (-1, -1),
                }
                for dname, (sx, sy) in diag_dirs.items():
                    if dname in input_lower:
                        for cmd in validated:
                            if cmd.get("action") != "set_target":
                                continue
                            x, y = cmd.get("x", 0), cmd.get("y", 0)
                            single_axis  = (x == 0 and y != 0) or (y == 0 and x != 0)
                            wrong_sign_x = x != 0 and ((x > 0) != (sx > 0))
                            wrong_sign_y = y != 0 and ((y > 0) != (sy > 0))
                            if single_axis:
                                # Model put the full distance on one axis — rescale to diagonal.
                                dist = abs(x) if x != 0 else abs(y)
                                cmd["x"] = round(sx * dist * 0.707)
                                cmd["y"] = round(sy * dist * 0.707)
                                result["fixed_diagonal"] = True
                            elif wrong_sign_x or wrong_sign_y:
                                # Magnitudes are already 0.707-scaled — just flip the bad sign(s).
                                cmd["x"] = abs(x) * (1 if sx > 0 else -1)
                                cmd["y"] = abs(y) * (1 if sy > 0 else -1)
                                result["fixed_diagonal"] = True
                            break  # only fix the first set_target after the diag word
                        break

                result["commands"] = validated
                result["explanation"] = ai_response.get("explanation", "")
                result["raw_response"] = ai_response
            else:
                result["method"] = f"rule ({backend} output invalid)"
                fallback = rule_based_parse(user_input)
                result["commands"] = fallback.get("commands", [])
                result["explanation"] = fallback.get("explanation", "")
                result["error"] = fallback.get("error")
        else:
            result["method"] = f"rule ({backend} error)"
            result["ai_error"] = ai_response["error"]
            fallback = rule_based_parse(user_input)
            result["commands"] = fallback.get("commands", [])
            result["explanation"] = fallback.get("explanation", "")
            result["error"] = fallback.get("error")
    else:
        result["method"] = f"rule ({backend} unavailable)"
        fallback = rule_based_parse(user_input)
        result["commands"] = fallback.get("commands", [])
        result["explanation"] = fallback.get("explanation", "")
        result["error"] = fallback.get("error")

    result["duration_ms"] = round((time.time() - start_time) * 1000)
    return result
