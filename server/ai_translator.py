"""
AI Command Translator
Converts natural language → robot commands using Google AI (Gemini/Gemma).
Falls back to rule-based parsing if the API key is missing or the request fails.
"""

import json
import math
import os
import re
import requests
import time

try:
    from config_local import GOOGLE_AI_API_KEY, GOOGLE_AI_MODEL
except ImportError:
    GOOGLE_AI_API_KEY = os.environ.get("GOOGLE_AI_API_KEY", "")
    GOOGLE_AI_MODEL   = os.environ.get("GOOGLE_AI_MODEL", "gemini-2.5-flash")

GOOGLE_AI_BASE = "https://generativelanguage.googleapis.com/v1beta/models"

SYSTEM_PROMPT = """You translate human instructions into JSON robot commands. Output ONLY valid JSON.

Coordinate system: Forward=+Y, Backward=-Y, Left=-X, Right=+X. Units: centimeters. 1 meter = 100cm.

Cardinal directions (full distance on one axis):
- North/Forward: x=0, y=+distance
- South/Backward: x=0, y=-distance
- East/Right: x=+distance, y=0
- West/Left: x=-distance, y=0

Diagonal directions (multiply distance by 0.707 for BOTH x and y):
- Northeast: x=+distance*0.707, y=+distance*0.707
- Northwest: x=-distance*0.707, y=+distance*0.707
- Southeast: x=+distance*0.707, y=-distance*0.707
- Southwest: x=-distance*0.707, y=-distance*0.707

ONLY these actions exist. DO NOT invent new ones:
- "set_target" with "x" and "y" in cm - ABSOLUTE world coordinates (for north/south/east/west)
- "move_relative" with "forward" and "right" in cm - RELATIVE translation (for forward/backward/strafe-left/strafe-right)
- "turn_relative" with "degrees" - PURE ROTATION in place. Negative = turn left (CCW), positive = turn right (CW). 90 = quarter turn, 180 = spin around.
- "backtrack" (no params - return to start)
- "stop" (no params)
- "reset" (no params)

CRITICAL DISTINCTIONS:
- "go north/south/east/west" = ABSOLUTE direction = use "set_target" with x,y
- "go forward/backward" or "go front/ahead/straight/advance" or "reverse/retreat" = use "move_relative" with forward (positive=forward, negative=back), right=0
- "go left/right N cm" = strafe = "move_relative" with forward=0, right=±N (this is a sideways MOVE, not a turn)
- "turn left/right" or "turn N degrees" or "rotate" or "spin around" = "turn_relative" — robot rotates in place WITHOUT moving
- "forward" means in front of the robot, NOT always north
- Synonyms: front=ahead=straight=advance=forward; reverse=retreat=backward

Output format: {"commands": [...], "explanation": "short text"}

Examples:
Input: "go forward 1 meter"
Output: {"commands": [{"action": "move_relative", "forward": 100, "right": 0}], "explanation": "Forward 100cm (relative to robot facing)"}

Input: "go left 50cm"
Output: {"commands": [{"action": "move_relative", "forward": 0, "right": -50}], "explanation": "Left 50cm (relative)"}

Input: "go backward 2m"
Output: {"commands": [{"action": "move_relative", "forward": -200, "right": 0}], "explanation": "Backward 200cm (relative)"}

Input: "go right 1m then forward 2m"
Output: {"commands": [{"action": "move_relative", "forward": 0, "right": 100}, {"action": "move_relative", "forward": 200, "right": 0}], "explanation": "Right 100cm then forward 200cm (relative)"}

Input: "go back 1 meter"
Output: {"commands": [{"action": "move_relative", "forward": -100, "right": 0}], "explanation": "Backward 100cm (relative)"}

Input: "go south 2m"
Output: {"commands": [{"action": "set_target", "x": 0, "y": -200}], "explanation": "South 200cm (absolute)"}

Input: "come back"
Output: {"commands": [{"action": "backtrack"}], "explanation": "Returning to start"}

Input: "return home"
Output: {"commands": [{"action": "backtrack"}], "explanation": "Returning to start"}

IMPORTANT: "go back X meters" means move BACKWARD (set_target with negative Y).
"come back" or "return" or "go home" means BACKTRACK to starting position. These are DIFFERENT.

Input: "go east 2m then north 1m"
Output: {"commands": [{"action": "set_target", "x": 200, "y": 0}, {"action": "set_target", "x": 200, "y": 100}], "explanation": "East 200cm then north 100cm (absolute)"}

Input: "stop"
Output: {"commands": [{"action": "stop"}], "explanation": "Stop"}

Input: "go southeast 2 meters"
Output: {"commands": [{"action": "set_target", "x": 141, "y": -141}], "explanation": "Southeast 200cm diagonal (141,141)"}

Input: "go northwest 3 meters"
Output: {"commands": [{"action": "set_target", "x": -212, "y": 212}], "explanation": "Northwest 300cm diagonal (212,212)"}

Input: "go northeast 1m then come back"
Output: {"commands": [{"action": "set_target", "x": 71, "y": 71}, {"action": "backtrack"}], "explanation": "Northeast 100cm then return"}

Input: "go front 50cm"
Output: {"commands": [{"action": "move_relative", "forward": 50, "right": 0}], "explanation": "Forward 50cm (relative)"}

Input: "turn left"
Output: {"commands": [{"action": "turn_relative", "degrees": -90}], "explanation": "Rotate 90 degrees left in place"}

Input: "turn right"
Output: {"commands": [{"action": "turn_relative", "degrees": 90}], "explanation": "Rotate 90 degrees right in place"}

Input: "turn 45 degrees"
Output: {"commands": [{"action": "turn_relative", "degrees": 45}], "explanation": "Rotate 45 degrees right (positive = CW)"}

Input: "spin around"
Output: {"commands": [{"action": "turn_relative", "degrees": 180}], "explanation": "180 degree spin in place"}

Input: "turn left 45"
Output: {"commands": [{"action": "turn_relative", "degrees": -45}], "explanation": "Rotate 45 degrees left in place"}

Input: "go ahead 1 meter then turn right and go forward 50cm"
Output: {"commands": [{"action": "move_relative", "forward": 100, "right": 0}, {"action": "turn_relative", "degrees": 90}, {"action": "move_relative", "forward": 50, "right": 0}], "explanation": "Forward 1m, turn right 90, then forward 50cm"}
"""


def check_ai():
    """Cheap check — just verify the key is configured. We don't ping Google
    on every dashboard poll because that burns quota and adds latency."""
    return bool(GOOGLE_AI_API_KEY)


# Backwards-compat alias for older imports.
check_ollama = check_ai


def _gemma_payload(user_input):
    """Gemma chat endpoint rejects systemInstruction; fold the system prompt
    into the user turn instead."""
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


def ask_ai(user_input):
    """Send text to Google AI (Gemini or Gemma), get parsed JSON dict."""
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


# Backwards-compat alias.
ask_ollama = ask_ai


def rule_based_parse(text):
    """Fallback: simple regex parser when Ollama is not available."""
    text = text.lower().strip()

    # Priority / emergency commands (no AI needed)
    if text in ("stop", "halt", "freeze", "quick stop", "force stop",
                "emergency stop", "emergency", "quick sto", "stp", "s"):
        return {"commands": [{"action": "stop"}], "explanation": "EMERGENCY STOP", "method": "priority"}
    if text in ("come back", "return", "go home", "backtrack", "go back home",
                "return home", "back home", "home"):
        return {"commands": [{"action": "backtrack"}], "explanation": "Returning to start", "method": "priority"}
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


def translate(user_input):
    """
    Main entry: translate natural language to robot commands.
    Uses Google AI (Gemini/Gemma) if a key is configured, falls back to rule-based.
    Returns dict with: commands, explanation, method, debug info.
    """
    start_time = time.time()
    result = {
        "input": user_input,
        "timestamp": time.strftime("%H:%M:%S"),
        "method": "unknown",
        "ai_available": check_ai(),
        "model": GOOGLE_AI_MODEL if check_ai() else "",
        "commands": [],
        "explanation": "",
        "error": None,
        "duration_ms": 0,
    }
    # Keep the legacy key for dashboards that still read it.
    result["ollama_available"] = result["ai_available"]

    # Check priority commands first (instant, no AI)
    priority = rule_based_parse(user_input)
    if priority.get("method") == "priority":
        result["commands"] = priority["commands"]
        result["explanation"] = priority["explanation"]
        result["method"] = "priority (instant)"
        result["duration_ms"] = round((time.time() - start_time) * 1000)
        return result

    # Try Google AI
    if check_ai():
        result["method"] = f"google-ai ({GOOGLE_AI_MODEL})"

        ai_response = ask_ai(user_input)

        if "error" not in ai_response:
            commands = ai_response.get("commands", [])
            # Validate: only allow known actions
            valid_actions = {"set_target", "move_relative", "turn_relative",
                             "backtrack", "stop", "reset"}
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
                # Post-validate: check if the model did diagonal math wrong.
                # If user said diagonal but only one axis has a value, fix it.
                input_lower = user_input.lower()
                diag_dirs = {
                    "northeast": (1, 1), "northwest": (-1, 1),
                    "southeast": (1, -1), "southwest": (-1, -1)
                }
                for dname, (sx, sy) in diag_dirs.items():
                    if dname in input_lower:
                        for cmd in validated:
                            if cmd.get("action") == "set_target":
                                x, y = cmd.get("x", 0), cmd.get("y", 0)
                                if (x == 0 and y != 0) or (y == 0 and x != 0):
                                    dist = abs(x) if x != 0 else abs(y)
                                    cmd["x"] = round(sx * dist * 0.707)
                                    cmd["y"] = round(sy * dist * 0.707)
                                    result["fixed_diagonal"] = True
                        break

                result["commands"] = validated
                result["explanation"] = ai_response.get("explanation", "")
                result["raw_response"] = ai_response
            else:
                result["method"] = "rule (ai output invalid)"
                fallback = rule_based_parse(user_input)
                result["commands"] = fallback.get("commands", [])
                result["explanation"] = fallback.get("explanation", "")
                result["error"] = fallback.get("error")
        else:
            result["method"] = "rule (ai error)"
            result["ai_error"] = ai_response["error"]
            fallback = rule_based_parse(user_input)
            result["commands"] = fallback.get("commands", [])
            result["explanation"] = fallback.get("explanation", "")
            result["error"] = fallback.get("error")
    else:
        result["method"] = "rule (no api key)"
        fallback = rule_based_parse(user_input)
        result["commands"] = fallback.get("commands", [])
        result["explanation"] = fallback.get("explanation", "")
        result["error"] = fallback.get("error")

    result["duration_ms"] = round((time.time() - start_time) * 1000)
    return result
