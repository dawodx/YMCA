#!/usr/bin/env python3
"""
Go1 Visual Dashboard - Web-based control with Record & Playback

Features:
- REALTIME: Click buttons to execute immediately
- RECORD: Record a sequence of actions
- PLAY: Play back recorded sequence
- Timing stats for each action
"""

import asyncio
import json
import subprocess
import time
import os
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import webbrowser

SCRIPT_DIR = Path(__file__).parent
SEQUENCES_DIR = SCRIPT_DIR / "sequences"
SAVED_POSES_FILE = SCRIPT_DIR / "saved_poses.json"
CUSTOM_COMMANDS_FILE = SCRIPT_DIR / "custom_commands.json"
DEFAULT_PARAMS_FILE = SCRIPT_DIR / "default_params.json"


def load_default_params():
    """Load saved default parameters from Parameter Editor."""
    if DEFAULT_PARAMS_FILE.exists():
        try:
            with open(DEFAULT_PARAMS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {}


def get_param(cmd_id, param_name, fallback):
    """Get a parameter value, using saved default if available."""
    defaults = load_default_params()
    if cmd_id in defaults and param_name in defaults[cmd_id]:
        return defaults[cmd_id][param_name]
    return fallback


def load_custom_commands():
    """Load custom commands from Parameter Editor."""
    if CUSTOM_COMMANDS_FILE.exists():
        try:
            with open(CUSTOM_COMMANDS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {}

try:
    from go1pylib import Go1, Go1Mode
except ImportError:
    print("ERROR: go1pylib not installed!")
    print("Run: pip install go1pylib")
    exit(1)

# Global state
robot = None
action_log = []
connected = False
recorded_sequence = []
is_recording = False

# Ensure sequences directory exists
SEQUENCES_DIR.mkdir(exist_ok=True)


def load_saved_poses():
    """Load custom poses from saved_poses.json"""
    if SAVED_POSES_FILE.exists():
        try:
            with open(SAVED_POSES_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {}


def get_sequence_files():
    """Get list of saved sequence files."""
    files = []
    if SEQUENCES_DIR.exists():
        for f in SEQUENCES_DIR.glob("*.json"):
            files.append(f.stem)
    return sorted(files)


def save_sequence_file(name, sequence):
    """Save sequence to file."""
    filepath = SEQUENCES_DIR / f"{name}.json"
    with open(filepath, 'w') as f:
        json.dump(sequence, f, indent=2)
    return True


def load_sequence_file(name):
    """Load sequence from file."""
    filepath = SEQUENCES_DIR / f"{name}.json"
    if filepath.exists():
        with open(filepath, 'r') as f:
            return json.load(f)
    return []

# ============== ROBOT CONTROL ==============

GO1_IP = "192.168.12.1"
GO1_USER = "pi"
GO1_PASS = "123"


def unlock_sdk_mode():
    """SSH into Go1 and kill mqttControlNode."""
    try:
        result = subprocess.run("which sshpass", shell=True, capture_output=True)
        if result.returncode != 0:
            return False, "sshpass not installed"

        ssh_cmd = f"sshpass -p '{GO1_PASS}' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 {GO1_USER}@{GO1_IP} 'sudo pkill -f mqttControlNode 2>/dev/null; echo OK'"
        result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True, timeout=10)

        if "OK" in result.stdout:
            return True, "SDK unlocked"
        return False, result.stderr.strip()
    except Exception as e:
        return False, str(e)


def timed_action(name, action_func):
    """Execute action and return timing."""
    start = time.time()
    try:
        action_func()
        elapsed = (time.time() - start) * 1000
        result = {"name": name, "time_ms": round(elapsed, 1), "success": True}
    except Exception as e:
        elapsed = (time.time() - start) * 1000
        result = {"name": name, "time_ms": round(elapsed, 1), "success": False, "error": str(e)}

    action_log.append(result)
    if len(action_log) > 50:
        action_log.pop(0)
    return result


async def timed_async_action(name, action_coro):
    """Execute async action and return timing."""
    start = time.time()
    try:
        await action_coro
        elapsed = (time.time() - start) * 1000
        result = {"name": name, "time_ms": round(elapsed, 1), "success": True}
    except Exception as e:
        elapsed = (time.time() - start) * 1000
        result = {"name": name, "time_ms": round(elapsed, 1), "success": False, "error": str(e)}

    action_log.append(result)
    if len(action_log) > 50:
        action_log.pop(0)
    return result


# ============== COMMAND DEFINITIONS ==============

COMMANDS = {
    # Modes
    "stand": {"name": "Stand", "category": "mode", "color": "#4CAF50", "duration": 500},
    "standUp": {"name": "Stand Up", "category": "mode", "color": "#4CAF50", "duration": 1000},
    "standDown": {"name": "Sit Down", "category": "mode", "color": "#4CAF50", "duration": 1000},
    "walk": {"name": "Walk Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "run": {"name": "Run Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "climb": {"name": "Climb Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "damping": {"name": "Damping", "category": "mode", "color": "#FF9800", "duration": 500},
    "recoverStand": {"name": "Recovery", "category": "mode", "color": "#FF9800", "duration": 2000},

    # Dances (only 1 & 2 work on Go1 Pro)
    "dance1": {"name": "Dance 1", "category": "dance", "color": "#E91E63", "duration": 5000},
    "dance2": {"name": "Dance 2", "category": "dance", "color": "#E91E63", "duration": 5000},

    # Special moves (only these work on Go1 Pro)
    "jumpYaw": {"name": "Jump Yaw", "category": "special", "color": "#FF5722", "duration": 2000},
    "straightHand1": {"name": "Straight Hand", "category": "special", "color": "#FF5722", "duration": 2000},

    # Movement
    "forward": {"name": "Forward", "category": "move", "color": "#00BCD4", "duration": 200},
    "backward": {"name": "Backward", "category": "move", "color": "#00BCD4", "duration": 200},
    "left": {"name": "Strafe L", "category": "move", "color": "#00BCD4", "duration": 200},
    "right": {"name": "Strafe R", "category": "move", "color": "#00BCD4", "duration": 200},
    "turnLeft": {"name": "Turn L", "category": "move", "color": "#00BCD4", "duration": 200},
    "turnRight": {"name": "Turn R", "category": "move", "color": "#00BCD4", "duration": 200},

    # Body pose
    "lookUp": {"name": "Look Up", "category": "pose", "color": "#607D8B", "duration": 400},
    "lookDown": {"name": "Look Down", "category": "pose", "color": "#607D8B", "duration": 400},
    "leanLeft": {"name": "Lean L", "category": "pose", "color": "#607D8B", "duration": 400},
    "leanRight": {"name": "Lean R", "category": "pose", "color": "#607D8B", "duration": 400},
    "twistLeft": {"name": "Twist L", "category": "pose", "color": "#607D8B", "duration": 400},
    "twistRight": {"name": "Twist R", "category": "pose", "color": "#607D8B", "duration": 400},
    "squat": {"name": "Squat", "category": "pose", "color": "#607D8B", "duration": 400},
    "extend": {"name": "Extend", "category": "pose", "color": "#607D8B", "duration": 400},

    # Delays for choreography
    "wait500": {"name": "Wait 0.5s", "category": "wait", "color": "#9E9E9E", "duration": 500},
    "wait1000": {"name": "Wait 1s", "category": "wait", "color": "#9E9E9E", "duration": 1000},
    "wait2000": {"name": "Wait 2s", "category": "wait", "color": "#9E9E9E", "duration": 2000},

    # LED colors
    "ledRed": {"name": "LED Red", "category": "led", "color": "#f44336", "duration": 100, "rgb": [255, 0, 0]},
    "ledGreen": {"name": "LED Green", "category": "led", "color": "#4CAF50", "duration": 100, "rgb": [0, 255, 0]},
    "ledBlue": {"name": "LED Blue", "category": "led", "color": "#2196F3", "duration": 100, "rgb": [0, 0, 255]},
    "ledYellow": {"name": "LED Yellow", "category": "led", "color": "#ffd700", "duration": 100, "rgb": [255, 215, 0]},
    "ledPink": {"name": "LED Pink", "category": "led", "color": "#E91E63", "duration": 100, "rgb": [255, 20, 147]},
    "ledCyan": {"name": "LED Cyan", "category": "led", "color": "#00BCD4", "duration": 100, "rgb": [0, 188, 212]},
    "ledOff": {"name": "LED Off", "category": "led", "color": "#333333", "duration": 100, "rgb": [0, 0, 0]},

    # YMCA Dance Poses (from dance_mujoco.py)
    "ymcaY": {"name": "Y Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaM": {"name": "M Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaC": {"name": "C Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaA": {"name": "A Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaMarch": {"name": "March", "category": "ymca", "color": "#FF9800", "duration": 2000},
    "ymcaDance": {"name": "YMCA Dance!", "category": "ymca", "color": "#E91E63", "duration": 30000},
}


def execute_command(cmd):
    """Execute a command and return timing."""
    global robot

    if cmd.startswith("wait"):
        # Just a delay, no robot action
        delay = int(cmd.replace("wait", ""))
        time.sleep(delay / 1000)
        return {"name": cmd, "time_ms": delay, "success": True}

    if not robot or not connected:
        return {"name": cmd, "time_ms": 0, "success": False, "error": "Not connected"}

    # Mode commands (via go1pylib)
    mode_map = {
        "stand": Go1Mode.STAND,
        "standUp": Go1Mode.STAND_UP,
        "standDown": Go1Mode.STAND_DOWN,
        "walk": Go1Mode.WALK,
        "run": Go1Mode.RUN,
        "climb": Go1Mode.CLIMB,
        "damping": Go1Mode.DAMPING,
        "recoverStand": Go1Mode.RECOVER_STAND,
        "dance1": Go1Mode.DANCE1,
        "dance2": Go1Mode.DANCE2,
        "straightHand1": Go1Mode.STRAIGHT_HAND1,
    }

    if cmd in mode_map:
        return timed_action(cmd, lambda: robot.set_mode(mode_map[cmd]))

    # Raw MQTT commands (jumpYaw uses MQTT)
    if cmd == "jumpYaw":
        return timed_action(cmd, lambda: robot.mqtt.client.publish("controller/action", cmd, qos=1))

    # LED commands
    if cmd.startswith("led"):
        led_info = COMMANDS.get(cmd)
        if led_info and "rgb" in led_info:
            r, g, b = led_info["rgb"]
            return timed_action(cmd, lambda: robot.set_led_color(r, g, b))

    # YMCA Dance commands
    if cmd == "ymcaY":
        # Y Pose: Front legs up (using straight hand move)
        return timed_action(cmd, lambda: robot.set_mode(Go1Mode.STRAIGHT_HAND1))
    elif cmd == "ymcaM":
        # M Pose: Deep crouch
        return timed_action(cmd, lambda: robot.set_mode(Go1Mode.STAND_DOWN))
    elif cmd == "ymcaC":
        # C Pose: Lean right (body pose)
        intensity = get_param("lean_right", "intensity", 0.6)
        duration = int(get_param("lean_right", "duration", 300))
        def do_c_pose():
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.2)
            loop = asyncio.new_event_loop()
            loop.run_until_complete(robot.lean_right(intensity, duration))
            loop.close()
        return timed_action(cmd, do_c_pose)
    elif cmd == "ymcaA":
        # A Pose: Tall stance
        intensity = get_param("extend_up", "intensity", 0.5)
        duration = int(get_param("extend_up", "duration", 300))
        def do_a_pose():
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.2)
            loop = asyncio.new_event_loop()
            loop.run_until_complete(robot.extend_up(intensity, duration))
            loop.close()
        return timed_action(cmd, do_a_pose)
    elif cmd == "ymcaMarch":
        # March in place using dance1
        return timed_action(cmd, lambda: robot.set_mode(Go1Mode.DANCE1))
    elif cmd == "ymcaDance":
        # Full YMCA dance sequence
        lean_intensity = get_param("lean_right", "intensity", 0.6)
        lean_duration = int(get_param("lean_right", "duration", 300))
        def do_ymca_dance():
            poses = [
                (Go1Mode.DANCE1, 2.0),       # Intro march
                (Go1Mode.STRAIGHT_HAND1, 1.5),  # Y
                (Go1Mode.STAND_DOWN, 1.5),      # M
                ("leanRight", 1.5),              # C
                (Go1Mode.STAND_UP, 1.5),        # A
                (Go1Mode.STRAIGHT_HAND1, 1.5),  # Y
                (Go1Mode.STAND_DOWN, 1.5),      # M
                ("leanRight", 1.5),              # C
                (Go1Mode.STAND_UP, 1.5),        # A
                (Go1Mode.DANCE2, 3.0),       # Outro
            ]
            for pose, duration in poses:
                if isinstance(pose, str):
                    # Body pose command
                    robot.set_mode(Go1Mode.STAND)
                    time.sleep(0.2)
                    loop = asyncio.new_event_loop()
                    loop.run_until_complete(robot.lean_right(lean_intensity, lean_duration))
                    loop.close()
                else:
                    robot.set_mode(pose)
                time.sleep(duration)
        return timed_action(cmd, do_ymca_dance)

    return {"name": cmd, "time_ms": 0, "success": False, "error": "Unknown command"}


async def execute_movement(cmd):
    """Execute movement command."""
    global robot

    if not robot or not connected:
        return {"name": cmd, "time_ms": 0, "success": False, "error": "Not connected"}

    robot.set_mode(Go1Mode.WALK)

    if cmd == "forward":
        speed = get_param("go_forward", "speed", 0.5)
        duration = int(get_param("go_forward", "duration", 200))
        return await timed_async_action(cmd, robot.go_forward(speed, duration))
    elif cmd == "backward":
        speed = get_param("go_backward", "speed", 0.5)
        duration = int(get_param("go_backward", "duration", 200))
        return await timed_async_action(cmd, robot.go_backward(speed, duration))
    elif cmd == "left":
        speed = get_param("go_left", "speed", 0.4)
        duration = int(get_param("go_left", "duration", 200))
        return await timed_async_action(cmd, robot.go_left(speed, duration))
    elif cmd == "right":
        speed = get_param("go_right", "speed", 0.4)
        duration = int(get_param("go_right", "duration", 200))
        return await timed_async_action(cmd, robot.go_right(speed, duration))
    elif cmd == "turnLeft":
        speed = get_param("turn_left", "speed", 0.6)
        duration = int(get_param("turn_left", "duration", 200))
        return await timed_async_action(cmd, robot.turn_left(speed, duration))
    elif cmd == "turnRight":
        speed = get_param("turn_right", "speed", 0.6)
        duration = int(get_param("turn_right", "duration", 200))
        return await timed_async_action(cmd, robot.turn_right(speed, duration))

    return {"name": cmd, "time_ms": 0, "success": False, "error": "Unknown movement"}


async def execute_pose(cmd):
    """Execute pose command."""
    global robot

    if not robot or not connected:
        return {"name": cmd, "time_ms": 0, "success": False, "error": "Not connected"}

    robot.set_mode(Go1Mode.STAND)
    await asyncio.sleep(0.1)

    if cmd == "lookUp":
        intensity = get_param("look_up", "intensity", 0.5)
        duration = int(get_param("look_up", "duration", 300))
        return await timed_async_action(cmd, robot.look_up(intensity, duration))
    elif cmd == "lookDown":
        intensity = get_param("look_down", "intensity", 0.5)
        duration = int(get_param("look_down", "duration", 300))
        return await timed_async_action(cmd, robot.look_down(intensity, duration))
    elif cmd == "leanLeft":
        intensity = get_param("lean_left", "intensity", 0.5)
        duration = int(get_param("lean_left", "duration", 300))
        return await timed_async_action(cmd, robot.lean_left(intensity, duration))
    elif cmd == "leanRight":
        intensity = get_param("lean_right", "intensity", 0.5)
        duration = int(get_param("lean_right", "duration", 300))
        return await timed_async_action(cmd, robot.lean_right(intensity, duration))
    elif cmd == "twistLeft":
        intensity = get_param("twist_left", "intensity", 0.5)
        duration = int(get_param("twist_left", "duration", 300))
        return await timed_async_action(cmd, robot.twist_left(intensity, duration))
    elif cmd == "twistRight":
        intensity = get_param("twist_right", "intensity", 0.5)
        duration = int(get_param("twist_right", "duration", 300))
        return await timed_async_action(cmd, robot.twist_right(intensity, duration))
    elif cmd == "squat":
        intensity = get_param("squat_down", "intensity", 0.5)
        duration = int(get_param("squat_down", "duration", 300))
        return await timed_async_action(cmd, robot.squat_down(intensity, duration))
    elif cmd == "extend":
        intensity = get_param("extend_up", "intensity", 0.5)
        duration = int(get_param("extend_up", "duration", 300))
        return await timed_async_action(cmd, robot.extend_up(intensity, duration))

    return {"name": cmd, "time_ms": 0, "success": False, "error": "Unknown pose"}


def run_command(cmd):
    """Run any command (sync wrapper)."""
    category = COMMANDS.get(cmd, {}).get('category', '')

    if category in ['move']:
        loop = asyncio.new_event_loop()
        result = loop.run_until_complete(execute_movement(cmd))
        loop.close()
    elif category in ['pose']:
        loop = asyncio.new_event_loop()
        result = loop.run_until_complete(execute_pose(cmd))
        loop.close()
    else:
        result = execute_command(cmd)

    return result


# ============== WEB SERVER ==============

HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Go1 Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: white;
            padding: 15px;
        }
        .header {
            text-align: center;
            margin-bottom: 15px;
        }
        .header h1 { font-size: 24px; color: #ffd700; }
        .status-bar {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin: 10px 0;
            flex-wrap: wrap;
        }
        .status {
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
        }
        .status.connected { background: #4CAF50; }
        .status.disconnected { background: #f44336; }
        .status.recording { background: #f44336; animation: pulse 1s infinite; }
        .status.playing { background: #2196F3; animation: pulse 1s infinite; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .mode-selector {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin: 15px 0;
        }
        .mode-btn {
            padding: 12px 25px;
            border: 3px solid transparent;
            border-radius: 25px;
            font-size: 14px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s;
        }
        .mode-btn.realtime { background: #4CAF50; }
        .mode-btn.record { background: #f44336; }
        .mode-btn.teach { background: #00BCD4; }
        .mode-btn.play { background: #2196F3; }
        .mode-btn.active { border-color: #ffd700; transform: scale(1.1); }
        .teach-banner {
            display: none;
            background: linear-gradient(90deg, #00BCD4, #009688);
            color: white;
            text-align: center;
            padding: 12px;
            margin: 10px 0;
            border-radius: 8px;
            font-weight: bold;
        }
        .teach-banner.active { display: block; animation: pulse 1.5s infinite; }
        .mode-btn:hover { transform: scale(1.05); }

        .container {
            max-width: 1400px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 280px;
            gap: 15px;
        }
        @media (max-width: 900px) {
            .container { grid-template-columns: 1fr; }
        }

        .panel {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 12px;
            border: 1px solid rgba(255,255,255,0.1);
            margin-bottom: 10px;
        }
        .panel h2 {
            font-size: 12px;
            color: #ffd700;
            margin-bottom: 10px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .btn-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(85px, 1fr));
            gap: 6px;
        }
        .btn {
            padding: 10px 6px;
            border: none;
            border-radius: 8px;
            font-size: 10px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            color: white;
            text-transform: uppercase;
        }
        .btn:hover { transform: scale(1.05); box-shadow: 0 5px 15px rgba(0,0,0,0.3); }
        .btn:active { transform: scale(0.95); }
        .btn.danger { animation: glow 2s infinite; }
        @keyframes glow {
            0%, 100% { box-shadow: 0 0 5px rgba(244,67,54,0.5); }
            50% { box-shadow: 0 0 20px rgba(244,67,54,0.8); }
        }

        .sequence-panel {
            background: rgba(0,0,0,0.3);
            border-radius: 10px;
            padding: 10px;
            min-height: 150px;
            max-height: 300px;
            overflow-y: auto;
        }
        .sequence-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 6px 10px;
            margin: 4px 0;
            background: rgba(255,255,255,0.1);
            border-radius: 6px;
            font-size: 11px;
        }
        .sequence-item .remove {
            cursor: pointer;
            color: #f44336;
            font-weight: bold;
        }
        .sequence-controls {
            display: flex;
            gap: 8px;
            margin-top: 10px;
        }
        .seq-btn {
            flex: 1;
            padding: 10px;
            border: none;
            border-radius: 8px;
            font-weight: bold;
            cursor: pointer;
            font-size: 11px;
        }

        .log {
            max-height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 10px;
        }
        .log-entry {
            padding: 6px 8px;
            border-bottom: 1px solid rgba(255,255,255,0.05);
            display: flex;
            justify-content: space-between;
        }
        .log-entry.success { border-left: 3px solid #4CAF50; }
        .log-entry.error { border-left: 3px solid #f44336; }
        .log-time { color: #ffd700; font-weight: bold; }

        .led-picker {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }
        .led-btn {
            width: 35px;
            height: 35px;
            border-radius: 50%;
            border: 2px solid rgba(255,255,255,0.3);
            cursor: pointer;
        }
        .led-btn:hover { border-color: white; transform: scale(1.1); }

        .connect-row {
            display: flex;
            gap: 8px;
        }
        .connect-row .btn { flex: 1; padding: 12px; font-size: 12px; }

        .stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
            margin-bottom: 10px;
        }
        .stat {
            background: rgba(0,0,0,0.3);
            padding: 8px;
            border-radius: 8px;
            text-align: center;
        }
        .stat-value { font-size: 20px; font-weight: bold; color: #ffd700; }
        .stat-label { font-size: 9px; color: rgba(255,255,255,0.5); text-transform: uppercase; }

        .now-playing {
            text-align: center;
            padding: 15px;
            background: rgba(33,150,243,0.2);
            border-radius: 10px;
            margin-bottom: 10px;
            display: none;
        }
        .now-playing.active { display: block; }
        .now-playing .cmd { font-size: 18px; font-weight: bold; color: #2196F3; }
        .now-playing .progress { font-size: 12px; color: rgba(255,255,255,0.7); }
    </style>
</head>
<body>
    <div class="header">
        <h1>Go1 Control Dashboard</h1>
        <div class="status-bar">
            <span class="status" id="targetStatus" style="background:#2196F3;cursor:pointer;" onclick="toggleTarget()">REAL ROBOT</span>
            <span class="status disconnected" id="connStatus">Disconnected</span>
            <span class="status" id="modeStatus" style="background:#4CAF50;">REALTIME</span>
        </div>
    </div>

    <div class="mode-selector">
        <button class="mode-btn realtime active" onclick="setMode('realtime')">REALTIME</button>
        <button class="mode-btn record" onclick="setMode('record')">REC</button>
        <button class="mode-btn play" onclick="playSequence()">PLAY</button>
        <button class="mode-btn stop" onclick="stopPlayback()" style="background:#f44336;">STOP</button>
    </div>

    <div class="now-playing" id="nowPlaying">
        <div class="cmd" id="currentCmd">-</div>
        <div class="progress" id="playProgress">0 / 0</div>
    </div>


    <div class="container">
        <div class="main-panel">
            <div class="panel">
                <div class="connect-row">
                    <button class="btn" style="background:#4CAF50;" onclick="connect()">Connect</button>
                    <button class="btn" style="background:#FF9800;" onclick="unlock()">Unlock SDK</button>
                    <button class="btn" style="background:#9C27B0;" onclick="startSimulation()">Start Simulation</button>
                    <button class="btn" style="background:#607D8B;" onclick="resetSimulation()">Reset Sim</button>
                    <button class="btn" style="background:#00BCD4;" onclick="openPoseBuilder()">Pose Builder</button>
                    <button class="btn" style="background:#FF9800;" onclick="openParamEditor()">Param Editor</button>
                    <button class="btn" style="background:#E91E63;" onclick="openChoreographer()">Choreographer</button>
                </div>
            </div>

            <div class="panel">
                <h2>Modes</h2>
                <div class="btn-grid" id="mode-btns"></div>
            </div>

            <div class="panel">
                <h2>Dance & Special</h2>
                <div class="btn-grid" id="dance-btns"></div>
            </div>

            <div class="panel">
                <h2>Movement</h2>
                <div class="btn-grid" id="move-btns"></div>
            </div>

            <div class="panel">
                <h2>Body Pose</h2>
                <div class="btn-grid" id="pose-btns"></div>
            </div>

            <div class="panel">
                <h2>Timing / Waits</h2>
                <div class="btn-grid" id="wait-btns"></div>
            </div>

            <div class="panel">
                <h2>LED Colors</h2>
                <div class="btn-grid" id="led-btns"></div>
            </div>

            <div class="panel">
                <h2>YMCA Dance</h2>
                <div class="btn-grid" id="ymca-btns"></div>
            </div>

            <div class="panel">
                <h2>Custom Poses (from Pose Builder)</h2>
                <div class="btn-grid" id="custom-pose-btns"></div>
            </div>

            <div class="panel">
                <h2>Custom Commands (from Param Editor)</h2>
                <div class="btn-grid" id="custom-cmd-btns"></div>
            </div>
        </div>

        <div class="side-panel">
            <div class="panel">
                <h2>Stats</h2>
                <div class="stats">
                    <div class="stat">
                        <div class="stat-value" id="avgTime">0</div>
                        <div class="stat-label">Avg ms</div>
                    </div>
                    <div class="stat">
                        <div class="stat-value" id="totalCmds">0</div>
                        <div class="stat-label">Commands</div>
                    </div>
                </div>
            </div>

            <div class="panel">
                <h2>Recorded Sequence (<span id="seqCount">0</span>)</h2>
                <div class="sequence-panel" id="sequence"></div>
                <div class="sequence-controls">
                    <button class="seq-btn" style="background:#f44336;" onclick="clearSequence()">Clear</button>
                </div>
                <div style="margin-top:10px;">
                    <input type="text" id="seqFileName" placeholder="Sequence name..." style="width:100%;padding:8px;border-radius:6px;border:1px solid #444;background:#222;color:white;margin-bottom:8px;">
                    <div class="sequence-controls">
                        <button class="seq-btn" style="background:#9C27B0;" onclick="saveSequenceToFile()">Save to File</button>
                    </div>
                </div>
                <div style="margin-top:10px;">
                    <select id="seqFileSelect" style="width:100%;padding:8px;border-radius:6px;border:1px solid #444;background:#222;color:white;margin-bottom:8px;">
                        <option value="">-- Select sequence --</option>
                    </select>
                    <div class="sequence-controls">
                        <button class="seq-btn" style="background:#607D8B;" onclick="loadSequenceFromFile()">Load from File</button>
                        <button class="seq-btn" style="background:#2196F3;" onclick="refreshSequenceList()">Refresh</button>
                    </div>
                </div>
            </div>

            <div class="panel">
                <h2>Action Log</h2>
                <div class="log" id="log"></div>
            </div>
        </div>
    </div>

    <script>
        const COMMANDS = COMMANDS_JSON;
        let currentMode = 'realtime';
        let sequence = [];
        let isPlaying = false;
        let apiBase = '';  // Empty = same server (real robot dashboard)
        let isSimMode = false;

        function toggleTarget() {
            isSimMode = !isSimMode;
            const el = document.getElementById('targetStatus');
            if (isSimMode) {
                apiBase = 'http://localhost:8891';
                el.textContent = 'SIMULATION';
                el.style.background = '#9C27B0';
            } else {
                apiBase = '';
                el.textContent = 'REAL ROBOT';
                el.style.background = '#2196F3';
            }
            // Update connection status
            document.getElementById('connStatus').textContent = 'Disconnected';
            document.getElementById('connStatus').className = 'status disconnected';
        }

        function createButtons() {
            const categories = {
                'mode': 'mode-btns',
                'dance': 'dance-btns',
                'special': 'dance-btns',
                'move': 'move-btns',
                'pose': 'pose-btns',
                'wait': 'wait-btns',
                'led': 'led-btns',
                'ymca': 'ymca-btns'
            };

            for (const [cmd, info] of Object.entries(COMMANDS)) {
                const container = document.getElementById(categories[info.category]);
                if (!container) continue;

                const btn = document.createElement('button');
                btn.className = 'btn' + (info.category === 'danger' ? ' danger' : '');
                btn.style.background = info.color;
                btn.textContent = info.name;
                btn.onclick = () => handleClick(cmd);
                container.appendChild(btn);
            }
        }

        function setMode(mode) {
            currentMode = mode;
            document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
            const modeBtn = document.querySelector('.mode-btn.' + mode);
            if (modeBtn) modeBtn.classList.add('active');

            const statusEl = document.getElementById('modeStatus');
            const teachBanner = document.getElementById('teachBanner');
            teachBanner.classList.remove('active');
            statusEl.classList.remove('recording');

            if (mode === 'realtime') {
                statusEl.textContent = 'REALTIME';
                statusEl.style.background = '#4CAF50';
            } else if (mode === 'record') {
                statusEl.textContent = 'RECORDING';
                statusEl.style.background = '#f44336';
                statusEl.classList.add('recording');
            } else if (mode === 'teach') {
                statusEl.textContent = 'TEACH MODE';
                statusEl.style.background = '#00BCD4';
                teachBanner.classList.add('active');
            }
        }

        async function startTeachMode() {
            // Put robot in damping mode (soft/compliant joints)
            await sendCmd('damping');
            addLog('Teach Mode: Robot is soft', true, 0);

            // Switch to teach mode (which records like record mode)
            setMode('teach');
            currentMode = 'record';  // Buttons will record to sequence

            document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
            document.querySelector('.mode-btn.teach').classList.add('active');
        }

        function exitTeachMode() {
            // Stand the robot back up
            sendCmd('recoverStand');
            setMode('realtime');
        }

        function handleClick(cmd) {
            if (currentMode === 'realtime') {
                sendCmd(cmd);
            } else if (currentMode === 'record') {
                addToSequence(cmd);
            }
        }

        function addToSequence(cmd) {
            const info = COMMANDS[cmd];
            sequence.push({ cmd, name: info.name, duration: info.duration });
            renderSequence();
        }

        function removeFromSequence(index) {
            sequence.splice(index, 1);
            renderSequence();
        }

        function renderSequence() {
            const container = document.getElementById('sequence');
            container.innerHTML = '';
            sequence.forEach((item, i) => {
                const div = document.createElement('div');
                div.className = 'sequence-item';
                div.innerHTML = `
                    <span>${i + 1}. ${item.name} (${item.duration}ms)</span>
                    <span class="remove" onclick="removeFromSequence(${i})">✕</span>
                `;
                container.appendChild(div);
            });
            document.getElementById('seqCount').textContent = sequence.length;
        }

        function clearSequence() {
            sequence = [];
            renderSequence();
        }

        async function saveSequenceToFile() {
            const name = document.getElementById('seqFileName').value.trim();
            if (!name) {
                alert('Enter a sequence name!');
                return;
            }
            if (sequence.length === 0) {
                alert('No actions to save!');
                return;
            }
            try {
                const res = await fetch('/api/sequence/save', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({name: name, sequence: sequence})
                });
                const data = await res.json();
                if (data.success) {
                    addLog('Saved: ' + name, true, 0);
                    document.getElementById('seqFileName').value = '';
                    refreshSequenceList();
                } else {
                    alert('Save failed!');
                }
            } catch(e) {
                alert('Save error: ' + e);
            }
        }

        async function loadSequenceFromFile() {
            const name = document.getElementById('seqFileSelect').value;
            if (!name) {
                alert('Select a sequence file!');
                return;
            }
            try {
                const res = await fetch('/api/sequence/load/' + encodeURIComponent(name));
                const data = await res.json();
                if (data.sequence) {
                    sequence = data.sequence;
                    renderSequence();
                    addLog('Loaded: ' + name, true, 0);
                } else {
                    alert('Load failed!');
                }
            } catch(e) {
                alert('Load error: ' + e);
            }
        }

        async function refreshSequenceList() {
            try {
                const res = await fetch('/api/sequence/list');
                const data = await res.json();
                const select = document.getElementById('seqFileSelect');
                select.innerHTML = '<option value="">-- Select sequence --</option>';
                for (const name of data.files || []) {
                    const opt = document.createElement('option');
                    opt.value = name;
                    opt.textContent = name;
                    select.appendChild(opt);
                }
            } catch(e) {
                console.error('Failed to load sequence list');
            }
        }

        async function loadCustomPoses() {
            try {
                const res = await fetch('/api/custom_poses');
                const poses = await res.json();
                const container = document.getElementById('custom-pose-btns');
                container.innerHTML = '';
                for (const [name, pose] of Object.entries(poses)) {
                    const btn = document.createElement('button');
                    btn.className = 'btn';
                    btn.style.background = '#00BCD4';
                    btn.textContent = name;
                    btn.onclick = () => sendCustomPose(name, pose);
                    container.appendChild(btn);
                }
            } catch(e) {
                console.error('Failed to load custom poses');
            }
        }

        async function sendCustomPose(name, pose) {
            if (currentMode === 'record') {
                // Add as custom pose to sequence
                sequence.push({ cmd: 'customPose:' + name, name: name, duration: 800, pose: pose });
                renderSequence();
            } else {
                // Execute immediately
                try {
                    await fetch('/api/custom_pose', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({pose: pose})
                    });
                    addLog(name, true, 0);
                } catch(e) {
                    addLog(name + ' failed', false, 0);
                }
            }
        }

        async function playSequence() {
            if (sequence.length === 0) {
                alert('No actions recorded!');
                return;
            }
            if (isPlaying) return;

            isPlaying = true;
            const nowPlaying = document.getElementById('nowPlaying');
            nowPlaying.classList.add('active');

            
            const statusEl = document.getElementById('modeStatus');
            statusEl.textContent = 'PLAYING';
            statusEl.style.background = '#2196F3';
            statusEl.classList.add('playing');

            for (let i = 0; i < sequence.length; i++) {
                if (!isPlaying) break;

                const item = sequence[i];
                document.getElementById('currentCmd').textContent = item.name;
                document.getElementById('playProgress').textContent = `${i + 1} / ${sequence.length}`;

                await sendCmd(item.cmd);
                await sleep(item.duration);
            }

            stopPlayback();
        }

        function stopPlayback() {
            isPlaying = false;
            const nowPlaying = document.getElementById('nowPlaying');
            nowPlaying.classList.remove('active');

            
            const statusEl = document.getElementById('modeStatus');
            statusEl.textContent = 'REALTIME';
            statusEl.style.background = '#4CAF50';
            statusEl.classList.remove('playing');
            setMode('realtime');

            // Send stand command to stop robot
            sendCmd('stand');
        }

        function sleep(ms) {
            return new Promise(resolve => setTimeout(resolve, ms));
        }

        async function connect() {
            const res = await fetch(apiBase + '/api/connect', {method: 'POST'});
            const data = await res.json();
            updateConnStatus(data.connected);
            addLog('Connect', data.connected, data.time_ms || 0);
        }

        async function startSimulation() {
            addLog('Starting Simulation...', true, 0);
            try {
                const res = await fetch('/api/start_sim', {method: 'POST'});
                const data = await res.json();
                if (data.success) {
                    // Switch to simulation mode
                    isSimMode = true;
                    apiBase = 'http://localhost:8891';
                    const el = document.getElementById('targetStatus');
                    el.textContent = 'SIMULATION';
                    el.style.background = '#9C27B0';
                    addLog('Simulation Started', true, data.time_ms || 0);
                    // Wait a bit for sim to start, then auto-connect
                    setTimeout(async () => {
                        await connect();
                    }, 2000);
                } else {
                    addLog('Sim Error: ' + data.error, false, 0);
                }
            } catch(e) {
                addLog('Failed to start sim', false, 0);
            }
        }

        async function resetSimulation() {
            if (!isSimMode) {
                addLog('Switch to SIM mode first', false, 0);
                return;
            }
            try {
                const res = await fetch(apiBase + '/api/reset', {method: 'POST'});
                const data = await res.json();
                addLog('Sim Reset', data.success, data.time_ms || 0);
            } catch(e) {
                addLog('Reset failed', false, 0);
            }
        }

        async function openPoseBuilder() {
            addLog('Opening Pose Builder...', true, 0);
            // Start the server first
            fetch('/api/start_pose_builder', {method: 'POST'});
            // Open the page after a short delay
            setTimeout(() => {
                window.open('http://localhost:8892', '_blank');
            }, 1000);
        }

        async function openParamEditor() {
            addLog('Opening Param Editor...', true, 0);
            // Start the server first
            fetch('/api/start_param_editor', {method: 'POST'});
            // Open the page after a short delay
            setTimeout(() => {
                window.open('http://localhost:8893', '_blank');
            }, 1000);
        }

        async function openChoreographer() {
            addLog('Opening Choreographer...', true, 0);
            // Start the server first
            fetch('/api/start_choreographer', {method: 'POST'});
            // Open the page after a short delay
            setTimeout(() => {
                window.open('http://localhost:8894', '_blank');
            }, 1000);
        }

        async function loadCustomCommands() {
            try {
                const res = await fetch('/api/custom_commands');
                const cmds = await res.json();
                const container = document.getElementById('custom-cmd-btns');
                container.innerHTML = '';
                for (const [name, cmd] of Object.entries(cmds)) {
                    const btn = document.createElement('button');
                    btn.className = 'btn';
                    btn.style.background = '#FF9800';
                    btn.textContent = name;
                    btn.onclick = () => runCustomCommand(name, cmd);
                    container.appendChild(btn);
                }
                if (Object.keys(cmds).length === 0) {
                    container.innerHTML = '<span style="color:#666;font-size:11px;">No custom commands yet</span>';
                }
            } catch(e) {
                console.error('Failed to load custom commands');
            }
        }

        async function runCustomCommand(name, cmd) {
            if (currentMode === 'record') {
                sequence.push({ cmd: 'customCmd:' + name, name: name, duration: 500 });
                renderSequence();
            } else {
                try {
                    await fetch('/api/run_custom_command', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({base_cmd: cmd.base_cmd, params: cmd.params})
                    });
                    addLog(name, true, 0);
                } catch(e) {
                    addLog(name + ' failed', false, 0);
                }
            }
        }

        async function unlock() {
            const res = await fetch(apiBase + '/api/unlock', {method: 'POST'});
            const data = await res.json();
            addLog('Unlock', data.success, data.time_ms || 0);
        }

        async function sendCmd(cmd) {
            const res = await fetch(apiBase + '/api/cmd/' + cmd, {method: 'POST'});
            const data = await res.json();
            addLog(data.name, data.success, data.time_ms);
            updateStats();
            return data;
        }

        async function setLed(r, g, b) {
            await fetch(apiBase + `/api/led/${r}/${g}/${b}`, {method: 'POST'});
        }

        function updateConnStatus(connected) {
            const el = document.getElementById('connStatus');
            el.textContent = connected ? 'Connected' : 'Disconnected';
            el.className = 'status ' + (connected ? 'connected' : 'disconnected');
        }

        function addLog(name, success, time_ms) {
            const log = document.getElementById('log');
            const entry = document.createElement('div');
            entry.className = 'log-entry ' + (success ? 'success' : 'error');
            entry.innerHTML = `<span>${name}</span><span class="log-time">${time_ms}ms</span>`;
            log.insertBefore(entry, log.firstChild);
            if (log.children.length > 30) log.removeChild(log.lastChild);
        }

        function updateStats() {
            const entries = document.querySelectorAll('.log-entry');
            let total = 0, count = 0;
            entries.forEach(e => {
                const time = parseFloat(e.querySelector('.log-time').textContent);
                if (time > 0) { total += time; count++; }
            });
            document.getElementById('avgTime').textContent = count ? Math.round(total / count) : 0;
            document.getElementById('totalCmds').textContent = entries.length;
        }

        setInterval(async () => {
            try {
                const res = await fetch(apiBase + '/api/status');
                const data = await res.json();
                updateConnStatus(data.connected);
            } catch(e) {}
        }, 3000);

        createButtons();
        loadCustomPoses();
        loadCustomCommands();
        refreshSequenceList();
    </script>
</body>
</html>
"""


class DashboardHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            html = HTML.replace('COMMANDS_JSON', json.dumps(COMMANDS))
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode())
        elif self.path == '/api/status':
            self.send_json({'connected': connected, 'log': action_log[-10:]})
        elif self.path == '/api/custom_poses':
            poses = load_saved_poses()
            self.send_json(poses)
        elif self.path == '/api/sequence/list':
            files = get_sequence_files()
            self.send_json({'files': files})
        elif self.path.startswith('/api/sequence/load/'):
            name = urllib.parse.unquote(self.path.split('/')[-1])
            seq = load_sequence_file(name)
            self.send_json({'sequence': seq})
        elif self.path == '/api/custom_commands':
            cmds = load_custom_commands()
            self.send_json(cmds)
        else:
            self.send_error(404)

    def do_POST(self):
        global robot, connected

        if self.path == '/api/connect':
            start = time.time()
            try:
                robot = Go1()
                robot.init()
                connected = True
                elapsed = (time.time() - start) * 1000
                self.send_json({'connected': True, 'time_ms': round(elapsed, 1)})
            except Exception as e:
                connected = False
                self.send_json({'connected': False, 'error': str(e)})

        elif self.path == '/api/unlock':
            start = time.time()
            success, msg = unlock_sdk_mode()
            elapsed = (time.time() - start) * 1000
            self.send_json({'success': success, 'message': msg, 'time_ms': round(elapsed, 1)})

        elif self.path.startswith('/api/cmd/'):
            cmd = self.path.split('/')[-1]
            result = run_command(cmd)
            self.send_json(result)

        elif self.path.startswith('/api/led/'):
            parts = self.path.split('/')
            if len(parts) >= 6 and robot and connected:
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                robot.set_led_color(r, g, b)
                self.send_json({'success': True})
            else:
                self.send_json({'success': False})

        elif self.path == '/api/start_sim':
            start = time.time()
            try:
                # Find paths
                from pathlib import Path
                script_dir = Path(__file__).parent
                project_root = script_dir.parent
                mjpython = project_root / ".venv" / "bin" / "mjpython"
                sim_script = script_dir / "go1_sim_server.py"

                if not mjpython.exists():
                    self.send_json({'success': False, 'error': 'mjpython not found'})
                    return
                if not sim_script.exists():
                    self.send_json({'success': False, 'error': 'Sim script not found'})
                    return

                # Start simulation in background
                subprocess.Popen(
                    [str(mjpython), str(sim_script)],
                    cwd=str(project_root),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                elapsed = (time.time() - start) * 1000
                self.send_json({'success': True, 'time_ms': round(elapsed, 1)})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/start_pose_builder':
            start = time.time()
            try:
                script_dir = Path(__file__).parent
                project_root = script_dir.parent
                python_cmd = project_root / ".venv" / "bin" / "python"
                pose_script = script_dir / "go1_pose_builder.py"

                if not pose_script.exists():
                    self.send_json({'success': False, 'error': 'Pose Builder not found'})
                    return

                # Start Pose Builder in new terminal
                apple_script = f'''
                tell application "Terminal"
                    activate
                    do script "cd {project_root} && {python_cmd} {pose_script}"
                end tell
                '''
                subprocess.Popen(["osascript", "-e", apple_script])
                elapsed = (time.time() - start) * 1000
                self.send_json({'success': True, 'time_ms': round(elapsed, 1)})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/start_param_editor':
            start = time.time()
            try:
                script_dir = Path(__file__).parent
                project_root = script_dir.parent
                python_cmd = project_root / ".venv" / "bin" / "python"
                param_script = script_dir / "go1_param_editor.py"

                if not param_script.exists():
                    self.send_json({'success': False, 'error': 'Param Editor not found'})
                    return

                apple_script = f'''
                tell application "Terminal"
                    activate
                    do script "cd {project_root} && {python_cmd} {param_script}"
                end tell
                '''
                subprocess.Popen(["osascript", "-e", apple_script])
                elapsed = (time.time() - start) * 1000
                self.send_json({'success': True, 'time_ms': round(elapsed, 1)})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/start_choreographer':
            start = time.time()
            try:
                script_dir = Path(__file__).parent
                project_root = script_dir.parent
                python_cmd = project_root / ".venv" / "bin" / "python"
                choreo_script = script_dir / "go1_choreographer.py"

                if not choreo_script.exists():
                    self.send_json({'success': False, 'error': 'Choreographer not found'})
                    return

                apple_script = f'''
                tell application "Terminal"
                    activate
                    do script "cd {project_root} && {python_cmd} {choreo_script}"
                end tell
                '''
                subprocess.Popen(["osascript", "-e", apple_script])
                elapsed = (time.time() - start) * 1000
                self.send_json({'success': True, 'time_ms': round(elapsed, 1)})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/run_custom_command':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length).decode()
                data = json.loads(body)
                base_cmd = data.get('base_cmd')
                params = data.get('params', {})

                if robot and connected:
                    loop = asyncio.new_event_loop()
                    robot.set_mode(Go1Mode.STAND)
                    time.sleep(0.1)

                    intensity = params.get('intensity', 0.5)
                    duration = int(params.get('duration', 300))
                    speed = params.get('speed', 0.5)

                    if base_cmd == 'look_up':
                        loop.run_until_complete(robot.look_up(intensity, duration))
                    elif base_cmd == 'look_down':
                        loop.run_until_complete(robot.look_down(intensity, duration))
                    elif base_cmd == 'lean_left':
                        loop.run_until_complete(robot.lean_left(intensity, duration))
                    elif base_cmd == 'lean_right':
                        loop.run_until_complete(robot.lean_right(intensity, duration))
                    elif base_cmd == 'twist_left':
                        loop.run_until_complete(robot.twist_left(intensity, duration))
                    elif base_cmd == 'twist_right':
                        loop.run_until_complete(robot.twist_right(intensity, duration))
                    elif base_cmd == 'squat_down':
                        loop.run_until_complete(robot.squat_down(intensity, duration))
                    elif base_cmd == 'extend_up':
                        loop.run_until_complete(robot.extend_up(intensity, duration))
                    elif base_cmd == 'go_forward':
                        robot.set_mode(Go1Mode.WALK)
                        loop.run_until_complete(robot.go_forward(speed, duration))
                    elif base_cmd == 'go_backward':
                        robot.set_mode(Go1Mode.WALK)
                        loop.run_until_complete(robot.go_backward(speed, duration))
                    elif base_cmd == 'set_led':
                        robot.set_led_color(int(params.get('r', 0)), int(params.get('g', 0)), int(params.get('b', 0)))
                    loop.close()

                self.send_json({'success': True})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/sequence/save':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length).decode()
                data = json.loads(body)
                name = data.get('name', 'untitled')
                seq = data.get('sequence', [])
                save_sequence_file(name, seq)
                self.send_json({'success': True})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        elif self.path == '/api/custom_pose':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length).decode()
                data = json.loads(body)
                pose = data.get('pose', {})
                # Execute the custom body pose
                if robot and connected:
                    robot.set_mode(Go1Mode.STAND)
                    time.sleep(0.1)
                    loop = asyncio.new_event_loop()
                    roll = pose.get('roll', 0)
                    pitch = pose.get('pitch', 0)
                    yaw = pose.get('yaw', 0)
                    height = pose.get('height', 0)
                    if roll < 0:
                        loop.run_until_complete(robot.lean_left(abs(roll), 200))
                    elif roll > 0:
                        loop.run_until_complete(robot.lean_right(abs(roll), 200))
                    if pitch < 0:
                        loop.run_until_complete(robot.look_down(abs(pitch), 200))
                    elif pitch > 0:
                        loop.run_until_complete(robot.look_up(abs(pitch), 200))
                    if yaw < 0:
                        loop.run_until_complete(robot.twist_left(abs(yaw), 200))
                    elif yaw > 0:
                        loop.run_until_complete(robot.twist_right(abs(yaw), 200))
                    if height < 0:
                        loop.run_until_complete(robot.squat_down(abs(height), 200))
                    elif height > 0:
                        loop.run_until_complete(robot.extend_up(abs(height), 200))
                    loop.close()
                self.send_json({'success': True})
            except Exception as e:
                self.send_json({'success': False, 'error': str(e)})

        else:
            self.send_error(404)

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


def main():
    port = 8890
    server = HTTPServer(('', port), DashboardHandler)

    print("\n" + "=" * 50)
    print("  Go1 Control Dashboard")
    print("=" * 50)
    print(f"\n  Open: http://localhost:{port}")
    print("\n  MODES:")
    print("  REALTIME - Execute immediately on click")
    print("  RECORD   - Record actions to sequence")
    print("  PLAY     - Play recorded sequence")
    print("\n  Press Ctrl+C to stop")
    print("=" * 50 + "\n")

    webbrowser.open(f'http://localhost:{port}')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
