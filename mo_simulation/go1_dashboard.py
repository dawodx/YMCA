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
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser

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

    # Dances
    "dance1": {"name": "Dance 1", "category": "dance", "color": "#E91E63", "duration": 5000},
    "dance2": {"name": "Dance 2", "category": "dance", "color": "#E91E63", "duration": 5000},
    "dance3": {"name": "Dance 3", "category": "dance", "color": "#9C27B0", "duration": 5000},
    "dance4": {"name": "Dance 4", "category": "dance", "color": "#9C27B0", "duration": 5000},

    # Special moves
    "jumpYaw": {"name": "Jump Yaw", "category": "special", "color": "#FF5722", "duration": 2000},
    "straightHand1": {"name": "Straight Hand", "category": "special", "color": "#FF5722", "duration": 2000},
    "frontJump": {"name": "Front Jump", "category": "special", "color": "#FF5722", "duration": 1500},
    "frontPounce": {"name": "Front Pounce", "category": "special", "color": "#FF5722", "duration": 1500},
    "stretch": {"name": "Stretch", "category": "special", "color": "#795548", "duration": 2000},
    "pray": {"name": "Pray", "category": "special", "color": "#795548", "duration": 2000},
    "handStand": {"name": "Hand Stand", "category": "special", "color": "#795548", "duration": 3000},
    "wiggleHips": {"name": "Wiggle Hips", "category": "special", "color": "#795548", "duration": 2000},
    "bound": {"name": "Bound Jump", "category": "special", "color": "#795548", "duration": 1500},
    "backflip": {"name": "BACKFLIP", "category": "danger", "color": "#f44336", "duration": 3000},

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

    # Raw MQTT commands
    mqtt_commands = ["dance3", "dance4", "jumpYaw", "frontJump", "frontPounce",
                     "stretch", "pray", "handStand", "wiggleHips", "bound", "backflip"]

    if cmd in mqtt_commands:
        return timed_action(cmd, lambda: robot.mqtt.client.publish("controller/action", cmd, qos=1))

    return {"name": cmd, "time_ms": 0, "success": False, "error": "Unknown command"}


async def execute_movement(cmd):
    """Execute movement command."""
    global robot

    if not robot or not connected:
        return {"name": cmd, "time_ms": 0, "success": False, "error": "Not connected"}

    robot.set_mode(Go1Mode.WALK)

    if cmd == "forward":
        return await timed_async_action(cmd, robot.go_forward(0.5, 200))
    elif cmd == "backward":
        return await timed_async_action(cmd, robot.go_backward(0.5, 200))
    elif cmd == "left":
        return await timed_async_action(cmd, robot.go_left(0.4, 200))
    elif cmd == "right":
        return await timed_async_action(cmd, robot.go_right(0.4, 200))
    elif cmd == "turnLeft":
        return await timed_async_action(cmd, robot.turn_left(0.6, 200))
    elif cmd == "turnRight":
        return await timed_async_action(cmd, robot.turn_right(0.6, 200))

    return {"name": cmd, "time_ms": 0, "success": False, "error": "Unknown movement"}


async def execute_pose(cmd):
    """Execute pose command."""
    global robot

    if not robot or not connected:
        return {"name": cmd, "time_ms": 0, "success": False, "error": "Not connected"}

    robot.set_mode(Go1Mode.STAND)
    await asyncio.sleep(0.1)

    if cmd == "lookUp":
        return await timed_async_action(cmd, robot.look_up(0.5, 300))
    elif cmd == "lookDown":
        return await timed_async_action(cmd, robot.look_down(0.5, 300))
    elif cmd == "leanLeft":
        return await timed_async_action(cmd, robot.lean_left(0.5, 300))
    elif cmd == "leanRight":
        return await timed_async_action(cmd, robot.lean_right(0.5, 300))
    elif cmd == "twistLeft":
        return await timed_async_action(cmd, robot.twist_left(0.5, 300))
    elif cmd == "twistRight":
        return await timed_async_action(cmd, robot.twist_right(0.5, 300))
    elif cmd == "squat":
        return await timed_async_action(cmd, robot.squat_down(0.5, 300))
    elif cmd == "extend":
        return await timed_async_action(cmd, robot.extend_up(0.5, 300))

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
        .mode-btn.play { background: #2196F3; }
        .mode-btn.active { border-color: #ffd700; transform: scale(1.1); }
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
                <h2>LED</h2>
                <div class="led-picker">
                    <div class="led-btn" style="background:#f44336;" onclick="setLed(255,0,0)"></div>
                    <div class="led-btn" style="background:#4CAF50;" onclick="setLed(0,255,0)"></div>
                    <div class="led-btn" style="background:#2196F3;" onclick="setLed(0,0,255)"></div>
                    <div class="led-btn" style="background:#ffd700;" onclick="setLed(255,215,0)"></div>
                    <div class="led-btn" style="background:#E91E63;" onclick="setLed(255,20,147)"></div>
                    <div class="led-btn" style="background:#00BCD4;" onclick="setLed(0,188,212)"></div>
                    <div class="led-btn" style="background:#333;" onclick="setLed(0,0,0)"></div>
                </div>
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
                    <button class="seq-btn" style="background:#9C27B0;" onclick="saveSequence()">Save</button>
                    <button class="seq-btn" style="background:#607D8B;" onclick="loadSequence()">Load</button>
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

        function createButtons() {
            const categories = {
                'mode': 'mode-btns',
                'dance': 'dance-btns',
                'special': 'dance-btns',
                'danger': 'dance-btns',
                'move': 'move-btns',
                'pose': 'pose-btns',
                'wait': 'wait-btns'
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
            document.querySelector('.mode-btn.' + mode).classList.add('active');

            const statusEl = document.getElementById('modeStatus');
            if (mode === 'realtime') {
                statusEl.textContent = 'REALTIME';
                statusEl.style.background = '#4CAF50';
            } else if (mode === 'record') {
                statusEl.textContent = 'RECORDING';
                statusEl.style.background = '#f44336';
                statusEl.classList.add('recording');
            }
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

        function saveSequence() {
            const json = JSON.stringify(sequence);
            localStorage.setItem('go1_sequence', json);
            alert('Sequence saved! (' + sequence.length + ' actions)');
        }

        function loadSequence() {
            const json = localStorage.getItem('go1_sequence');
            if (json) {
                sequence = JSON.parse(json);
                renderSequence();
                alert('Sequence loaded! (' + sequence.length + ' actions)');
            } else {
                alert('No saved sequence found');
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
            const res = await fetch('/api/connect', {method: 'POST'});
            const data = await res.json();
            updateConnStatus(data.connected);
            addLog('Connect', data.connected, data.time_ms || 0);
        }

        async function unlock() {
            const res = await fetch('/api/unlock', {method: 'POST'});
            const data = await res.json();
            addLog('Unlock', data.success, data.time_ms || 0);
        }

        async function sendCmd(cmd) {
            const res = await fetch('/api/cmd/' + cmd, {method: 'POST'});
            const data = await res.json();
            addLog(data.name, data.success, data.time_ms);
            updateStats();
            return data;
        }

        async function setLed(r, g, b) {
            await fetch(`/api/led/${r}/${g}/${b}`, {method: 'POST'});
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
                const res = await fetch('/api/status');
                const data = await res.json();
                updateConnStatus(data.connected);
            } catch(e) {}
        }, 3000);

        createButtons();
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
