#!/usr/bin/env python3
"""
Go1 Visual Dashboard - Web-based control with timing info

Features:
- Visual buttons for all commands
- Real-time timing for each action
- LED color picker
- Connection status
"""

import asyncio
import json
import subprocess
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser

try:
    from go1pylib import Go1, Go1Mode
except ImportError:
    print("ERROR: go1pylib not installed!")
    print("Run: pip install go1pylib")
    exit(1)

# Global robot instance
robot = None
action_log = []
connected = False

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
        elapsed = (time.time() - start) * 1000  # ms
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
    "stand": {"name": "Stand", "category": "mode", "color": "#4CAF50"},
    "standUp": {"name": "Stand Up", "category": "mode", "color": "#4CAF50"},
    "standDown": {"name": "Sit Down", "category": "mode", "color": "#4CAF50"},
    "walk": {"name": "Walk Mode", "category": "mode", "color": "#2196F3"},
    "run": {"name": "Run Mode", "category": "mode", "color": "#2196F3"},
    "climb": {"name": "Climb Mode", "category": "mode", "color": "#2196F3"},
    "damping": {"name": "Damping", "category": "mode", "color": "#FF9800"},
    "recoverStand": {"name": "Recovery", "category": "mode", "color": "#FF9800"},

    # Dances
    "dance1": {"name": "Dance 1", "category": "dance", "color": "#E91E63"},
    "dance2": {"name": "Dance 2", "category": "dance", "color": "#E91E63"},
    "dance3": {"name": "Dance 3", "category": "dance", "color": "#9C27B0"},
    "dance4": {"name": "Dance 4", "category": "dance", "color": "#9C27B0"},

    # Special moves
    "jumpYaw": {"name": "Jump Yaw", "category": "special", "color": "#FF5722"},
    "straightHand1": {"name": "Straight Hand", "category": "special", "color": "#FF5722"},
    "frontJump": {"name": "Front Jump", "category": "special", "color": "#FF5722"},
    "frontPounce": {"name": "Front Pounce", "category": "special", "color": "#FF5722"},
    "stretch": {"name": "Stretch", "category": "special", "color": "#795548"},
    "pray": {"name": "Pray", "category": "special", "color": "#795548"},
    "handStand": {"name": "Hand Stand", "category": "special", "color": "#795548"},
    "wiggleHips": {"name": "Wiggle Hips", "category": "special", "color": "#795548"},
    "bound": {"name": "Bound Jump", "category": "special", "color": "#795548"},
    "backflip": {"name": "BACKFLIP", "category": "danger", "color": "#f44336"},

    # Movement
    "forward": {"name": "Forward", "category": "move", "color": "#00BCD4"},
    "backward": {"name": "Backward", "category": "move", "color": "#00BCD4"},
    "left": {"name": "Strafe Left", "category": "move", "color": "#00BCD4"},
    "right": {"name": "Strafe Right", "category": "move", "color": "#00BCD4"},
    "turnLeft": {"name": "Turn Left", "category": "move", "color": "#00BCD4"},
    "turnRight": {"name": "Turn Right", "category": "move", "color": "#00BCD4"},

    # Body pose
    "lookUp": {"name": "Look Up", "category": "pose", "color": "#607D8B"},
    "lookDown": {"name": "Look Down", "category": "pose", "color": "#607D8B"},
    "leanLeft": {"name": "Lean Left", "category": "pose", "color": "#607D8B"},
    "leanRight": {"name": "Lean Right", "category": "pose", "color": "#607D8B"},
    "twistLeft": {"name": "Twist Left", "category": "pose", "color": "#607D8B"},
    "twistRight": {"name": "Twist Right", "category": "pose", "color": "#607D8B"},
    "squat": {"name": "Squat", "category": "pose", "color": "#607D8B"},
    "extend": {"name": "Extend", "category": "pose", "color": "#607D8B"},
}


def execute_command(cmd):
    """Execute a command and return timing."""
    global robot

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


# ============== WEB SERVER ==============

HTML = """<!DOCTYPE html>
<html>
<head>
    <title>Go1 Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: white;
            padding: 20px;
        }
        .header {
            text-align: center;
            margin-bottom: 20px;
        }
        .header h1 {
            font-size: 28px;
            color: #ffd700;
            margin-bottom: 5px;
        }
        .status {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
        }
        .status.connected { background: #4CAF50; }
        .status.disconnected { background: #f44336; }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 300px;
            gap: 20px;
        }
        @media (max-width: 900px) {
            .container { grid-template-columns: 1fr; }
        }
        .panel {
            background: rgba(255,255,255,0.05);
            border-radius: 15px;
            padding: 15px;
            border: 1px solid rgba(255,255,255,0.1);
        }
        .panel h2 {
            font-size: 14px;
            color: #ffd700;
            margin-bottom: 15px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .btn-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(100px, 1fr));
            gap: 8px;
        }
        .btn {
            padding: 12px 8px;
            border: none;
            border-radius: 10px;
            font-size: 11px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            color: white;
            text-transform: uppercase;
        }
        .btn:hover {
            transform: scale(1.05);
            box-shadow: 0 5px 20px rgba(0,0,0,0.3);
        }
        .btn:active { transform: scale(0.95); }
        .btn.danger { animation: pulse 1s infinite; }
        @keyframes pulse {
            0%, 100% { box-shadow: 0 0 0 0 rgba(244,67,54,0.5); }
            50% { box-shadow: 0 0 0 10px rgba(244,67,54,0); }
        }
        .category { margin-bottom: 20px; }
        .category-title {
            font-size: 11px;
            color: rgba(255,255,255,0.5);
            margin-bottom: 8px;
            text-transform: uppercase;
        }
        .log {
            max-height: 400px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 11px;
        }
        .log-entry {
            padding: 8px 10px;
            border-bottom: 1px solid rgba(255,255,255,0.05);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .log-entry.success { border-left: 3px solid #4CAF50; }
        .log-entry.error { border-left: 3px solid #f44336; }
        .log-time {
            color: #ffd700;
            font-weight: bold;
        }
        .led-picker {
            display: flex;
            gap: 10px;
            margin-top: 10px;
        }
        .led-btn {
            width: 40px;
            height: 40px;
            border-radius: 50%;
            border: 3px solid rgba(255,255,255,0.3);
            cursor: pointer;
        }
        .led-btn:hover { border-color: white; }
        .connect-btn {
            width: 100%;
            padding: 15px;
            font-size: 14px;
            margin-bottom: 15px;
        }
        .timing-stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 15px;
        }
        .stat {
            background: rgba(0,0,0,0.3);
            padding: 10px;
            border-radius: 8px;
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #ffd700;
        }
        .stat-label {
            font-size: 10px;
            color: rgba(255,255,255,0.5);
            text-transform: uppercase;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🐕 Go1 Dashboard</h1>
        <span class="status disconnected" id="status">Disconnected</span>
    </div>

    <div class="container">
        <div class="main-panel">
            <div class="panel">
                <button class="btn connect-btn" style="background: #4CAF50;" onclick="connect()">
                    Connect to Go1
                </button>
                <button class="btn" style="background: #FF9800; width: 100%;" onclick="unlock()">
                    🔓 Unlock SDK Mode
                </button>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>Mode Controls</h2>
                <div class="btn-grid" id="mode-btns"></div>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>Dance & Special Moves</h2>
                <div class="btn-grid" id="dance-btns"></div>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>Movement</h2>
                <div class="btn-grid" id="move-btns"></div>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>Body Pose</h2>
                <div class="btn-grid" id="pose-btns"></div>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>LED Control</h2>
                <div class="led-picker">
                    <div class="led-btn" style="background: #f44336;" onclick="setLed(255,0,0)"></div>
                    <div class="led-btn" style="background: #4CAF50;" onclick="setLed(0,255,0)"></div>
                    <div class="led-btn" style="background: #2196F3;" onclick="setLed(0,0,255)"></div>
                    <div class="led-btn" style="background: #ffd700;" onclick="setLed(255,215,0)"></div>
                    <div class="led-btn" style="background: #E91E63;" onclick="setLed(255,20,147)"></div>
                    <div class="led-btn" style="background: #333;" onclick="setLed(0,0,0)"></div>
                </div>
            </div>
        </div>

        <div class="side-panel">
            <div class="panel">
                <h2>Timing Stats</h2>
                <div class="timing-stats">
                    <div class="stat">
                        <div class="stat-value" id="avg-time">0</div>
                        <div class="stat-label">Avg (ms)</div>
                    </div>
                    <div class="stat">
                        <div class="stat-value" id="total-cmds">0</div>
                        <div class="stat-label">Commands</div>
                    </div>
                </div>
            </div>

            <div class="panel" style="margin-top: 15px;">
                <h2>Action Log</h2>
                <div class="log" id="log"></div>
            </div>
        </div>
    </div>

    <script>
        const COMMANDS = COMMANDS_JSON;

        function createButtons() {
            const categories = {
                'mode': 'mode-btns',
                'dance': 'dance-btns',
                'special': 'dance-btns',
                'danger': 'dance-btns',
                'move': 'move-btns',
                'pose': 'pose-btns'
            };

            for (const [cmd, info] of Object.entries(COMMANDS)) {
                const container = document.getElementById(categories[info.category]);
                if (!container) continue;

                const btn = document.createElement('button');
                btn.className = 'btn' + (info.category === 'danger' ? ' danger' : '');
                btn.style.background = info.color;
                btn.textContent = info.name;
                btn.onclick = () => sendCmd(cmd);
                container.appendChild(btn);
            }
        }

        async function connect() {
            const res = await fetch('/api/connect', {method: 'POST'});
            const data = await res.json();
            updateStatus(data.connected);
            addLog('Connect', data.connected, data.time_ms || 0);
        }

        async function unlock() {
            const res = await fetch('/api/unlock', {method: 'POST'});
            const data = await res.json();
            addLog('Unlock SDK', data.success, data.time_ms || 0);
        }

        async function sendCmd(cmd) {
            const res = await fetch('/api/cmd/' + cmd, {method: 'POST'});
            const data = await res.json();
            addLog(data.name, data.success, data.time_ms);
            updateStats();
        }

        async function setLed(r, g, b) {
            await fetch(`/api/led/${r}/${g}/${b}`, {method: 'POST'});
            addLog(`LED(${r},${g},${b})`, true, 0);
        }

        function updateStatus(connected) {
            const el = document.getElementById('status');
            el.textContent = connected ? 'Connected' : 'Disconnected';
            el.className = 'status ' + (connected ? 'connected' : 'disconnected');
        }

        function addLog(name, success, time_ms) {
            const log = document.getElementById('log');
            const entry = document.createElement('div');
            entry.className = 'log-entry ' + (success ? 'success' : 'error');
            entry.innerHTML = `<span>${name}</span><span class="log-time">${time_ms}ms</span>`;
            log.insertBefore(entry, log.firstChild);
            if (log.children.length > 50) log.removeChild(log.lastChild);
            updateStats();
        }

        function updateStats() {
            const entries = document.querySelectorAll('.log-entry');
            let total = 0, count = 0;
            entries.forEach(e => {
                const time = parseFloat(e.querySelector('.log-time').textContent);
                if (time > 0) { total += time; count++; }
            });
            document.getElementById('avg-time').textContent = count ? Math.round(total / count) : 0;
            document.getElementById('total-cmds').textContent = entries.length;
        }

        // Poll status
        setInterval(async () => {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                updateStatus(data.connected);
            } catch(e) {}
        }, 2000);

        createButtons();
    </script>
</body>
</html>
"""


class DashboardHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress logs

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            html = HTML.replace('COMMANDS_JSON', json.dumps(COMMANDS))
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
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
            category = COMMANDS.get(cmd, {}).get('category', '')

            if category in ['move']:
                # Run async movement
                loop = asyncio.new_event_loop()
                result = loop.run_until_complete(execute_movement(cmd))
                loop.close()
            elif category in ['pose']:
                loop = asyncio.new_event_loop()
                result = loop.run_until_complete(execute_pose(cmd))
                loop.close()
            else:
                result = execute_command(cmd)

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
    print("  🐕 Go1 Visual Dashboard")
    print("=" * 50)
    print(f"\n  Open: http://localhost:{port}")
    print("\n  Features:")
    print("  - Visual buttons for all commands")
    print("  - Real-time timing for each action")
    print("  - Action log with success/failure")
    print("  - LED color picker")
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
