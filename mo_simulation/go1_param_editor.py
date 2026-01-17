#!/usr/bin/env python3
"""
Go1 Parameter Editor - See and tweak command parameters

Shows the actual function parameters for each command.
Adjust values and save as custom commands.

Run with: python mo_simulation/go1_param_editor.py
Open: http://localhost:8893
"""

import asyncio
import json
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
import webbrowser

try:
    from go1pylib import Go1, Go1Mode
except ImportError:
    print("ERROR: go1pylib not installed!")
    print("Run: pip install go1pylib")
    exit(1)

# Global state
robot = None
connected = False

SCRIPT_DIR = Path(__file__).parent
CUSTOM_COMMANDS_FILE = SCRIPT_DIR / "custom_commands.json"
DEFAULT_PARAMS_FILE = SCRIPT_DIR / "default_params.json"

# ============== COMMAND PARAMETERS ==============
# These are the actual parameters used by each command

COMMAND_PARAMS = {
    # Body Poses - function(intensity, duration_ms)
    "look_up": {
        "name": "Look Up",
        "function": "robot.look_up(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Tilt head/body upward"
    },
    "look_down": {
        "name": "Look Down",
        "function": "robot.look_down(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Tilt head/body downward"
    },
    "lean_left": {
        "name": "Lean Left",
        "function": "robot.lean_left(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Lean body to the left"
    },
    "lean_right": {
        "name": "Lean Right",
        "function": "robot.lean_right(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Lean body to the right"
    },
    "twist_left": {
        "name": "Twist Left",
        "function": "robot.twist_left(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Rotate body left (yaw)"
    },
    "twist_right": {
        "name": "Twist Right",
        "function": "robot.twist_right(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Rotate body right (yaw)"
    },
    "squat_down": {
        "name": "Squat Down",
        "function": "robot.squat_down(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Lower body height"
    },
    "extend_up": {
        "name": "Extend Up",
        "function": "robot.extend_up(intensity, duration)",
        "params": {"intensity": 0.5, "duration": 300},
        "category": "pose",
        "description": "Raise body height"
    },

    # Movement - function(speed, duration_ms)
    "go_forward": {
        "name": "Go Forward",
        "function": "robot.go_forward(speed, duration)",
        "params": {"speed": 0.5, "duration": 500},
        "category": "move",
        "description": "Walk forward"
    },
    "go_backward": {
        "name": "Go Backward",
        "function": "robot.go_backward(speed, duration)",
        "params": {"speed": 0.5, "duration": 500},
        "category": "move",
        "description": "Walk backward"
    },
    "go_left": {
        "name": "Strafe Left",
        "function": "robot.go_left(speed, duration)",
        "params": {"speed": 0.4, "duration": 500},
        "category": "move",
        "description": "Side-step left"
    },
    "go_right": {
        "name": "Strafe Right",
        "function": "robot.go_right(speed, duration)",
        "params": {"speed": 0.4, "duration": 500},
        "category": "move",
        "description": "Side-step right"
    },
    "turn_left": {
        "name": "Turn Left",
        "function": "robot.turn_left(speed, duration)",
        "params": {"speed": 0.6, "duration": 500},
        "category": "move",
        "description": "Rotate in place left"
    },
    "turn_right": {
        "name": "Turn Right",
        "function": "robot.turn_right(speed, duration)",
        "params": {"speed": 0.6, "duration": 500},
        "category": "move",
        "description": "Rotate in place right"
    },

    # Modes - robot.set_mode(Go1Mode.XXX)
    "stand": {
        "name": "Stand",
        "function": "robot.set_mode(Go1Mode.STAND)",
        "params": {},
        "category": "mode",
        "description": "Standing pose, ready for body control"
    },
    "stand_up": {
        "name": "Stand Up",
        "function": "robot.set_mode(Go1Mode.STAND_UP)",
        "params": {},
        "category": "mode",
        "description": "Stand up from sitting"
    },
    "stand_down": {
        "name": "Sit Down",
        "function": "robot.set_mode(Go1Mode.STAND_DOWN)",
        "params": {},
        "category": "mode",
        "description": "Sit down from standing"
    },
    "walk_mode": {
        "name": "Walk Mode",
        "function": "robot.set_mode(Go1Mode.WALK)",
        "params": {},
        "category": "mode",
        "description": "Enable walking gait"
    },
    "damping": {
        "name": "Damping",
        "function": "robot.set_mode(Go1Mode.DAMPING)",
        "params": {},
        "category": "mode",
        "description": "Soft/compliant joints"
    },
    "recover_stand": {
        "name": "Recovery Stand",
        "function": "robot.set_mode(Go1Mode.RECOVER_STAND)",
        "params": {},
        "category": "mode",
        "description": "Get up from fallen state"
    },

    # Dances & Special
    "dance1": {
        "name": "Dance 1",
        "function": "robot.set_mode(Go1Mode.DANCE1)",
        "params": {},
        "category": "dance",
        "description": "Built-in dance routine 1"
    },
    "dance2": {
        "name": "Dance 2",
        "function": "robot.set_mode(Go1Mode.DANCE2)",
        "params": {},
        "category": "dance",
        "description": "Built-in dance routine 2"
    },
    "straight_hand": {
        "name": "Straight Hand",
        "function": "robot.set_mode(Go1Mode.STRAIGHT_HAND1)",
        "params": {},
        "category": "dance",
        "description": "Raise front legs up"
    },
    "jump_yaw": {
        "name": "Jump Yaw",
        "function": "mqtt.publish('controller/action', 'jumpYaw')",
        "params": {},
        "category": "dance",
        "description": "Jump and rotate"
    },

    # LED
    "set_led": {
        "name": "Set LED Color",
        "function": "robot.set_led_color(r, g, b)",
        "params": {"r": 255, "g": 0, "b": 0},
        "category": "led",
        "description": "Set LED color (0-255 each)"
    },
}

# Custom saved commands
custom_commands = {}


def load_custom_commands():
    global custom_commands
    if CUSTOM_COMMANDS_FILE.exists():
        try:
            with open(CUSTOM_COMMANDS_FILE, 'r') as f:
                custom_commands = json.load(f)
        except:
            custom_commands = {}


def save_custom_commands():
    with open(CUSTOM_COMMANDS_FILE, 'w') as f:
        json.dump(custom_commands, f, indent=2)


def load_default_params():
    """Load saved default parameters."""
    if DEFAULT_PARAMS_FILE.exists():
        try:
            with open(DEFAULT_PARAMS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {}


def save_default_params(cmd_id, params):
    """Save default parameters for a command."""
    defaults = load_default_params()
    defaults[cmd_id] = params
    with open(DEFAULT_PARAMS_FILE, 'w') as f:
        json.dump(defaults, f, indent=2)


def get_command_params():
    """Get command params with saved defaults applied."""
    saved_defaults = load_default_params()
    params = {}
    for cmd_id, cmd in COMMAND_PARAMS.items():
        params[cmd_id] = dict(cmd)
        # Override with saved defaults if they exist
        if cmd_id in saved_defaults:
            params[cmd_id]['params'] = saved_defaults[cmd_id]
    return params


def connect_robot():
    global robot, connected
    try:
        robot = Go1()
        robot.init()
        connected = True
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        connected = False
        return False


def execute_command(cmd_id, params):
    """Execute a command with given parameters."""
    global robot, connected
    if not robot or not connected:
        return {"success": False, "error": "Not connected"}

    try:
        loop = asyncio.new_event_loop()

        # Body poses
        if cmd_id == "look_up":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.look_up(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "look_down":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.look_down(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "lean_left":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.lean_left(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "lean_right":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.lean_right(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "twist_left":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.twist_left(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "twist_right":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.twist_right(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "squat_down":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.squat_down(params.get("intensity", 0.5), params.get("duration", 300)))
        elif cmd_id == "extend_up":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.1)
            loop.run_until_complete(robot.extend_up(params.get("intensity", 0.5), params.get("duration", 300)))

        # Movement
        elif cmd_id == "go_forward":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.go_forward(params.get("speed", 0.5), params.get("duration", 500)))
        elif cmd_id == "go_backward":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.go_backward(params.get("speed", 0.5), params.get("duration", 500)))
        elif cmd_id == "go_left":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.go_left(params.get("speed", 0.4), params.get("duration", 500)))
        elif cmd_id == "go_right":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.go_right(params.get("speed", 0.4), params.get("duration", 500)))
        elif cmd_id == "turn_left":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.turn_left(params.get("speed", 0.6), params.get("duration", 500)))
        elif cmd_id == "turn_right":
            robot.set_mode(Go1Mode.WALK)
            loop.run_until_complete(robot.turn_right(params.get("speed", 0.6), params.get("duration", 500)))

        # Modes
        elif cmd_id == "stand":
            robot.set_mode(Go1Mode.STAND)
        elif cmd_id == "stand_up":
            robot.set_mode(Go1Mode.STAND_UP)
        elif cmd_id == "stand_down":
            robot.set_mode(Go1Mode.STAND_DOWN)
        elif cmd_id == "walk_mode":
            robot.set_mode(Go1Mode.WALK)
        elif cmd_id == "damping":
            robot.set_mode(Go1Mode.DAMPING)
        elif cmd_id == "recover_stand":
            robot.set_mode(Go1Mode.RECOVER_STAND)

        # Dances
        elif cmd_id == "dance1":
            robot.set_mode(Go1Mode.DANCE1)
        elif cmd_id == "dance2":
            robot.set_mode(Go1Mode.DANCE2)
        elif cmd_id == "straight_hand":
            robot.set_mode(Go1Mode.STRAIGHT_HAND1)
        elif cmd_id == "jump_yaw":
            robot.mqtt.client.publish("controller/action", "jumpYaw", qos=1)

        # LED
        elif cmd_id == "set_led":
            robot.set_led_color(params.get("r", 255), params.get("g", 0), params.get("b", 0))

        loop.close()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ============== HTML ==============

HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Go1 Parameter Editor</title>
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
        .header { text-align: center; margin-bottom: 20px; }
        .header h1 { font-size: 28px; color: #FF9800; }
        .header p { color: #888; font-size: 14px; margin-top: 5px; }

        .status { padding: 8px 16px; border-radius: 20px; font-size: 12px; display: inline-block; margin-bottom: 15px; }
        .status.connected { background: #4CAF50; }
        .status.disconnected { background: #f44336; }

        .container { max-width: 1200px; margin: 0 auto; }

        .top-bar { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 8px;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn:hover { transform: scale(1.05); }
        .btn-green { background: #4CAF50; color: white; }
        .btn-orange { background: #FF9800; color: white; }

        .category { margin-bottom: 25px; }
        .category h2 { font-size: 16px; color: #FF9800; margin-bottom: 10px; text-transform: uppercase; }

        .cmd-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(350px, 1fr)); gap: 15px; }

        .cmd-card {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 15px;
            border: 1px solid rgba(255,255,255,0.1);
        }
        .cmd-card:hover { border-color: #FF9800; }
        .cmd-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .cmd-name { font-size: 16px; font-weight: bold; color: #4CAF50; }
        .cmd-func { font-family: monospace; font-size: 11px; color: #888; background: rgba(0,0,0,0.3); padding: 4px 8px; border-radius: 4px; }
        .cmd-desc { font-size: 12px; color: #aaa; margin-bottom: 12px; }

        .param-row { display: flex; align-items: center; gap: 10px; margin-bottom: 8px; }
        .param-label { font-size: 12px; color: #888; min-width: 70px; }
        .param-input {
            flex: 1;
            padding: 6px 10px;
            border: 1px solid #444;
            border-radius: 6px;
            background: #222;
            color: white;
            font-size: 13px;
        }
        .param-slider { flex: 2; }

        .cmd-actions { display: flex; gap: 8px; margin-top: 12px; }
        .cmd-btn {
            flex: 1;
            padding: 8px;
            border: none;
            border-radius: 6px;
            font-size: 11px;
            font-weight: bold;
            cursor: pointer;
        }
        .cmd-btn.run { background: #4CAF50; color: white; }
        .cmd-btn.save { background: #9C27B0; color: white; }
        .cmd-btn:hover { opacity: 0.8; }

        .custom-section { margin-top: 30px; padding-top: 20px; border-top: 1px solid #333; }
        .custom-section h2 { color: #9C27B0; }
        .custom-cmd { background: rgba(156,39,176,0.1); border-color: #9C27B0; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Go1 Parameter Editor</h1>
        <p>See and tweak the actual function parameters for each command</p>
    </div>

    <div class="container">
        <div class="top-bar">
            <span class="status disconnected" id="connStatus">Disconnected</span>
            <button class="btn btn-green" onclick="connect()">Connect to Robot</button>
            <button class="btn btn-orange" onclick="resetRobot()">Recovery Stand</button>
        </div>

        <div id="commands"></div>

        <div class="custom-section" id="customSection">
            <h2>Saved Custom Commands</h2>
            <div class="cmd-grid" id="customCommands"></div>
        </div>
    </div>

    <script>
        const COMMANDS = COMMANDS_JSON;
        let customCommands = {};

        function renderCommands() {
            const container = document.getElementById('commands');
            const categories = {};

            // Group by category
            for (const [id, cmd] of Object.entries(COMMANDS)) {
                if (!categories[cmd.category]) categories[cmd.category] = [];
                categories[cmd.category].push({id, ...cmd});
            }

            let html = '';
            const categoryNames = {
                'pose': 'Body Poses',
                'move': 'Movement',
                'mode': 'Modes',
                'dance': 'Dance & Special',
                'led': 'LED Control'
            };

            for (const [cat, cmds] of Object.entries(categories)) {
                html += `<div class="category"><h2>${categoryNames[cat] || cat}</h2><div class="cmd-grid">`;
                for (const cmd of cmds) {
                    html += renderCommandCard(cmd.id, cmd);
                }
                html += '</div></div>';
            }

            container.innerHTML = html;
        }

        function renderCommandCard(id, cmd) {
            let paramsHtml = '';
            for (const [param, value] of Object.entries(cmd.params || {})) {
                if (param === 'intensity' || param === 'speed') {
                    paramsHtml += `
                        <div class="param-row">
                            <span class="param-label">${param}:</span>
                            <input type="range" class="param-slider" min="0" max="1" step="0.1" value="${value}" id="${id}_${param}" oninput="updateValue('${id}', '${param}')">
                            <input type="number" class="param-input" style="width:60px" min="0" max="1" step="0.1" value="${value}" id="${id}_${param}_val" onchange="syncSlider('${id}', '${param}')">
                        </div>`;
                } else if (param === 'duration') {
                    paramsHtml += `
                        <div class="param-row">
                            <span class="param-label">${param} (ms):</span>
                            <input type="range" class="param-slider" min="100" max="2000" step="100" value="${value}" id="${id}_${param}" oninput="updateValue('${id}', '${param}')">
                            <input type="number" class="param-input" style="width:70px" min="100" max="5000" step="50" value="${value}" id="${id}_${param}_val" onchange="syncSlider('${id}', '${param}')">
                        </div>`;
                } else if (param === 'r' || param === 'g' || param === 'b') {
                    paramsHtml += `
                        <div class="param-row">
                            <span class="param-label">${param.toUpperCase()}:</span>
                            <input type="range" class="param-slider" min="0" max="255" step="1" value="${value}" id="${id}_${param}" oninput="updateValue('${id}', '${param}')">
                            <input type="number" class="param-input" style="width:60px" min="0" max="255" value="${value}" id="${id}_${param}_val" onchange="syncSlider('${id}', '${param}')">
                        </div>`;
                }
            }

            const hasParams = Object.keys(cmd.params || {}).length > 0;

            return `
                <div class="cmd-card">
                    <div class="cmd-header">
                        <span class="cmd-name">${cmd.name}</span>
                    </div>
                    <div class="cmd-func">${cmd.function}</div>
                    <div class="cmd-desc">${cmd.description}</div>
                    ${paramsHtml}
                    <div class="cmd-actions">
                        <button class="cmd-btn run" onclick="runCommand('${id}')">Run</button>
                        ${hasParams ? `<button class="cmd-btn save" style="background:#4CAF50" onclick="saveAsDefault('${id}')">Save Default</button>` : ''}
                        ${hasParams ? `<button class="cmd-btn save" onclick="saveAsCustom('${id}')">Save Custom</button>` : ''}
                    </div>
                </div>`;
        }

        function updateValue(id, param) {
            const slider = document.getElementById(id + '_' + param);
            const input = document.getElementById(id + '_' + param + '_val');
            input.value = slider.value;
        }

        function syncSlider(id, param) {
            const slider = document.getElementById(id + '_' + param);
            const input = document.getElementById(id + '_' + param + '_val');
            slider.value = input.value;
        }

        function getParams(id) {
            const cmd = COMMANDS[id];
            const params = {};
            for (const param of Object.keys(cmd.params || {})) {
                const input = document.getElementById(id + '_' + param + '_val');
                if (input) {
                    params[param] = parseFloat(input.value);
                }
            }
            return params;
        }

        async function connect() {
            const res = await fetch('/api/connect', {method: 'POST'});
            const data = await res.json();
            const el = document.getElementById('connStatus');
            el.textContent = data.connected ? 'Connected' : 'Disconnected';
            el.className = 'status ' + (data.connected ? 'connected' : 'disconnected');
        }

        async function resetRobot() {
            await fetch('/api/execute', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: 'recover_stand', params: {}})
            });
        }

        async function runCommand(id) {
            const params = getParams(id);
            await fetch('/api/execute', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: id, params: params})
            });
        }

        async function saveAsDefault(id) {
            const params = getParams(id);
            const res = await fetch('/api/save_default', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: id, params: params})
            });
            if (res.ok) {
                alert('Default saved! Main dashboard will use these values.');
            }
        }

        function saveAsCustom(id) {
            const name = prompt('Enter name for custom command:');
            if (!name) return;

            const params = getParams(id);
            const cmd = COMMANDS[id];

            fetch('/api/save_custom', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    name: name,
                    base_cmd: id,
                    params: params,
                    function: cmd.function,
                    description: cmd.description
                })
            }).then(() => loadCustomCommands());
        }

        async function loadCustomCommands() {
            const res = await fetch('/api/custom_commands');
            customCommands = await res.json();
            renderCustomCommands();
        }

        function renderCustomCommands() {
            const container = document.getElementById('customCommands');
            if (Object.keys(customCommands).length === 0) {
                container.innerHTML = '<p style="color:#666">No custom commands saved yet</p>';
                return;
            }

            let html = '';
            for (const [name, cmd] of Object.entries(customCommands)) {
                html += `
                    <div class="cmd-card custom-cmd">
                        <div class="cmd-header">
                            <span class="cmd-name">${name}</span>
                            <span style="color:#f44336;cursor:pointer" onclick="deleteCustom('${name}')">x</span>
                        </div>
                        <div class="cmd-func">${cmd.function}</div>
                        <div class="cmd-desc">Params: ${JSON.stringify(cmd.params)}</div>
                        <div class="cmd-actions">
                            <button class="cmd-btn run" onclick="runCustom('${name}')">Run</button>
                        </div>
                    </div>`;
            }
            container.innerHTML = html;
        }

        async function runCustom(name) {
            const cmd = customCommands[name];
            await fetch('/api/execute', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: cmd.base_cmd, params: cmd.params})
            });
        }

        async function deleteCustom(name) {
            if (!confirm('Delete "' + name + '"?')) return;
            await fetch('/api/delete_custom', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({name: name})
            });
            loadCustomCommands();
        }

        renderCommands();
        loadCustomCommands();
    </script>
</body>
</html>
"""


class ParamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            # Use merged params (saved defaults + base params)
            html = HTML.replace('COMMANDS_JSON', json.dumps(get_command_params()))
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode())
        elif self.path == '/api/custom_commands':
            self.send_json(custom_commands)
        else:
            self.send_error(404)

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode() if content_length > 0 else '{}'
        try:
            data = json.loads(body) if body else {}
        except:
            data = {}

        if self.path == '/api/connect':
            success = connect_robot()
            self.send_json({'connected': success})

        elif self.path == '/api/execute':
            cmd = data.get('cmd')
            params = data.get('params', {})
            result = execute_command(cmd, params)
            self.send_json(result)

        elif self.path == '/api/save_custom':
            name = data.get('name')
            custom_commands[name] = {
                'base_cmd': data.get('base_cmd'),
                'params': data.get('params'),
                'function': data.get('function'),
                'description': data.get('description')
            }
            save_custom_commands()
            self.send_json({'success': True})

        elif self.path == '/api/delete_custom':
            name = data.get('name')
            if name in custom_commands:
                del custom_commands[name]
                save_custom_commands()
            self.send_json({'success': True})

        elif self.path == '/api/save_default':
            cmd_id = data.get('cmd')
            params = data.get('params', {})
            save_default_params(cmd_id, params)
            self.send_json({'success': True})

        else:
            self.send_error(404)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


def main():
    load_custom_commands()

    port = 8893
    server = HTTPServer(('', port), ParamHandler)

    print("\n" + "=" * 55)
    print("  Go1 Parameter Editor")
    print("=" * 55)
    print(f"\n  Open: http://localhost:{port}")
    print("\n  - See actual function parameters")
    print("  - Adjust with sliders")
    print("  - Save as custom commands")
    print("\n  Press Ctrl+C to stop")
    print("=" * 55 + "\n")

    webbrowser.open(f'http://localhost:{port}')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
