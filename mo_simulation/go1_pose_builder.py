#!/usr/bin/env python3
"""
Go1 Pose Builder - Body Pose Control for Real Robot

Create poses using body adjustments (lean, look, twist, squat, extend).
Works with the real Go1 robot via go1pylib.

Run with: python mo_simulation/go1_pose_builder.py
Open: http://localhost:8892
"""

import asyncio
import time
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
import webbrowser
from pathlib import Path

try:
    from go1pylib import Go1, Go1Mode
except ImportError:
    print("ERROR: go1pylib not installed!")
    print("Run: pip install go1pylib")
    exit(1)

# Global state
robot = None
connected = False
saved_poses = {}
sequence = []

# Current body pose values (normalized -1 to 1)
current_pose = {
    "roll": 0.0,      # Lean left(-) / right(+)
    "pitch": 0.0,     # Look down(-) / up(+)
    "yaw": 0.0,       # Twist left(-) / right(+)
    "height": 0.0,    # Squat(-) / extend(+)
}

# ============== ROBOT CONNECTION ==============

def connect_robot():
    """Connect to real Go1 robot."""
    global robot, connected
    try:
        robot = Go1()
        robot.init()
        connected = True
        print("Connected to Go1 robot!")
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        connected = False
        return False

def send_body_pose(pose):
    """Send body pose adjustments to robot."""
    global robot, connected
    if not robot or not connected:
        return False

    try:
        # Put in stand mode first
        robot.set_mode(Go1Mode.STAND)
        time.sleep(0.1)

        loop = asyncio.new_event_loop()

        # Apply each adjustment
        roll = pose.get("roll", 0)
        pitch = pose.get("pitch", 0)
        yaw = pose.get("yaw", 0)
        height = pose.get("height", 0)

        # Roll (lean left/right)
        if roll < 0:
            loop.run_until_complete(robot.lean_left(abs(roll), 200))
        elif roll > 0:
            loop.run_until_complete(robot.lean_right(abs(roll), 200))

        # Pitch (look up/down)
        if pitch < 0:
            loop.run_until_complete(robot.look_down(abs(pitch), 200))
        elif pitch > 0:
            loop.run_until_complete(robot.look_up(abs(pitch), 200))

        # Yaw (twist left/right)
        if yaw < 0:
            loop.run_until_complete(robot.twist_left(abs(yaw), 200))
        elif yaw > 0:
            loop.run_until_complete(robot.twist_right(abs(yaw), 200))

        # Height (squat/extend)
        if height < 0:
            loop.run_until_complete(robot.squat_down(abs(height), 200))
        elif height > 0:
            loop.run_until_complete(robot.extend_up(abs(height), 200))

        loop.close()
        return True
    except Exception as e:
        print(f"Send failed: {e}")
        return False


def execute_move(move_name):
    """Execute a special move."""
    global robot, connected
    if not robot or not connected:
        return False

    try:
        # Direct mode commands
        if move_name == "straightHand":
            robot.set_mode(Go1Mode.STRAIGHT_HAND1)
        elif move_name == "dance1":
            robot.set_mode(Go1Mode.DANCE1)
        elif move_name == "dance2":
            robot.set_mode(Go1Mode.DANCE2)
        elif move_name == "stand":
            robot.set_mode(Go1Mode.STAND)
        elif move_name == "standDown":
            robot.set_mode(Go1Mode.STAND_DOWN)
        elif move_name == "standUp":
            robot.set_mode(Go1Mode.STAND_UP)
        elif move_name == "recover":
            robot.set_mode(Go1Mode.RECOVER_STAND)
        # MQTT action commands
        elif move_name == "jumpYaw":
            robot.mqtt.client.publish("controller/action", "jumpYaw", qos=1)
        elif move_name == "pray":
            robot.mqtt.client.publish("controller/action", "pray", qos=1)
        # Body pose combos for front leg effect
        elif move_name == "frontUp":
            robot.set_mode(Go1Mode.STAND)
            time.sleep(0.2)
            loop = asyncio.new_event_loop()
            loop.run_until_complete(robot.look_up(0.9, 300))
            loop.close()
        return True
    except Exception as e:
        print(f"Move failed: {e}")
        return False

# ============== POSE MANAGEMENT ==============

def save_pose(name, pose):
    """Save a pose with given name."""
    saved_poses[name] = pose.copy()
    save_poses_to_file()
    return True

def delete_pose(name):
    """Delete a saved pose."""
    if name in saved_poses:
        del saved_poses[name]
        save_poses_to_file()
        return True
    return False

def save_poses_to_file():
    """Save all poses to JSON file."""
    filepath = Path(__file__).parent / "saved_poses.json"
    with open(filepath, 'w') as f:
        json.dump(saved_poses, f, indent=2)

def load_poses_from_file():
    """Load poses from JSON file."""
    global saved_poses
    filepath = Path(__file__).parent / "saved_poses.json"
    if filepath.exists():
        try:
            with open(filepath, 'r') as f:
                saved_poses = json.load(f)
        except:
            saved_poses = {}
    # Add some preset poses (body pose format)
    if not saved_poses:
        saved_poses = {
            "Stand": {"roll": 0, "pitch": 0, "yaw": 0, "height": 0},
            "Squat": {"roll": 0, "pitch": 0, "yaw": 0, "height": -0.7},
            "Tall": {"roll": 0, "pitch": 0, "yaw": 0, "height": 0.7},
            "Lean Left": {"roll": -0.6, "pitch": 0, "yaw": 0, "height": 0},
            "Lean Right": {"roll": 0.6, "pitch": 0, "yaw": 0, "height": 0},
            "Look Up": {"roll": 0, "pitch": 0.6, "yaw": 0, "height": 0},
            "Look Down": {"roll": 0, "pitch": -0.5, "yaw": 0, "height": 0},
            "Twist Left": {"roll": 0, "pitch": 0, "yaw": -0.5, "height": 0},
            "Twist Right": {"roll": 0, "pitch": 0, "yaw": 0.5, "height": 0},
        }

# ============== WEB SERVER ==============

HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Go1 Pose Builder</title>
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
        .header h1 { font-size: 28px; color: #00BCD4; }
        .header p { color: #888; font-size: 14px; margin-top: 5px; }

        .container {
            max-width: 900px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 280px;
            gap: 20px;
        }
        @media (max-width: 700px) { .container { grid-template-columns: 1fr; } }

        .panel {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
        }
        .panel h2 {
            font-size: 14px;
            color: #00BCD4;
            margin-bottom: 15px;
            text-transform: uppercase;
        }

        .pose-slider { margin-bottom: 20px; }
        .pose-slider label {
            display: flex;
            justify-content: space-between;
            font-size: 13px;
            color: #aaa;
            margin-bottom: 6px;
        }
        .pose-slider .slider-row { display: flex; align-items: center; gap: 10px; }
        .pose-slider input[type="range"] {
            flex: 1;
            height: 8px;
            border-radius: 4px;
            background: #333;
            outline: none;
            -webkit-appearance: none;
        }
        .pose-slider input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #00BCD4;
            cursor: pointer;
        }
        .pose-value {
            color: #00BCD4;
            font-weight: bold;
            min-width: 50px;
            text-align: right;
        }

        .btn {
            padding: 12px 20px;
            border: none;
            border-radius: 8px;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            margin: 4px;
        }
        .btn:hover { transform: scale(1.05); }
        .btn-primary { background: #00BCD4; color: white; }
        .btn-success { background: #4CAF50; color: white; }
        .btn-warning { background: #FF9800; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-purple { background: #9C27B0; color: white; }

        .btn-row { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 15px; }

        .pose-btn {
            padding: 10px 16px;
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 8px;
            font-size: 12px;
            cursor: pointer;
            background: rgba(255,255,255,0.1);
            color: white;
            transition: all 0.2s;
            margin: 3px;
        }
        .pose-btn:hover { border-color: #00BCD4; background: rgba(0,188,212,0.2); }

        .sequence-list {
            background: rgba(0,0,0,0.3);
            border-radius: 8px;
            padding: 10px;
            min-height: 80px;
            max-height: 150px;
            overflow-y: auto;
        }
        .sequence-item {
            display: flex;
            justify-content: space-between;
            padding: 6px 10px;
            margin-bottom: 4px;
            background: rgba(255,255,255,0.1);
            border-radius: 4px;
            font-size: 12px;
        }
        .sequence-item .remove { color: #f44336; cursor: pointer; }

        .save-input { display: flex; gap: 8px; margin-bottom: 10px; }
        .save-input input {
            flex: 1;
            padding: 10px 12px;
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 6px;
            background: rgba(0,0,0,0.3);
            color: white;
            font-size: 13px;
        }

        .status {
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 12px;
            display: inline-block;
            margin-bottom: 15px;
        }
        .status.connected { background: #4CAF50; }
        .status.disconnected { background: #f44336; }

        .quick-poses { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 10px; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Go1 Pose Builder</h1>
        <p>Adjust body pose sliders - works with real robot!</p>
    </div>

    <div class="container">
        <div class="main-panel">
            <div class="panel">
                <span class="status disconnected" id="connStatus">Disconnected</span>
                <div class="btn-row">
                    <button class="btn btn-success" onclick="connect()">Connect to Robot</button>
                    <button class="btn btn-warning" onclick="sendPose()">Send Pose</button>
                    <button class="btn btn-primary" onclick="resetPose()">Reset</button>
                </div>
            </div>

            <div class="panel">
                <h2>Body Pose Controls</h2>

                <div class="pose-slider">
                    <label><span>Roll (Lean Left/Right)</span><span class="pose-value" id="valRoll">0.00</span></label>
                    <div class="slider-row">
                        <span>L</span>
                        <input type="range" min="-1" max="1" step="0.05" value="0" id="sliderRoll" oninput="updatePose('roll')">
                        <span>R</span>
                    </div>
                </div>

                <div class="pose-slider">
                    <label><span>Pitch (Look Down/Up)</span><span class="pose-value" id="valPitch">0.00</span></label>
                    <div class="slider-row">
                        <span>D</span>
                        <input type="range" min="-1" max="1" step="0.05" value="0" id="sliderPitch" oninput="updatePose('pitch')">
                        <span>U</span>
                    </div>
                </div>

                <div class="pose-slider">
                    <label><span>Yaw (Twist Left/Right)</span><span class="pose-value" id="valYaw">0.00</span></label>
                    <div class="slider-row">
                        <span>L</span>
                        <input type="range" min="-1" max="1" step="0.05" value="0" id="sliderYaw" oninput="updatePose('yaw')">
                        <span>R</span>
                    </div>
                </div>

                <div class="pose-slider">
                    <label><span>Height (Squat/Extend)</span><span class="pose-value" id="valHeight">0.00</span></label>
                    <div class="slider-row">
                        <span>-</span>
                        <input type="range" min="-1" max="1" step="0.05" value="0" id="sliderHeight" oninput="updatePose('height')">
                        <span>+</span>
                    </div>
                </div>

                <h2>Quick Poses</h2>
                <div class="quick-poses">
                    <button class="pose-btn" onclick="quickPose(0,0,0,0)">Stand</button>
                    <button class="pose-btn" onclick="quickPose(0,0,0,-0.7)">Squat</button>
                    <button class="pose-btn" onclick="quickPose(0,0,0,0.7)">Tall</button>
                    <button class="pose-btn" onclick="quickPose(-0.6,0,0,0)">Lean L</button>
                    <button class="pose-btn" onclick="quickPose(0.6,0,0,0)">Lean R</button>
                    <button class="pose-btn" onclick="quickPose(0,0.6,0,0)">Look Up</button>
                    <button class="pose-btn" onclick="quickPose(0,-0.6,0,0)">Look Down</button>
                    <button class="pose-btn" onclick="quickPose(0,0,-0.5,0)">Twist L</button>
                    <button class="pose-btn" onclick="quickPose(0,0,0.5,0)">Twist R</button>
                </div>
            </div>

            <div class="panel">
                <h2>Front Leg Moves (Trump Arms!)</h2>
                <div class="quick-poses">
                    <button class="pose-btn" style="background:#E91E63;" onclick="doMove('straightHand')">Straight Hand (Y)</button>
                    <button class="pose-btn" style="background:#9C27B0;" onclick="doMove('frontUp')">Front Up</button>
                    <button class="pose-btn" style="background:#FF9800;" onclick="doMove('pray')">Pray</button>
                </div>
                <h2 style="margin-top:15px;">SDK Dances</h2>
                <div class="quick-poses">
                    <button class="pose-btn" style="background:#2196F3;" onclick="doMove('dance1')">Dance 1</button>
                    <button class="pose-btn" style="background:#2196F3;" onclick="doMove('dance2')">Dance 2</button>
                    <button class="pose-btn" style="background:#FF5722;" onclick="doMove('jumpYaw')">Jump Yaw</button>
                </div>
                <h2 style="margin-top:15px;">Recovery</h2>
                <div class="quick-poses">
                    <button class="pose-btn" style="background:#4CAF50;" onclick="doMove('stand')">Stand</button>
                    <button class="pose-btn" style="background:#4CAF50;" onclick="doMove('recover')">Recovery</button>
                    <button class="pose-btn" style="background:#607D8B;" onclick="doMove('standDown')">Sit</button>
                </div>
            </div>
        </div>

        <div class="side-panel">
            <div class="panel">
                <h2>Save Pose</h2>
                <div class="save-input">
                    <input type="text" id="poseName" placeholder="Pose name...">
                    <button class="btn btn-purple" onclick="savePose()">Save</button>
                </div>
            </div>

            <div class="panel">
                <h2>Saved Poses</h2>
                <div id="savedPoses"></div>
            </div>

            <div class="panel">
                <h2>Sequence</h2>
                <div class="btn-row">
                    <button class="btn btn-success" onclick="playSequence()">Play</button>
                    <button class="btn btn-danger" onclick="clearSequence()">Clear</button>
                </div>
                <div class="sequence-list" id="sequence"></div>
                <p style="font-size:11px;color:#666;margin-top:8px;">Double-click saved pose to add</p>
            </div>
        </div>
    </div>

    <script>
        let currentPose = {roll: 0, pitch: 0, yaw: 0, height: 0};
        let savedPoses = {};
        let sequence = [];

        function updatePose(axis) {
            const slider = document.getElementById('slider' + axis.charAt(0).toUpperCase() + axis.slice(1));
            const value = parseFloat(slider.value);
            currentPose[axis] = value;
            document.getElementById('val' + axis.charAt(0).toUpperCase() + axis.slice(1)).textContent = value.toFixed(2);
        }

        function setSliders(pose) {
            for (const axis of ['roll', 'pitch', 'yaw', 'height']) {
                const val = pose[axis] || 0;
                document.getElementById('slider' + axis.charAt(0).toUpperCase() + axis.slice(1)).value = val;
                document.getElementById('val' + axis.charAt(0).toUpperCase() + axis.slice(1)).textContent = val.toFixed(2);
                currentPose[axis] = val;
            }
        }

        function resetPose() {
            setSliders({roll: 0, pitch: 0, yaw: 0, height: 0});
            sendPose();
        }

        function quickPose(roll, pitch, yaw, height) {
            setSliders({roll, pitch, yaw, height});
            sendPose();
        }

        async function connect() {
            const res = await fetch('/api/connect', {method: 'POST'});
            const data = await res.json();
            const el = document.getElementById('connStatus');
            el.textContent = data.connected ? 'Connected to Go1' : 'Disconnected';
            el.className = 'status ' + (data.connected ? 'connected' : 'disconnected');
        }

        async function sendPose() {
            await fetch('/api/pose', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({pose: currentPose})
            });
        }

        async function doMove(moveName) {
            await fetch('/api/move/' + moveName, {method: 'POST'});
        }

        async function savePose() {
            const name = document.getElementById('poseName').value.trim();
            if (!name) { alert('Enter a pose name'); return; }
            await fetch('/api/save_pose', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({name: name, pose: {...currentPose}})
            });
            document.getElementById('poseName').value = '';
            loadSavedPoses();
        }

        async function loadSavedPoses() {
            const res = await fetch('/api/poses');
            savedPoses = await res.json();
            renderSavedPoses();
        }

        function renderSavedPoses() {
            const container = document.getElementById('savedPoses');
            container.innerHTML = '';
            for (const [name, pose] of Object.entries(savedPoses)) {
                const wrapper = document.createElement('div');
                wrapper.style.cssText = 'position:relative;display:inline-block;margin:3px;';

                const btn = document.createElement('button');
                btn.className = 'pose-btn';
                btn.textContent = name;
                btn.onclick = () => { setSliders(pose); sendPose(); };
                btn.ondblclick = () => addToSequence(name, pose);

                const delBtn = document.createElement('span');
                delBtn.textContent = 'x';
                delBtn.style.cssText = 'position:absolute;top:-6px;right:-6px;background:#f44336;color:white;border-radius:50%;width:16px;height:16px;font-size:10px;cursor:pointer;display:flex;align-items:center;justify-content:center;font-weight:bold;';
                delBtn.onclick = (e) => { e.stopPropagation(); deletePose(name); };

                wrapper.appendChild(btn);
                wrapper.appendChild(delBtn);
                container.appendChild(wrapper);
            }
        }

        async function deletePose(name) {
            if (!confirm('Delete "' + name + '"?')) return;
            await fetch('/api/delete_pose', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({name: name})
            });
            loadSavedPoses();
        }

        function addToSequence(name, pose) {
            sequence.push({name, pose: {...pose}});
            renderSequence();
        }

        function removeFromSequence(idx) {
            sequence.splice(idx, 1);
            renderSequence();
        }

        function renderSequence() {
            const container = document.getElementById('sequence');
            container.innerHTML = '';
            sequence.forEach((item, i) => {
                const div = document.createElement('div');
                div.className = 'sequence-item';
                div.innerHTML = '<span>' + (i+1) + '. ' + item.name + '</span><span class="remove" onclick="removeFromSequence(' + i + ')">X</span>';
                container.appendChild(div);
            });
        }

        function clearSequence() { sequence = []; renderSequence(); }

        async function playSequence() {
            for (const item of sequence) {
                setSliders(item.pose);
                await fetch('/api/pose', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({pose: item.pose})
                });
                await new Promise(r => setTimeout(r, 800));
            }
        }

        loadSavedPoses();
    </script>
</body>
</html>
"""

class PoseHandler(BaseHTTPRequestHandler):
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
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path == '/api/poses':
            self.send_json(saved_poses)
        else:
            self.send_error(404)

    def do_POST(self):
        global current_pose, connected

        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode() if content_length > 0 else '{}'

        try:
            data = json.loads(body) if body else {}
        except:
            data = {}

        if self.path == '/api/connect':
            success = connect_robot()
            self.send_json({'connected': success})

        elif self.path == '/api/pose':
            pose = data.get('pose', current_pose)
            current_pose.update(pose)
            # Send to real robot
            success = send_body_pose(pose)
            self.send_json({'success': success, 'pose': pose})

        elif self.path == '/api/save_pose':
            name = data.get('name', 'Unnamed')
            pose = data.get('pose', current_pose)
            save_pose(name, pose)
            self.send_json({'success': True})

        elif self.path == '/api/delete_pose':
            name = data.get('name')
            delete_pose(name)
            self.send_json({'success': True})

        elif self.path.startswith('/api/move/'):
            move_name = self.path.split('/')[-1]
            success = execute_move(move_name)
            self.send_json({'success': success})

        else:
            self.send_error(404)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


def main():
    load_poses_from_file()

    port = 8892
    server = HTTPServer(('', port), PoseHandler)

    print("\n" + "=" * 50)
    print("  Go1 Pose Builder")
    print("=" * 50)
    print(f"\n  Open: http://localhost:{port}")
    print("\n  - Adjust sliders to create poses")
    print("  - Click 'Save' to store a pose")
    print("  - Click pose button to load & send")
    print("  - Double-click to add to sequence")
    print("  - Click 'Play' to run sequence")
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
