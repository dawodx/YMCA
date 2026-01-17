#!/usr/bin/env python3
"""
Go1 Pose Builder - Custom Joint Control via Low-Level UDP

Create custom poses by adjusting individual joint positions.
Save poses and play them back as choreography.

Run with: python mo_simulation/go1_pose_builder.py
Open: http://localhost:8892
"""

import time
import json
import struct
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import webbrowser
from pathlib import Path

# Go1 Low-Level UDP Config
GO1_IP = "192.168.12.1"
LOW_LEVEL_PORT = 8007  # Low-level control port

# Joint limits (radians) - approximate safe ranges
JOINT_LIMITS = {
    'hip': (-0.8, 0.8),
    'thigh': (-0.5, 1.5),
    'calf': (-2.7, -0.8),
}

# Default standing pose (12 joints)
DEFAULT_POSE = [
    0.0, 0.9, -1.8,   # FR: hip, thigh, calf
    0.0, 0.9, -1.8,   # FL
    0.0, 0.9, -1.8,   # RR
    0.0, 0.9, -1.8,   # RL
]

# Global state
current_pose = DEFAULT_POSE.copy()
saved_poses = {}
sequence = []
udp_socket = None
connected = False

# Joint names for display
JOINT_NAMES = [
    "FR Hip", "FR Thigh", "FR Calf",
    "FL Hip", "FL Thigh", "FL Calf",
    "RR Hip", "RR Thigh", "RR Calf",
    "RL Hip", "RL Thigh", "RL Calf",
]

# ============== UDP LOW-LEVEL CONTROL ==============

def connect_udp():
    """Connect to Go1 low-level UDP interface."""
    global udp_socket, connected
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.settimeout(1.0)
        connected = True
        print(f"UDP connected to {GO1_IP}:{LOW_LEVEL_PORT}")
        return True
    except Exception as e:
        print(f"UDP connection failed: {e}")
        connected = False
        return False

def send_joint_positions(positions):
    """
    Send joint positions to Go1 via low-level UDP.
    Note: This requires the robot to be in low-level mode.
    """
    global udp_socket, connected

    if not connected or not udp_socket:
        return False

    try:
        # Build LowCmd packet (simplified)
        # Real implementation needs proper CRC and structure
        # This is a placeholder - actual protocol is more complex
        cmd_data = struct.pack('<12f', *positions)
        udp_socket.sendto(cmd_data, (GO1_IP, LOW_LEVEL_PORT))
        return True
    except Exception as e:
        print(f"Send failed: {e}")
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
        with open(filepath, 'r') as f:
            saved_poses = json.load(f)
    # Add some preset poses
    if not saved_poses:
        saved_poses = {
            "Stand": [0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8],
            "Sit": [0, 1.4, -2.6, 0, 1.4, -2.6, 0, 1.4, -2.6, 0, 1.4, -2.6],
            "Tall": [0, 0.5, -1.0, 0, 0.5, -1.0, 0, 0.5, -1.0, 0, 0.5, -1.0],
            "Wide": [0.3, 0.8, -1.6, -0.3, 0.8, -1.6, 0.3, 0.8, -1.6, -0.3, 0.8, -1.6],
            "Lean Left": [0.2, 1.0, -1.9, -0.2, 0.7, -1.5, 0.2, 1.0, -1.9, -0.2, 0.7, -1.5],
            "Lean Right": [-0.2, 0.7, -1.5, 0.2, 1.0, -1.9, -0.2, 0.7, -1.5, 0.2, 1.0, -1.9],
            "Front Up": [0, 0.5, -1.0, 0, 0.5, -1.0, 0, 1.2, -2.2, 0, 1.2, -2.2],
            "Back Up": [0, 1.2, -2.2, 0, 1.2, -2.2, 0, 0.5, -1.0, 0, 0.5, -1.0],
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
        .header {
            text-align: center;
            margin-bottom: 20px;
        }
        .header h1 { font-size: 28px; color: #00BCD4; }
        .header p { color: #888; font-size: 14px; margin-top: 5px; }

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
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
        }
        .panel h2 {
            font-size: 14px;
            color: #00BCD4;
            margin-bottom: 15px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        .legs-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        .leg-panel {
            background: rgba(0,0,0,0.2);
            border-radius: 8px;
            padding: 12px;
        }
        .leg-panel h3 {
            font-size: 12px;
            color: #ffd700;
            margin-bottom: 10px;
        }

        .joint-slider {
            margin-bottom: 12px;
        }
        .joint-slider label {
            display: flex;
            justify-content: space-between;
            font-size: 11px;
            color: #aaa;
            margin-bottom: 4px;
        }
        .joint-slider input[type="range"] {
            width: 100%;
            height: 6px;
            border-radius: 3px;
            background: #333;
            outline: none;
            -webkit-appearance: none;
        }
        .joint-slider input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 16px;
            height: 16px;
            border-radius: 50%;
            background: #00BCD4;
            cursor: pointer;
        }
        .joint-value {
            color: #00BCD4;
            font-weight: bold;
        }

        .btn {
            padding: 10px 16px;
            border: none;
            border-radius: 6px;
            font-size: 12px;
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

        .btn-row {
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
            margin-bottom: 15px;
        }

        .pose-btn {
            padding: 8px 14px;
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 6px;
            font-size: 11px;
            cursor: pointer;
            background: rgba(255,255,255,0.1);
            color: white;
            transition: all 0.2s;
        }
        .pose-btn:hover {
            border-color: #00BCD4;
            background: rgba(0,188,212,0.2);
        }

        .sequence-list {
            background: rgba(0,0,0,0.3);
            border-radius: 8px;
            padding: 10px;
            min-height: 100px;
            max-height: 200px;
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
        .sequence-item .remove {
            color: #f44336;
            cursor: pointer;
        }

        .save-input {
            display: flex;
            gap: 8px;
            margin-bottom: 10px;
        }
        .save-input input {
            flex: 1;
            padding: 8px 12px;
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 6px;
            background: rgba(0,0,0,0.3);
            color: white;
            font-size: 12px;
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

        .robot-diagram {
            text-align: center;
            padding: 20px;
            font-family: monospace;
            font-size: 12px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Go1 Pose Builder</h1>
        <p>Adjust joint sliders to create custom poses</p>
    </div>

    <div class="container">
        <div class="main-panel">
            <div class="panel">
                <span class="status disconnected" id="connStatus">Disconnected</span>
                <div class="btn-row">
                    <button class="btn btn-success" onclick="connect()">Connect</button>
                    <button class="btn btn-warning" onclick="sendPose()">Send to Robot</button>
                    <button class="btn btn-primary" onclick="resetPose()">Reset to Stand</button>
                </div>
            </div>

            <div class="panel">
                <h2>Joint Controls</h2>
                <div class="legs-grid">
                    <div class="leg-panel">
                        <h3>Front Right (FR)</h3>
                        <div class="joint-slider">
                            <label><span>Hip</span><span class="joint-value" id="val0">0.00</span></label>
                            <input type="range" min="-0.8" max="0.8" step="0.01" value="0" id="joint0" oninput="updateJoint(0)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Thigh</span><span class="joint-value" id="val1">0.90</span></label>
                            <input type="range" min="-0.5" max="1.5" step="0.01" value="0.9" id="joint1" oninput="updateJoint(1)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Calf</span><span class="joint-value" id="val2">-1.80</span></label>
                            <input type="range" min="-2.7" max="-0.8" step="0.01" value="-1.8" id="joint2" oninput="updateJoint(2)">
                        </div>
                    </div>

                    <div class="leg-panel">
                        <h3>Front Left (FL)</h3>
                        <div class="joint-slider">
                            <label><span>Hip</span><span class="joint-value" id="val3">0.00</span></label>
                            <input type="range" min="-0.8" max="0.8" step="0.01" value="0" id="joint3" oninput="updateJoint(3)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Thigh</span><span class="joint-value" id="val4">0.90</span></label>
                            <input type="range" min="-0.5" max="1.5" step="0.01" value="0.9" id="joint4" oninput="updateJoint(4)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Calf</span><span class="joint-value" id="val5">-1.80</span></label>
                            <input type="range" min="-2.7" max="-0.8" step="0.01" value="-1.8" id="joint5" oninput="updateJoint(5)">
                        </div>
                    </div>

                    <div class="leg-panel">
                        <h3>Rear Right (RR)</h3>
                        <div class="joint-slider">
                            <label><span>Hip</span><span class="joint-value" id="val6">0.00</span></label>
                            <input type="range" min="-0.8" max="0.8" step="0.01" value="0" id="joint6" oninput="updateJoint(6)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Thigh</span><span class="joint-value" id="val7">0.90</span></label>
                            <input type="range" min="-0.5" max="1.5" step="0.01" value="0.9" id="joint7" oninput="updateJoint(7)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Calf</span><span class="joint-value" id="val8">-1.80</span></label>
                            <input type="range" min="-2.7" max="-0.8" step="0.01" value="-1.8" id="joint8" oninput="updateJoint(8)">
                        </div>
                    </div>

                    <div class="leg-panel">
                        <h3>Rear Left (RL)</h3>
                        <div class="joint-slider">
                            <label><span>Hip</span><span class="joint-value" id="val9">0.00</span></label>
                            <input type="range" min="-0.8" max="0.8" step="0.01" value="0" id="joint9" oninput="updateJoint(9)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Thigh</span><span class="joint-value" id="val10">0.90</span></label>
                            <input type="range" min="-0.5" max="1.5" step="0.01" value="0.9" id="joint10" oninput="updateJoint(10)">
                        </div>
                        <div class="joint-slider">
                            <label><span>Calf</span><span class="joint-value" id="val11">-1.80</span></label>
                            <input type="range" min="-2.7" max="-0.8" step="0.01" value="-1.8" id="joint11" oninput="updateJoint(11)">
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="side-panel">
            <div class="panel">
                <h2>Save Current Pose</h2>
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
            </div>

            <div class="panel">
                <div class="robot-diagram">
                    <pre>
    [FL]----[FR]
      |      |
      |  ^^  |
      |      |
    [RL]----[RR]
                    </pre>
                    <p>Front = Head direction</p>
                </div>
            </div>
        </div>
    </div>

    <script>
        let currentPose = [0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8];
        let savedPoses = {};
        let sequence = [];

        function updateJoint(idx) {
            const slider = document.getElementById('joint' + idx);
            const value = parseFloat(slider.value);
            currentPose[idx] = value;
            document.getElementById('val' + idx).textContent = value.toFixed(2);
        }

        function setSliders(pose) {
            for (let i = 0; i < 12; i++) {
                document.getElementById('joint' + i).value = pose[i];
                document.getElementById('val' + i).textContent = pose[i].toFixed(2);
                currentPose[i] = pose[i];
            }
        }

        function resetPose() {
            setSliders([0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8]);
        }

        async function connect() {
            const res = await fetch('/api/connect', {method: 'POST'});
            const data = await res.json();
            const el = document.getElementById('connStatus');
            el.textContent = data.connected ? 'Connected' : 'Disconnected';
            el.className = 'status ' + (data.connected ? 'connected' : 'disconnected');
        }

        async function sendPose() {
            await fetch('/api/pose', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({pose: currentPose})
            });
        }

        async function savePose() {
            const name = document.getElementById('poseName').value.trim();
            if (!name) { alert('Enter a pose name'); return; }

            await fetch('/api/save_pose', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({name: name, pose: currentPose.slice()})
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
                const btn = document.createElement('button');
                btn.className = 'pose-btn';
                btn.textContent = name;
                btn.onclick = () => {
                    setSliders(pose);
                    sendPose();
                };
                btn.ondblclick = () => addToSequence(name, pose);
                container.appendChild(btn);
            }
        }

        function addToSequence(name, pose) {
            sequence.push({name, pose: pose.slice()});
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

        function clearSequence() {
            sequence = [];
            renderSequence();
        }

        async function playSequence() {
            for (const item of sequence) {
                setSliders(item.pose);
                await fetch('/api/pose', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({pose: item.pose})
                });
                await new Promise(r => setTimeout(r, 1000));
            }
        }

        // Load saved poses on start
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
            success = connect_udp()
            self.send_json({'connected': success})

        elif self.path == '/api/pose':
            pose = data.get('pose', DEFAULT_POSE)
            current_pose = pose
            # Try to send to robot
            if connected:
                send_joint_positions(pose)
            self.send_json({'success': True, 'pose': pose})

        elif self.path == '/api/save_pose':
            name = data.get('name', 'Unnamed')
            pose = data.get('pose', current_pose)
            save_pose(name, pose)
            self.send_json({'success': True})

        elif self.path == '/api/delete_pose':
            name = data.get('name')
            delete_pose(name)
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
