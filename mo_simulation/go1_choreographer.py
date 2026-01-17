#!/usr/bin/env python3
"""
Go1 Choreography Editor - Visual timeline for dance moves

Features:
- Timeline visualization with song waveform
- Draggable section boundaries
- Draggable move markers
- Zoom in/out like Adobe Premiere
- Click timeline to add moves
- Play song and robot moves together
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
    print("WARNING: go1pylib not installed - robot control disabled")
    Go1 = None
    Go1Mode = None

SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent
CHOREO_DIR = SCRIPT_DIR / "choreography"
MUSIC_DIR = PROJECT_ROOT / "alex_music" / "music"

CHOREO_DIR.mkdir(exist_ok=True)

robot = None
connected = False

DEFAULT_CHOREOGRAPHY = {
    "name": "YMCA Dance",
    "song": "ymca_music.mp3",
    "bpm": 129,
    "duration_ms": 290000,
    "sections": [
        {"name": "INTRO", "start_ms": 0, "end_ms": 26000, "color": "#FFD700"},
        {"name": "YOUNG MAN", "start_ms": 26000, "end_ms": 55000, "color": "#4CAF50"},
        {"name": "CHORUS", "start_ms": 55000, "end_ms": 85000, "color": "#E91E63"},
        {"name": "YOUNG MAN 2", "start_ms": 85000, "end_ms": 115000, "color": "#4CAF50"},
        {"name": "CHORUS 2", "start_ms": 115000, "end_ms": 145000, "color": "#E91E63"},
        {"name": "YOUNG MAN 3", "start_ms": 145000, "end_ms": 175000, "color": "#4CAF50"},
        {"name": "CHORUS 3", "start_ms": 175000, "end_ms": 290000, "color": "#E91E63"},
    ],
    "moves": [
        {"time_ms": 0, "cmd": "stand", "params": {}, "label": "Stand Ready"},
        {"time_ms": 500, "cmd": "ledYellow", "params": {}, "label": "LED Yellow"},
        {"time_ms": 26000, "cmd": "dance1", "params": {}, "label": "Dance 1"},
        {"time_ms": 55000, "cmd": "stand", "params": {}, "label": "Stand"},
        {"time_ms": 56000, "cmd": "lookUp", "params": {"intensity": 0.6, "duration": 400}, "label": "Y"},
        {"time_ms": 57000, "cmd": "lookDown", "params": {"intensity": 0.5, "duration": 400}, "label": "M"},
        {"time_ms": 58000, "cmd": "squat", "params": {"intensity": 0.5, "duration": 400}, "label": "C"},
        {"time_ms": 59000, "cmd": "extend", "params": {"intensity": 0.5, "duration": 400}, "label": "A"},
    ]
}

AVAILABLE_COMMANDS = {
    "Modes": ["stand", "standUp", "standDown", "walk", "damping", "recoverStand"],
    "Poses": ["lookUp", "lookDown", "leanLeft", "leanRight", "twistLeft", "twistRight", "squat", "extend"],
    "Dance": ["dance1", "dance2", "straightHand1", "jumpYaw"],
    "LED": ["ledRed", "ledGreen", "ledBlue", "ledYellow", "ledPink", "ledCyan", "ledWhite", "ledOff"],
    "YMCA": ["ymcaY", "ymcaM", "ymcaC", "ymcaA", "ymcaDance"],
}


def get_choreography_files():
    return sorted([f.stem for f in CHOREO_DIR.glob("*.json")])


def save_choreography(name, data):
    with open(CHOREO_DIR / f"{name}.json", 'w') as f:
        json.dump(data, f, indent=2)


def load_choreography(name):
    path = CHOREO_DIR / f"{name}.json"
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return None


def connect_robot():
    global robot, connected
    if Go1 is None:
        return False
    try:
        robot = Go1()
        robot.init()
        connected = True
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        return False


def execute_move(cmd, params):
    global robot, connected
    if not robot or not connected:
        return False
    try:
        mode_map = {
            "stand": Go1Mode.STAND, "standUp": Go1Mode.STAND_UP, "standDown": Go1Mode.STAND_DOWN,
            "walk": Go1Mode.WALK, "damping": Go1Mode.DAMPING, "recoverStand": Go1Mode.RECOVER_STAND,
            "dance1": Go1Mode.DANCE1, "dance2": Go1Mode.DANCE2, "straightHand1": Go1Mode.STRAIGHT_HAND1,
        }
        if cmd in mode_map:
            robot.set_mode(mode_map[cmd])
            return True

        led_map = {
            "ledRed": (255,0,0), "ledGreen": (0,255,0), "ledBlue": (0,0,255),
            "ledYellow": (255,255,0), "ledPink": (255,100,150), "ledCyan": (0,255,255),
            "ledWhite": (255,255,255), "ledOff": (0,0,0),
        }
        if cmd in led_map:
            robot.set_led_color(*led_map[cmd])
            return True

        intensity = params.get("intensity", 0.5)
        duration = int(params.get("duration", 300))
        loop = asyncio.new_event_loop()
        robot.set_mode(Go1Mode.STAND)
        time.sleep(0.1)

        pose_funcs = {
            "lookUp": robot.look_up, "lookDown": robot.look_down,
            "leanLeft": robot.lean_left, "leanRight": robot.lean_right,
            "twistLeft": robot.twist_left, "twistRight": robot.twist_right,
            "squat": robot.squat_down, "extend": robot.extend_up,
        }
        if cmd in pose_funcs:
            loop.run_until_complete(pose_funcs[cmd](intensity, duration))
        loop.close()
        return True
    except Exception as e:
        print(f"Execute error: {e}")
        return False


HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Go1 Choreography Editor</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #0d0d1a;
            color: white;
            overflow-x: hidden;
        }
        .header {
            background: rgba(0,0,0,0.5);
            padding: 12px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid #333;
        }
        .header h1 { font-size: 20px; color: #E91E63; }
        .btn {
            padding: 8px 16px;
            border: none;
            border-radius: 6px;
            font-size: 12px;
            font-weight: 600;
            cursor: pointer;
        }
        .btn:hover { opacity: 0.85; }
        .btn-pink { background: #E91E63; color: white; }
        .btn-green { background: #4CAF50; color: white; }
        .btn-blue { background: #2196F3; color: white; }
        .btn-gray { background: #555; color: white; }
        .btn-orange { background: #FF9800; color: white; }

        .main { display: flex; height: calc(100vh - 50px); }
        .timeline-panel { flex: 1; padding: 15px; overflow-y: auto; }
        .sidebar { width: 320px; background: rgba(0,0,0,0.3); border-left: 1px solid #333; display: flex; flex-direction: column; }

        .panel { background: rgba(255,255,255,0.03); border-radius: 8px; padding: 12px; margin-bottom: 12px; }
        .panel h3 { color: #E91E63; font-size: 14px; margin-bottom: 10px; }

        .row { display: flex; gap: 10px; align-items: center; margin-bottom: 8px; }
        .row label { color: #888; font-size: 12px; min-width: 60px; }
        .row input, .row select {
            background: #1a1a2e;
            border: 1px solid #333;
            border-radius: 4px;
            padding: 6px 10px;
            color: white;
            font-size: 13px;
        }
        .row input[type="text"], .row input[type="number"] { flex: 1; }

        /* Playback */
        .playback { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
        .time-display { font-family: monospace; font-size: 22px; color: #E91E63; }
        .progress-bar {
            flex: 1;
            min-width: 200px;
            height: 6px;
            background: #333;
            border-radius: 3px;
            cursor: pointer;
            margin-top: 10px;
        }
        .progress-fill { height: 100%; background: #E91E63; border-radius: 3px; width: 0%; }

        /* Zoom controls */
        .zoom-controls { display: flex; gap: 8px; align-items: center; margin-bottom: 10px; }
        .zoom-controls span { color: #888; font-size: 12px; }
        .zoom-btn { width: 30px; height: 30px; font-size: 16px; padding: 0; }

        /* Timeline container */
        .timeline-wrapper {
            position: relative;
            overflow-x: auto;
            overflow-y: hidden;
            background: #0a0a12;
            border-radius: 8px;
            margin-bottom: 10px;
        }
        .timeline-container {
            position: relative;
            height: 140px;
            min-width: 100%;
        }
        .waveform-canvas {
            position: absolute;
            top: 30px;
            left: 0;
            width: 100%;
            height: 110px;
        }

        /* Sections */
        .sections-layer {
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 30px;
            display: flex;
        }
        .section {
            height: 100%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 10px;
            font-weight: bold;
            color: rgba(0,0,0,0.8);
            position: relative;
            cursor: pointer;
            user-select: none;
        }
        .section-handle {
            position: absolute;
            right: -4px;
            top: 0;
            width: 8px;
            height: 100%;
            cursor: ew-resize;
            background: rgba(255,255,255,0.3);
            opacity: 0;
            z-index: 5;
        }
        .section:hover .section-handle { opacity: 1; }

        /* Moves layer */
        .moves-layer {
            position: absolute;
            top: 30px;
            left: 0;
            right: 0;
            bottom: 0;
        }
        .move-marker {
            position: absolute;
            top: 10px;
            bottom: 10px;
            width: 6px;
            background: linear-gradient(180deg, #4CAF50, #2E7D32);
            border-radius: 3px;
            cursor: grab;
            z-index: 10;
            transition: width 0.1s, box-shadow 0.1s;
        }
        .move-marker:hover, .move-marker.dragging {
            width: 10px;
            box-shadow: 0 0 10px rgba(76,175,80,0.8);
        }
        .move-marker.selected {
            background: linear-gradient(180deg, #E91E63, #C2185B);
            box-shadow: 0 0 12px rgba(233,30,99,0.8);
        }
        .move-marker .label {
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(0,0,0,0.8);
            padding: 2px 6px;
            border-radius: 3px;
            font-size: 9px;
            white-space: nowrap;
            pointer-events: none;
            opacity: 0;
        }
        .move-marker:hover .label { opacity: 1; }

        /* Debug flags */
        .flag-marker {
            position: absolute;
            top: 0;
            bottom: 0;
            width: 2px;
            background: #FF9800;
            z-index: 15;
            cursor: pointer;
        }
        .flag-marker::after {
            content: '';
            position: absolute;
            top: 0;
            left: -4px;
            border-left: 5px solid transparent;
            border-right: 5px solid transparent;
            border-top: 8px solid #FF9800;
        }

        /* Playhead */
        .playhead {
            position: absolute;
            top: 0;
            width: 2px;
            height: 100%;
            background: #fff;
            z-index: 20;
            pointer-events: none;
        }
        .playhead::before {
            content: '';
            position: absolute;
            top: 0;
            left: -5px;
            border-left: 6px solid transparent;
            border-right: 6px solid transparent;
            border-top: 8px solid #fff;
        }

        /* Time ruler */
        .time-ruler {
            height: 20px;
            background: #151520;
            display: flex;
            font-size: 9px;
            color: #666;
            border-top: 1px solid #333;
        }
        .time-tick {
            border-left: 1px solid #333;
            padding-left: 4px;
            flex-shrink: 0;
        }

        /* Add move panel */
        .add-panel { background: rgba(76,175,80,0.1); border: 1px solid #4CAF50; }
        .cmd-cats { display: flex; gap: 5px; flex-wrap: wrap; margin-bottom: 8px; }
        .cmd-cat {
            padding: 4px 10px;
            background: #333;
            border-radius: 10px;
            font-size: 11px;
            cursor: pointer;
        }
        .cmd-cat.active { background: #4CAF50; }
        .cmd-btns { display: flex; gap: 5px; flex-wrap: wrap; }
        .cmd-btn {
            padding: 5px 10px;
            background: rgba(255,255,255,0.1);
            border: 1px solid #444;
            border-radius: 4px;
            color: white;
            font-size: 11px;
            cursor: pointer;
        }
        .cmd-btn:hover { border-color: #4CAF50; }

        /* Move list */
        .move-list { flex: 1; overflow-y: auto; padding: 10px; }
        .move-item {
            background: rgba(255,255,255,0.05);
            border-radius: 6px;
            padding: 8px;
            margin-bottom: 6px;
            cursor: pointer;
            border: 1px solid transparent;
        }
        .move-item:hover { border-color: #E91E63; }
        .move-item.selected { border-color: #E91E63; background: rgba(233,30,99,0.1); }
        .move-item.current { border-color: #4CAF50; background: rgba(76,175,80,0.2); box-shadow: 0 0 10px rgba(76,175,80,0.3); }
        .move-item.next { border-color: #FF9800; background: rgba(255,152,0,0.1); }
        .move-item .time { color: #E91E63; font-family: monospace; font-size: 11px; }
        .move-item .cmd { color: #4CAF50; font-weight: bold; font-size: 13px; }
        .move-item .lbl { color: #888; font-size: 10px; }
        .move-item .del { float: right; color: #f44336; cursor: pointer; }

        /* Editor */
        .move-editor { background: rgba(0,0,0,0.3); border-top: 1px solid #333; padding: 12px; }
        .editor-row { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
        .editor-row label { color: #888; font-size: 11px; min-width: 60px; }
        .editor-row input { flex: 1; background: #1a1a2e; border: 1px solid #333; border-radius: 4px; padding: 6px; color: white; }
        .editor-actions { display: flex; gap: 8px; margin-top: 10px; }

        /* Modal */
        .modal-overlay {
            display: none;
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.7);
            z-index: 100;
            justify-content: center;
            align-items: center;
        }
        .modal-overlay.show { display: flex; }
        .modal {
            background: #1a1a2e;
            border: 1px solid #E91E63;
            border-radius: 12px;
            padding: 20px;
            min-width: 300px;
            max-width: 400px;
        }
        .modal h2 { color: #E91E63; font-size: 16px; margin-bottom: 15px; }
        .modal-input {
            width: 100%;
            background: #0d0d1a;
            border: 1px solid #333;
            border-radius: 6px;
            padding: 10px;
            color: white;
            font-size: 14px;
            margin-bottom: 10px;
        }
        .modal-list {
            max-height: 200px;
            overflow-y: auto;
            margin-bottom: 15px;
        }
        .modal-item {
            padding: 10px 12px;
            background: rgba(255,255,255,0.05);
            border-radius: 6px;
            margin-bottom: 5px;
            cursor: pointer;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .modal-item:hover { background: rgba(233,30,99,0.2); }
        .modal-item.selected { background: rgba(233,30,99,0.3); border: 1px solid #E91E63; }
        .modal-actions { display: flex; gap: 10px; justify-content: flex-end; }
        .modal-empty { color: #666; text-align: center; padding: 20px; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Go1 Choreography Editor</h1>
        <div style="display:flex;gap:10px;align-items:center">
            <span id="status" style="padding:4px 10px;border-radius:10px;font-size:11px;background:#f44336">Disconnected</span>
            <button class="btn btn-green" onclick="connect()">Connect</button>
            <button class="btn btn-blue" onclick="loadDialog()">Load</button>
            <button class="btn btn-orange" onclick="saveDialog()">Save</button>
        </div>
    </div>

    <div class="main">
        <div class="timeline-panel">
            <div class="panel">
                <h3>Song Settings</h3>
                <div class="row">
                    <label>Name:</label>
                    <input type="text" id="choreoName" value="YMCA Dance">
                </div>
                <div class="row">
                    <label>BPM:</label>
                    <input type="number" id="bpm" value="129" style="width:70px">
                    <label>Duration:</label>
                    <input type="number" id="duration" value="290000" style="width:100px">
                    <span style="color:#666">ms</span>
                </div>
            </div>

            <div class="panel">
                <div class="playback">
                    <button class="btn btn-pink" onclick="togglePlay()" id="playBtn">Play</button>
                    <button class="btn btn-gray" onclick="stopPlay()">Stop</button>
                    <button class="btn btn-orange" onclick="toggleDebug()" id="debugBtn">Debug Mode</button>
                    <span class="time-display" id="curTime">0:00.0</span>
                    <span style="color:#666">/</span>
                    <span id="totTime" style="color:#666">0:00</span>
                    <label style="color:#888;font-size:11px;margin-left:10px">
                        <input type="checkbox" id="mute"> Mute
                    </label>
                    <label style="color:#888;font-size:11px;margin-left:10px">
                        <input type="checkbox" id="execRobot" checked> Execute Robot
                    </label>
                </div>
                <div class="progress-bar" onclick="seekBar(event)">
                    <div class="progress-fill" id="progFill"></div>
                </div>
                <div id="debugInfo" style="display:none;margin-top:10px;padding:10px;background:rgba(255,152,0,0.2);border-radius:6px;font-size:12px">
                    <div style="margin-bottom:8px">
                        <strong style="color:#FF9800">Debug Mode ON</strong> - Press SPACE while playing to flag current time. Flags saved with main Save button.
                    </div>
                    <span id="flagCount" style="color:#4CAF50">0 flags</span>
                    <button class="btn btn-gray" style="margin-left:10px;padding:4px 8px;font-size:10px" onclick="clearFlags()">Clear Flags</button>
                </div>
            </div>

            <div class="panel">
                <div class="zoom-controls">
                    <span>Zoom:</span>
                    <button class="btn btn-gray zoom-btn" onclick="zoomOut()">-</button>
                    <span id="zoomLevel">100%</span>
                    <button class="btn btn-gray zoom-btn" onclick="zoomIn()">+</button>
                    <button class="btn btn-gray" onclick="zoomFit()">Fit</button>
                </div>

                <audio id="audio" preload="auto">
                    <source src="/audio/ymca_music.mp3" type="audio/mpeg">
                </audio>

                <div class="timeline-wrapper" id="wrapper">
                    <div class="timeline-container" id="container" onclick="timelineClick(event)">
                        <canvas id="waveform" class="waveform-canvas"></canvas>
                        <div class="sections-layer" id="sections"></div>
                        <div class="moves-layer" id="moves"></div>
                        <div class="playhead" id="playhead"></div>
                    </div>
                    <div class="time-ruler" id="ruler"></div>
                </div>
            </div>

            <div class="panel add-panel">
                <h3>Add Move at <span id="addTimeDisp">0:00.0</span></h3>
                <div class="row">
                    <label>Time (ms):</label>
                    <input type="number" id="addTime" value="0" oninput="updAddDisp()">
                    <button class="btn btn-gray" onclick="setAddCurrent()">Current</button>
                </div>
                <div class="cmd-cats" id="cats"></div>
                <div class="cmd-btns" id="cmds"></div>
            </div>
        </div>

        <div class="sidebar">
            <div class="move-list" id="moveList"></div>
            <div class="move-editor" id="editor" style="display:none">
                <h3 style="color:#E91E63;font-size:13px;margin-bottom:10px">Edit Move</h3>
                <div class="editor-row">
                    <label>Time:</label>
                    <input type="number" id="editTime" oninput="liveUpdate()">
                </div>
                <div class="editor-row">
                    <label>Label:</label>
                    <input type="text" id="editLabel" oninput="liveUpdate()">
                </div>
                <div id="params"></div>
                <div class="editor-actions">
                    <button class="btn btn-green" onclick="testMove()">Test</button>
                    <button class="btn btn-gray" onclick="dupeMove()">Duplicate</button>
                    <button class="btn btn-pink" onclick="delSelected()">Delete</button>
                </div>
            </div>
        </div>
    </div>

    <!-- Save Modal -->
    <div class="modal-overlay" id="saveModal" onclick="closeSaveModal(event)">
        <div class="modal" onclick="event.stopPropagation()">
            <h2>Save Choreography</h2>
            <input type="text" class="modal-input" id="saveName" placeholder="Enter name...">
            <div class="modal-actions">
                <button class="btn btn-gray" onclick="closeSaveModal()">Cancel</button>
                <button class="btn btn-green" onclick="doSave()">Save</button>
            </div>
        </div>
    </div>

    <!-- Load Modal -->
    <div class="modal-overlay" id="loadModal" onclick="closeLoadModal(event)">
        <div class="modal" onclick="event.stopPropagation()">
            <h2>Load Choreography</h2>
            <div class="modal-list" id="loadList"></div>
            <div class="modal-actions">
                <button class="btn btn-gray" onclick="closeLoadModal()">Cancel</button>
                <button class="btn btn-blue" onclick="doLoad()" id="loadBtn" disabled>Load</button>
            </div>
        </div>
    </div>

<script>
let choreo = CHOREO_JSON;
const CMDS = COMMANDS_JSON;
let selIdx = -1;
let zoom = 1;
let playing = false;
let interval = null;
let curTime = 0;
let dragging = null;
let dragType = null;
let activeCat = 'Poses';
let debugMode = false;
let flags = [];

const audio = document.getElementById('audio');
const container = document.getElementById('container');
const wrapper = document.getElementById('wrapper');

// Debug mode - flag timestamps while playing
function toggleDebug() {
    debugMode = !debugMode;
    document.getElementById('debugBtn').style.background = debugMode ? '#4CAF50' : '#FF9800';
    document.getElementById('debugBtn').textContent = debugMode ? 'Debug ON' : 'Debug Mode';
    document.getElementById('debugInfo').style.display = debugMode ? 'block' : 'none';
    renderFlags();
}

function addFlag(ms) {
    if (!flags.includes(ms)) {
        flags.push(ms);
        flags.sort((a, b) => a - b);
        renderFlags();
        document.getElementById('flagCount').textContent = flags.length + ' flags';
    }
}

function clearFlags() {
    flags = [];
    renderFlags();
    document.getElementById('flagCount').textContent = '0 flags';
}

function renderFlags() {
    document.querySelectorAll('.flag-marker').forEach(f => f.remove());
    if (!flags.length) return;

    const dur = getDur();
    const w = container.offsetWidth;
    const movesLayer = document.getElementById('moves');

    flags.forEach((ms, i) => {
        const flag = document.createElement('div');
        flag.className = 'flag-marker';
        flag.style.left = (ms / dur * w) + 'px';
        flag.title = fmtTime(ms) + ' (click to add move, right-click to remove)';
        flag.onclick = (e) => {
            e.stopPropagation();
            document.getElementById('addTime').value = ms;
            updAddDisp();
        };
        flag.oncontextmenu = (e) => {
            e.preventDefault();
            flags.splice(i, 1);
            renderFlags();
            document.getElementById('flagCount').textContent = flags.length + ' flags';
        };
        movesLayer.appendChild(flag);
    });
}

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {
    // Space to flag in debug mode while playing
    if (e.code === 'Space' && debugMode && playing) {
        e.preventDefault();
        addFlag(curTime);
    }
    // Escape to close modals
    if (e.code === 'Escape') {
        closeSaveModal();
        closeLoadModal();
    }
    // Enter to submit save modal
    if (e.code === 'Enter' && document.getElementById('saveModal').classList.contains('show')) {
        doSave();
    }
});

// Init
function init() {
    renderSections();
    renderMoves();
    renderFlags();
    renderCats();
    renderCmds();
    renderRuler();
    loadWaveform();
    updTotTime();
}

function updTotTime() {
    const d = getDur();
    document.getElementById('totTime').textContent = fmtShort(d);
}

function getDur() {
    return parseInt(document.getElementById('duration').value) || 290000;
}

function fmtTime(ms) {
    const s = Math.floor(ms / 1000);
    return `${Math.floor(s/60)}:${(s%60).toString().padStart(2,'0')}.${Math.floor((ms%1000)/100)}`;
}

function fmtShort(ms) {
    const s = Math.floor(ms / 1000);
    return `${Math.floor(s/60)}:${(s%60).toString().padStart(2,'0')}`;
}

// Zoom
function zoomIn() { zoom = Math.min(zoom * 1.5, 10); applyZoom(); }
function zoomOut() { zoom = Math.max(zoom / 1.5, 0.5); applyZoom(); }
function zoomFit() { zoom = 1; applyZoom(); }

function applyZoom() {
    const w = wrapper.offsetWidth * zoom;
    container.style.width = w + 'px';
    document.getElementById('zoomLevel').textContent = Math.round(zoom * 100) + '%';
    renderSections();
    renderMoves();
    renderFlags();
    renderRuler();
    redrawWaveform();
}

// Waveform
let audioBuffer = null;

async function loadWaveform() {
    try {
        const res = await fetch('/audio/ymca_music.mp3');
        const buf = await res.arrayBuffer();
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        audioBuffer = await ctx.decodeAudioData(buf);
        redrawWaveform();
    } catch(e) {
        console.log('Waveform error:', e);
    }
}

function redrawWaveform() {
    const canvas = document.getElementById('waveform');
    const ctx = canvas.getContext('2d');
    const w = container.offsetWidth;
    const h = 110;

    canvas.width = w * 2;
    canvas.height = h * 2;
    canvas.style.width = w + 'px';
    ctx.scale(2, 2);

    ctx.fillStyle = '#0a0a12';
    ctx.fillRect(0, 0, w, h);

    if (!audioBuffer) return;

    const data = audioBuffer.getChannelData(0);
    const step = Math.ceil(data.length / w);
    const mid = h / 2;

    // Normalize - find max amplitude
    let maxAmp = 0;
    for (let i = 0; i < w; i++) {
        let min = 0, max = 0;
        for (let j = 0; j < step; j++) {
            const v = data[i * step + j] || 0;
            if (v < min) min = v;
            if (v > max) max = v;
        }
        const amp = max - min;
        if (amp > maxAmp) maxAmp = amp;
    }

    // Draw normalized waveform
    for (let i = 0; i < w; i++) {
        let min = 0, max = 0;
        for (let j = 0; j < step; j++) {
            const v = data[i * step + j] || 0;
            if (v < min) min = v;
            if (v > max) max = v;
        }

        // Normalize to use 80% of height
        const scale = (h * 0.4) / (maxAmp / 2);
        const y1 = mid + min * scale;
        const y2 = mid + max * scale;

        const grad = ctx.createLinearGradient(0, y1, 0, y2);
        grad.addColorStop(0, 'rgba(156,39,176,0.7)');
        grad.addColorStop(0.5, 'rgba(233,30,99,0.5)');
        grad.addColorStop(1, 'rgba(103,58,183,0.7)');

        ctx.fillStyle = grad;
        ctx.fillRect(i, y1, 1, y2 - y1);
    }

    // Center line
    ctx.strokeStyle = 'rgba(255,255,255,0.1)';
    ctx.beginPath();
    ctx.moveTo(0, mid);
    ctx.lineTo(w, mid);
    ctx.stroke();
}

// Sections
function renderSections() {
    const el = document.getElementById('sections');
    const dur = getDur();
    const w = container.offsetWidth;
    el.innerHTML = '';
    el.style.width = w + 'px';

    choreo.sections.forEach((sec, i) => {
        const div = document.createElement('div');
        div.className = 'section';
        div.style.background = sec.color;
        div.style.left = (sec.start_ms / dur * w) + 'px';
        div.style.width = ((sec.end_ms - sec.start_ms) / dur * w) + 'px';
        div.style.position = 'absolute';
        div.textContent = sec.name;
        div.onclick = (e) => { if (!dragging) seekTo(sec.start_ms); };

        // Drag handle for end
        const handle = document.createElement('div');
        handle.className = 'section-handle';
        handle.onmousedown = (e) => startDragSection(e, i);
        div.appendChild(handle);

        el.appendChild(div);
    });
}

function startDragSection(e, idx) {
    e.stopPropagation();
    dragging = idx;
    dragType = 'section';
    document.onmousemove = dragSection;
    document.onmouseup = stopDrag;
}

function dragSection(e) {
    if (dragType !== 'section') return;
    const rect = container.getBoundingClientRect();
    const x = e.clientX - rect.left + wrapper.scrollLeft;
    const dur = getDur();
    const w = container.offsetWidth;
    let ms = Math.round(x / w * dur);
    ms = Math.max(0, Math.min(dur, ms));

    // Update this section's end and next section's start
    choreo.sections[dragging].end_ms = ms;
    if (dragging < choreo.sections.length - 1) {
        choreo.sections[dragging + 1].start_ms = ms;
    }
    renderSections();
}

// Moves
function renderMoves() {
    const el = document.getElementById('moves');
    const list = document.getElementById('moveList');
    const dur = getDur();
    const w = container.offsetWidth;

    el.innerHTML = '';
    list.innerHTML = '';

    choreo.moves.sort((a, b) => a.time_ms - b.time_ms);

    choreo.moves.forEach((m, i) => {
        // Timeline marker
        const marker = document.createElement('div');
        marker.className = 'move-marker' + (i === selIdx ? ' selected' : '');
        marker.style.left = (m.time_ms / dur * w - 3) + 'px';
        marker.innerHTML = `<div class="label">${m.label || m.cmd}</div>`;
        marker.onmousedown = (e) => startDragMove(e, i);
        marker.onclick = (e) => { e.stopPropagation(); selectMove(i); };
        el.appendChild(marker);

        // List item
        const item = document.createElement('div');
        item.className = 'move-item' + (i === selIdx ? ' selected' : '');
        item.innerHTML = `
            <span class="del" onclick="delMove(${i},event)">&times;</span>
            <div class="time">${fmtTime(m.time_ms)}</div>
            <div class="cmd">${m.cmd}</div>
            <div class="lbl">${m.label || ''}</div>
        `;
        item.onclick = () => selectMove(i);
        list.appendChild(item);
    });
}

function startDragMove(e, idx) {
    e.stopPropagation();
    dragging = idx;
    dragType = 'move';
    e.target.classList.add('dragging');
    document.onmousemove = dragMove;
    document.onmouseup = stopDrag;
}

function dragMove(e) {
    if (dragType !== 'move') return;
    const rect = container.getBoundingClientRect();
    const x = e.clientX - rect.left + wrapper.scrollLeft;
    const dur = getDur();
    const w = container.offsetWidth;
    let ms = Math.round(x / w * dur);
    ms = Math.max(0, Math.min(dur, ms));
    choreo.moves[dragging].time_ms = ms;
    renderMoves();
    if (selIdx === dragging) {
        document.getElementById('editTime').value = ms;
    }
}

function stopDrag() {
    document.querySelectorAll('.dragging').forEach(el => el.classList.remove('dragging'));
    dragging = null;
    dragType = null;
    document.onmousemove = null;
    document.onmouseup = null;
}

function selectMove(i) {
    selIdx = i;
    renderMoves();
    showEditor();
}

function showEditor() {
    if (selIdx < 0) {
        document.getElementById('editor').style.display = 'none';
        return;
    }
    const m = choreo.moves[selIdx];
    document.getElementById('editor').style.display = 'block';
    document.getElementById('editTime').value = m.time_ms;
    document.getElementById('editLabel').value = m.label || '';

    const params = document.getElementById('params');
    params.innerHTML = '';
    if (m.params) {
        for (const [k, v] of Object.entries(m.params)) {
            const row = document.createElement('div');
            row.className = 'editor-row';
            if (k === 'intensity' || k === 'speed') {
                row.innerHTML = `<label>${k}:</label>
                    <input type="range" min="0" max="1" step="0.1" value="${v}" oninput="updParam('${k}',this.value)">
                    <span id="pv_${k}">${v}</span>`;
            } else if (k === 'duration') {
                row.innerHTML = `<label>${k}:</label>
                    <input type="range" min="100" max="2000" step="100" value="${v}" oninput="updParam('${k}',this.value)">
                    <span id="pv_${k}">${v}</span>`;
            }
            params.appendChild(row);
        }
    }
}

function updParam(k, v) {
    document.getElementById('pv_' + k).textContent = v;
    if (selIdx >= 0) choreo.moves[selIdx].params[k] = parseFloat(v);
}

function liveUpdate() {
    if (selIdx < 0) return;
    choreo.moves[selIdx].time_ms = parseInt(document.getElementById('editTime').value) || 0;
    choreo.moves[selIdx].label = document.getElementById('editLabel').value;
    renderMoves();
}

function delMove(i, e) {
    e.stopPropagation();
    choreo.moves.splice(i, 1);
    selIdx = -1;
    renderMoves();
    document.getElementById('editor').style.display = 'none';
}

function delSelected() {
    if (selIdx >= 0) {
        choreo.moves.splice(selIdx, 1);
        selIdx = -1;
        renderMoves();
        document.getElementById('editor').style.display = 'none';
    }
}

function dupeMove() {
    if (selIdx < 0) return;
    const m = JSON.parse(JSON.stringify(choreo.moves[selIdx]));
    m.time_ms += 500;
    m.label = (m.label || '') + ' copy';
    choreo.moves.push(m);
    renderMoves();
}

async function testMove() {
    if (selIdx < 0) return;
    const m = choreo.moves[selIdx];
    await fetch('/api/execute', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({cmd: m.cmd, params: m.params})
    });
}

// Add move
function renderCats() {
    const el = document.getElementById('cats');
    el.innerHTML = '';
    for (const cat of Object.keys(CMDS)) {
        const div = document.createElement('div');
        div.className = 'cmd-cat' + (cat === activeCat ? ' active' : '');
        div.textContent = cat;
        div.onclick = () => { activeCat = cat; renderCats(); renderCmds(); };
        el.appendChild(div);
    }
}

function renderCmds() {
    const el = document.getElementById('cmds');
    el.innerHTML = '';
    for (const cmd of CMDS[activeCat] || []) {
        const btn = document.createElement('button');
        btn.className = 'cmd-btn';
        btn.textContent = cmd;
        btn.onclick = () => addMove(cmd);
        el.appendChild(btn);
    }
}

function addMove(cmd) {
    const t = parseInt(document.getElementById('addTime').value) || 0;
    const params = {};
    if (['lookUp','lookDown','leanLeft','leanRight','twistLeft','twistRight','squat','extend'].includes(cmd)) {
        params.intensity = 0.5;
        params.duration = 300;
    }
    choreo.moves.push({ time_ms: t, cmd, params, label: cmd });
    renderMoves();
}

function updAddDisp() {
    document.getElementById('addTimeDisp').textContent = fmtTime(parseInt(document.getElementById('addTime').value) || 0);
}

function setAddCurrent() {
    document.getElementById('addTime').value = curTime;
    updAddDisp();
}

// Timeline click - seek to position and set add time
function timelineClick(e) {
    if (dragging) return;
    // Use wrapper rect (visible area) + scrollLeft to get position in full timeline
    const wrapperRect = wrapper.getBoundingClientRect();
    const x = e.clientX - wrapperRect.left + wrapper.scrollLeft;
    const dur = getDur();
    const w = container.offsetWidth;  // Full width including zoom
    const ms = Math.round(x / w * dur);

    const wasPlaying = playing;

    // Stop interval if playing
    if (playing) {
        if (interval) clearInterval(interval);
        interval = null;
        audio.pause();
        playing = false;
    }

    // Update curTime and UI
    curTime = ms;
    document.getElementById('curTime').textContent = fmtTime(ms);
    document.getElementById('progFill').style.width = (ms / dur * 100) + '%';
    document.getElementById('playhead').style.left = (ms / dur * container.offsetWidth) + 'px';
    document.getElementById('addTime').value = ms;
    updAddDisp();
    highlightCurrentMove(ms);

    // Seek audio and wait for it to complete
    audio.currentTime = ms / 1000;

    // Resume if was playing - wait for seeked event
    if (wasPlaying) {
        audio.addEventListener('seeked', function onSeeked() {
            audio.removeEventListener('seeked', onSeeked);
            play();
        }, { once: true });
    }
}

// Ruler
function renderRuler() {
    const el = document.getElementById('ruler');
    const dur = getDur();
    const w = container.offsetWidth;
    el.innerHTML = '';
    el.style.width = w + 'px';

    const step = dur > 180000 ? 30000 : 10000;
    const tickW = step / dur * w;

    for (let t = 0; t <= dur; t += step) {
        const tick = document.createElement('div');
        tick.className = 'time-tick';
        tick.style.width = tickW + 'px';
        tick.textContent = fmtShort(t);
        el.appendChild(tick);
    }
}

// Playback
function togglePlay() {
    if (playing) pause(); else play();
}

function play() {
    playing = true;
    document.getElementById('playBtn').textContent = 'Pause';
    audio.muted = document.getElementById('mute').checked;

    // Just play from wherever the audio currently is
    audio.play().catch(e => console.log('Play error:', e));

    const executeRobot = document.getElementById('execRobot').checked;
    let lastIdx = -1;
    // Use audio's actual position for tracking
    const startMs = Math.floor(audio.currentTime * 1000);
    curTime = startMs;
    choreo.moves.forEach((m, i) => { if (m.time_ms < startMs) lastIdx = i; });

    interval = setInterval(() => {
        curTime = Math.floor(audio.currentTime * 1000);
        const dur = getDur();
        if (curTime >= dur || audio.ended) { stopPlay(); return; }

        const pct = curTime / dur * 100;
        document.getElementById('curTime').textContent = fmtTime(curTime);
        document.getElementById('progFill').style.width = pct + '%';
        document.getElementById('playhead').style.left = (curTime / dur * container.offsetWidth) + 'px';

        // Sync add-move timestamp to current position
        document.getElementById('addTime').value = curTime;
        updAddDisp();

        // Highlight current/next move in list
        highlightCurrentMove(curTime);

        // Execute moves only if robot execution is enabled
        if (executeRobot) {
            choreo.moves.forEach((m, i) => {
                if (i > lastIdx && m.time_ms <= curTime) {
                    lastIdx = i;
                    execMove(m.cmd, m.params);
                }
            });
        } else {
            // Still track position for next play
            choreo.moves.forEach((m, i) => {
                if (i > lastIdx && m.time_ms <= curTime) lastIdx = i;
            });
        }
    }, 50);
}

function highlightCurrentMove(time) {
    const items = document.querySelectorAll('.move-item');
    items.forEach(item => {
        item.classList.remove('current', 'next');
    });

    // Find the current and next move
    let currentIdx = -1;
    let nextIdx = -1;
    for (let i = 0; i < choreo.moves.length; i++) {
        if (choreo.moves[i].time_ms <= time) {
            currentIdx = i;
        } else if (nextIdx === -1) {
            nextIdx = i;
            break;
        }
    }

    if (currentIdx >= 0 && items[currentIdx]) {
        items[currentIdx].classList.add('current');
        // Scroll into view
        items[currentIdx].scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    }
    if (nextIdx >= 0 && items[nextIdx]) {
        items[nextIdx].classList.add('next');
    }
}

function pause() {
    playing = false;
    document.getElementById('playBtn').textContent = 'Play';
    if (interval) clearInterval(interval);
    audio.pause();
}

function stopPlay() {
    pause();
    curTime = 0;
    audio.currentTime = 0;
    document.getElementById('curTime').textContent = fmtTime(0);
    document.getElementById('progFill').style.width = '0%';
    document.getElementById('playhead').style.left = '0px';

    // Clear move highlighting
    document.querySelectorAll('.move-item').forEach(item => {
        item.classList.remove('current', 'next');
    });
}

function seekTo(ms) {
    curTime = ms;
    const dur = getDur();

    // Update UI immediately
    document.getElementById('curTime').textContent = fmtTime(ms);
    document.getElementById('progFill').style.width = (ms / dur * 100) + '%';
    document.getElementById('playhead').style.left = (ms / dur * container.offsetWidth) + 'px';

    // Sync add-time
    document.getElementById('addTime').value = ms;
    updAddDisp();

    // Highlight current move
    highlightCurrentMove(ms);

    // Always set audio time directly
    try {
        audio.currentTime = ms / 1000;
    } catch(e) {
        console.log('Seek error:', e);
    }
}

function seekBar(e) {
    const bar = e.currentTarget;
    const pct = (e.clientX - bar.getBoundingClientRect().left) / bar.offsetWidth;
    seekTo(Math.floor(pct * getDur()));
}

async function execMove(cmd, params) {
    try {
        await fetch('/api/execute', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({cmd, params})
        });
    } catch(e) {}
}

// Connect
async function connect() {
    const res = await fetch('/api/connect', {method: 'POST'});
    const data = await res.json();
    const el = document.getElementById('status');
    el.textContent = data.connected ? 'Connected' : 'Disconnected';
    el.style.background = data.connected ? '#4CAF50' : '#f44336';
}

// Save/Load with modals
let selectedLoad = null;

function saveDialog() {
    document.getElementById('saveName').value = document.getElementById('choreoName').value;
    document.getElementById('saveModal').classList.add('show');
    document.getElementById('saveName').focus();
}

function closeSaveModal(e) {
    if (e && e.target !== e.currentTarget) return;
    document.getElementById('saveModal').classList.remove('show');
}

async function doSave() {
    const name = document.getElementById('saveName').value.trim();
    if (!name) return;
    choreo.name = name;
    choreo.bpm = parseInt(document.getElementById('bpm').value);
    choreo.duration_ms = getDur();
    choreo.flags = flags; // Save flags with choreography
    await fetch('/api/save', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({name, data: choreo})
    });
    closeSaveModal();
    document.getElementById('choreoName').value = name;
}

async function loadDialog() {
    const res = await fetch('/api/list');
    const files = await res.json();
    const list = document.getElementById('loadList');
    selectedLoad = null;
    document.getElementById('loadBtn').disabled = true;

    if (!files.length) {
        list.innerHTML = '<div class="modal-empty">No saved choreographies yet</div>';
    } else {
        list.innerHTML = files.map(f => `
            <div class="modal-item" onclick="selectLoad('${f}', this)">
                <span>${f}</span>
                <span style="color:#666;font-size:11px">.json</span>
            </div>
        `).join('');
    }
    document.getElementById('loadModal').classList.add('show');
}

function selectLoad(name, el) {
    document.querySelectorAll('.modal-item').forEach(i => i.classList.remove('selected'));
    el.classList.add('selected');
    selectedLoad = name;
    document.getElementById('loadBtn').disabled = false;
}

function closeLoadModal(e) {
    if (e && e.target !== e.currentTarget) return;
    document.getElementById('loadModal').classList.remove('show');
}

async function doLoad() {
    if (!selectedLoad) return;
    console.log('Loading:', selectedLoad);
    const url = '/api/load/' + encodeURIComponent(selectedLoad);
    console.log('URL:', url);
    const res = await fetch(url);
    const data = await res.json();
    console.log('Loaded data:', data);
    if (data && data.name) {
        choreo = data;
        document.getElementById('choreoName').value = data.name || selectedLoad;
        document.getElementById('bpm').value = data.bpm || 129;
        document.getElementById('duration').value = data.duration_ms || 290000;
        selIdx = -1;
        // Restore flags from saved data
        flags = data.flags || [];
        document.getElementById('flagCount').textContent = flags.length + ' flags';
        console.log('Calling init(), moves:', choreo.moves.length, 'flags:', flags.length);
        init();
        renderFlags();
    } else {
        console.log('Load failed - no data or no name');
    }
    closeLoadModal();
}

// Audio duration
audio.addEventListener('loadedmetadata', () => {
    const dur = Math.floor(audio.duration * 1000);
    document.getElementById('duration').value = dur;
    choreo.duration_ms = dur;
    updTotTime();
    applyZoom();
});

init();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            html = HTML.replace('CHOREO_JSON', json.dumps(DEFAULT_CHOREOGRAPHY))
            html = html.replace('COMMANDS_JSON', json.dumps(AVAILABLE_COMMANDS))
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode())
        elif self.path == '/api/list':
            self.send_json(get_choreography_files())
        elif self.path.startswith('/api/load/'):
            from urllib.parse import unquote
            name = unquote(self.path[10:])
            self.send_json(load_choreography(name) or {})
        elif self.path.startswith('/audio/'):
            path = MUSIC_DIR / self.path[7:]
            if path.exists():
                file_size = path.stat().st_size
                range_header = self.headers.get('Range')

                if range_header:
                    # Handle Range request for seeking
                    range_match = range_header.replace('bytes=', '').split('-')
                    start = int(range_match[0]) if range_match[0] else 0
                    end = int(range_match[1]) if range_match[1] else file_size - 1
                    length = end - start + 1

                    self.send_response(206)  # Partial Content
                    self.send_header('Content-Type', 'audio/mpeg')
                    self.send_header('Accept-Ranges', 'bytes')
                    self.send_header('Content-Range', f'bytes {start}-{end}/{file_size}')
                    self.send_header('Content-Length', str(length))
                    self.end_headers()

                    with open(path, 'rb') as f:
                        f.seek(start)
                        self.wfile.write(f.read(length))
                else:
                    # Full file request
                    self.send_response(200)
                    self.send_header('Content-Type', 'audio/mpeg')
                    self.send_header('Accept-Ranges', 'bytes')
                    self.send_header('Content-Length', str(file_size))
                    self.end_headers()
                    self.wfile.write(path.read_bytes())
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        data = json.loads(self.rfile.read(length).decode()) if length else {}

        if self.path == '/api/connect':
            self.send_json({'connected': connect_robot()})
        elif self.path == '/api/execute':
            execute_move(data.get('cmd'), data.get('params', {}))
            self.send_json({'ok': True})
        elif self.path == '/api/save':
            save_choreography(data['name'], data['data'])
            self.send_json({'ok': True})
        else:
            self.send_error(404)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


def main():
    port = 8894
    print(f"\n{'='*50}")
    print("  Go1 Choreography Editor")
    print(f"{'='*50}")
    print(f"\n  http://localhost:{port}")
    print("\n  Features:")
    print("  - Drag section boundaries to adjust")
    print("  - Drag move markers on timeline")
    print("  - Click timeline to set add-time & seek")
    print("  - Zoom in/out for precision editing")
    print(f"\n{'='*50}\n")

    webbrowser.open(f'http://localhost:{port}')
    HTTPServer(('', port), Handler).serve_forever()


if __name__ == "__main__":
    main()
