#!/usr/bin/env python3
"""
Go1 MuJoCo Simulation Server

Runs MuJoCo simulation with HTTP API compatible with the dashboard.
Use this to test choreography in simulation before running on real robot.

MUST be run with mjpython on macOS:
    .venv/bin/mjpython mo_simulation/go1_sim_server.py

Then open the dashboard and click "SIM" mode to connect.
"""

import time
import json
import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import webbrowser

SCRIPT_DIR = Path(__file__).parent.parent
GO1_XML = SCRIPT_DIR / "models/unitree_go1/scene.xml"

# Joint order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
#              RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf

# ============== POSES ==============

STAND = np.array([
    0, 0.9, -1.8,   # FR
    0, 0.9, -1.8,   # FL
    0, 0.9, -1.8,   # RR
    0, 0.9, -1.8,   # RL
])

SIT = np.array([
    0, 1.4, -2.6,   # FR - folded
    0, 1.4, -2.6,   # FL
    0, 1.4, -2.6,   # RR
    0, 1.4, -2.6,   # RL
])

STAND_UP_HIGH = np.array([
    0, 0.5, -1.0,   # FR - extended
    0, 0.5, -1.0,   # FL
    0, 0.5, -1.0,   # RR
    0, 0.5, -1.0,   # RL
])

# Dance poses
DANCE1_POSES = [
    np.array([0.3, 0.6, -1.2, -0.3, 0.6, -1.2, 0.3, 0.6, -1.2, -0.3, 0.6, -1.2]),  # Wide stance
    np.array([0, 1.2, -2.2, 0, 1.2, -2.2, 0, 0.5, -1.0, 0, 0.5, -1.0]),  # Front low, back high
    np.array([0, 0.5, -1.0, 0, 0.5, -1.0, 0, 1.2, -2.2, 0, 1.2, -2.2]),  # Front high, back low
    np.array([-0.3, 0.8, -1.6, 0.3, 0.8, -1.6, -0.3, 0.8, -1.6, 0.3, 0.8, -1.6]),  # Twist
]

DANCE2_POSES = [
    np.array([0, 0.3, -0.6, 0, 0.3, -0.6, 0, 1.3, -2.4, 0, 1.3, -2.4]),  # Front up
    np.array([0, 1.3, -2.4, 0, 1.3, -2.4, 0, 0.3, -0.6, 0, 0.3, -0.6]),  # Back up
    np.array([0.4, 0.7, -1.4, -0.4, 0.7, -1.4, 0.4, 0.7, -1.4, -0.4, 0.7, -1.4]),  # Side twist
    np.array([-0.4, 0.7, -1.4, 0.4, 0.7, -1.4, -0.4, 0.7, -1.4, 0.4, 0.7, -1.4]),  # Side twist other way
]

JUMP_POSE = np.array([
    0, 0.3, -0.6,   # Extended for jump
    0, 0.3, -0.6,
    0, 0.3, -0.6,
    0, 0.3, -0.6,
])

PRAY_POSE = np.array([
    -0.3, 0.4, -0.8,  # FR - front legs together
    0.3, 0.4, -0.8,   # FL
    0, 1.0, -2.0,     # RR - back legs normal
    0, 1.0, -2.0,     # RL
])

STRETCH_POSE = np.array([
    0, 0.2, -0.4,   # FR - front extended forward
    0, 0.2, -0.4,   # FL
    0, 1.4, -2.6,   # RR - back tucked
    0, 1.4, -2.6,   # RL
])

HANDSTAND_POSE = np.array([
    0, 0.1, -0.2,   # FR - very extended (weight bearing)
    0, 0.1, -0.2,   # FL
    0, -0.5, -0.3,  # RR - lifted up
    0, -0.5, -0.3,  # RL
])

# YMCA Poses (from dance_mujoco.py)
YMCA_Y = np.array([
    0.6, 0.0, -1.0,   # FR - up and out
    -0.6, 0.0, -1.0,  # FL - up and out
    -0.1, 0.9, -1.8,  # RR - standing
    0.1, 0.9, -1.8,   # RL - standing
])

YMCA_M = np.array([
    -0.1, 1.5, -2.6,  # FR - deep crouch
    0.1, 1.5, -2.6,   # FL
    -0.1, 1.5, -2.6,  # RR
    0.1, 1.5, -2.6,   # RL
])

YMCA_C = np.array([
    -0.3, 0.7, -1.4,  # FR - lean right
    0.5, 1.2, -2.2,   # FL
    -0.3, 0.7, -1.4,  # RR
    0.5, 1.2, -2.2,   # RL
])

YMCA_A = np.array([
    -0.1, 0.6, -1.4,  # FR - tall
    0.1, 0.6, -1.4,   # FL
    -0.1, 0.6, -1.4,  # RR
    0.1, 0.6, -1.4,   # RL
])

# Body pose adjustments (relative to stand)
LOOK_UP = np.array([0, -0.2, 0.3, 0, -0.2, 0.3, 0, 0.2, -0.3, 0, 0.2, -0.3])
LOOK_DOWN = np.array([0, 0.3, -0.4, 0, 0.3, -0.4, 0, -0.2, 0.2, 0, -0.2, 0.2])
LEAN_LEFT = np.array([0.2, 0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, -0.3, -0.2, -0.1, 0.1])
LEAN_RIGHT = np.array([-0.2, -0.1, 0.1, 0.2, 0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, -0.3])
TWIST_LEFT = np.array([0.2, 0, 0, -0.2, 0, 0, -0.2, 0, 0, 0.2, 0, 0])
TWIST_RIGHT = np.array([-0.2, 0, 0, 0.2, 0, 0, 0.2, 0, 0, -0.2, 0, 0])
SQUAT = np.array([0, 0.3, -0.5, 0, 0.3, -0.5, 0, 0.3, -0.5, 0, 0.3, -0.5])
EXTEND = np.array([0, -0.3, 0.5, 0, -0.3, 0.5, 0, -0.3, 0.5, 0, -0.3, 0.5])

# ============== GLOBAL STATE ==============

class SimState:
    def __init__(self):
        self.command = "stand"
        self.velocity = [0.0, 0.0]  # [forward, turn]
        self.target_pose = STAND.copy()
        self.dance_step = 0
        self.dance_time = 0
        self.led_color = [0, 255, 0]  # RGB for visualization
        self.connected = True
        self.action_start_time = 0
        self.action_duration = 0

state = SimState()

# ============== GAIT ==============

def cpg_gait(t, vx, wz):
    """Central Pattern Generator for trot gait."""
    if abs(vx) < 0.1 and abs(wz) < 0.1:
        return STAND.copy()

    pose = np.zeros(12)
    freq = 2.5
    phase = t * freq * 2 * np.pi

    lift_height = 0.15
    step_length = 0.2
    turn_amount = 0.15

    leg_data = [
        (0, 1, 2, 0, -1),
        (3, 4, 5, np.pi, 1),
        (6, 7, 8, np.pi, -1),
        (9, 10, 11, 0, 1),
    ]

    for hip_i, thigh_i, calf_i, leg_phase, side in leg_data:
        p = phase + leg_phase
        swing = max(0, np.sin(p))
        thigh_swing = step_length * vx * np.sin(p)
        thigh_lift = lift_height * swing
        calf_swing = 0.3 * swing
        hip_turn = turn_amount * wz * side

        pose[hip_i] = hip_turn
        pose[thigh_i] = STAND[1] + thigh_swing + thigh_lift
        pose[calf_i] = STAND[2] - calf_swing

    return pose

# ============== COMMAND EXECUTION ==============

def execute_sim_command(cmd):
    """Execute command in simulation."""
    global state

    start_time = time.time()
    state.command = cmd
    state.action_start_time = time.time()

    # Movement commands
    if cmd == "forward":
        state.velocity = [0.8, 0.0]
        state.action_duration = 500
    elif cmd == "backward":
        state.velocity = [-0.8, 0.0]
        state.action_duration = 500
    elif cmd == "left":
        state.velocity = [0.0, 0.5]
        state.action_duration = 500
    elif cmd == "right":
        state.velocity = [0.0, -0.5]
        state.action_duration = 500
    elif cmd == "turnLeft":
        state.velocity = [0.0, 0.8]
        state.action_duration = 500
    elif cmd == "turnRight":
        state.velocity = [0.0, -0.8]
        state.action_duration = 500

    # Mode commands
    elif cmd == "stand":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND.copy()
        state.action_duration = 500
    elif cmd == "standUp":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND_UP_HIGH.copy()
        state.action_duration = 1000
    elif cmd == "standDown":
        state.velocity = [0.0, 0.0]
        state.target_pose = SIT.copy()
        state.action_duration = 1000
    elif cmd == "damping":
        state.velocity = [0.0, 0.0]
        state.target_pose = SIT.copy()
        state.action_duration = 500
    elif cmd == "recoverStand":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND.copy()
        state.action_duration = 2000
    elif cmd == "walk":
        state.velocity = [0.3, 0.0]
        state.action_duration = 300
    elif cmd == "run":
        state.velocity = [0.8, 0.0]
        state.action_duration = 300
    elif cmd == "climb":
        state.velocity = [0.4, 0.0]
        state.action_duration = 300

    # Dance commands
    elif cmd == "dance1":
        state.velocity = [0.0, 0.0]
        state.command = "dance1"
        state.dance_step = 0
        state.dance_time = time.time()
        state.action_duration = 5000
    elif cmd == "dance2":
        state.velocity = [0.0, 0.0]
        state.command = "dance2"
        state.dance_step = 0
        state.dance_time = time.time()
        state.action_duration = 5000
    elif cmd in ["dance3", "dance4"]:
        state.velocity = [0.0, 0.0]
        state.command = "dance1"  # Use dance1 as fallback
        state.dance_step = 0
        state.dance_time = time.time()
        state.action_duration = 5000

    # Special moves
    elif cmd == "jumpYaw":
        state.velocity = [0.0, 0.0]
        state.target_pose = JUMP_POSE.copy()
        state.action_duration = 2000
    elif cmd in ["frontJump", "frontPounce", "bound"]:
        state.velocity = [0.0, 0.0]
        state.target_pose = JUMP_POSE.copy()
        state.action_duration = 1500
    elif cmd == "pray":
        state.velocity = [0.0, 0.0]
        state.target_pose = PRAY_POSE.copy()
        state.action_duration = 2000
    elif cmd == "stretch":
        state.velocity = [0.0, 0.0]
        state.target_pose = STRETCH_POSE.copy()
        state.action_duration = 2000
    elif cmd == "handStand":
        state.velocity = [0.0, 0.0]
        state.target_pose = HANDSTAND_POSE.copy()
        state.action_duration = 3000
    elif cmd == "wiggleHips":
        state.velocity = [0.0, 0.0]
        state.command = "wiggle"
        state.dance_time = time.time()
        state.action_duration = 2000
    elif cmd == "straightHand1":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND_UP_HIGH.copy()
        state.action_duration = 2000
    elif cmd == "backflip":
        # Just do a dramatic pose change
        state.velocity = [0.0, 0.0]
        state.target_pose = JUMP_POSE.copy()
        state.action_duration = 3000

    # Body poses
    elif cmd == "lookUp":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + LOOK_UP
        state.action_duration = 400
    elif cmd == "lookDown":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + LOOK_DOWN
        state.action_duration = 400
    elif cmd == "leanLeft":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + LEAN_LEFT
        state.action_duration = 400
    elif cmd == "leanRight":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + LEAN_RIGHT
        state.action_duration = 400
    elif cmd == "twistLeft":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + TWIST_LEFT
        state.action_duration = 400
    elif cmd == "twistRight":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + TWIST_RIGHT
        state.action_duration = 400
    elif cmd == "squat":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + SQUAT
        state.action_duration = 400
    elif cmd == "extend":
        state.velocity = [0.0, 0.0]
        state.target_pose = STAND + EXTEND
        state.action_duration = 400

    # LED commands
    elif cmd.startswith("led"):
        led_colors = {
            "ledRed": [255, 0, 0],
            "ledGreen": [0, 255, 0],
            "ledBlue": [0, 0, 255],
            "ledYellow": [255, 215, 0],
            "ledPink": [255, 20, 147],
            "ledCyan": [0, 188, 212],
            "ledOff": [0, 0, 0],
        }
        if cmd in led_colors:
            state.led_color = led_colors[cmd]
        state.action_duration = 100

    # Wait commands
    elif cmd.startswith("wait"):
        delay = int(cmd.replace("wait", ""))
        state.action_duration = delay

    # YMCA commands
    elif cmd == "ymcaY":
        state.velocity = [0.0, 0.0]
        state.target_pose = YMCA_Y.copy()
        state.action_duration = 1500
    elif cmd == "ymcaM":
        state.velocity = [0.0, 0.0]
        state.target_pose = YMCA_M.copy()
        state.action_duration = 1500
    elif cmd == "ymcaC":
        state.velocity = [0.0, 0.0]
        state.target_pose = YMCA_C.copy()
        state.action_duration = 1500
    elif cmd == "ymcaA":
        state.velocity = [0.0, 0.0]
        state.target_pose = YMCA_A.copy()
        state.action_duration = 1500
    elif cmd == "ymcaMarch":
        state.velocity = [0.0, 0.0]
        state.command = "ymcaMarch"
        state.dance_time = time.time()
        state.action_duration = 2000
    elif cmd == "ymcaDance":
        state.velocity = [0.0, 0.0]
        state.command = "ymcaDance"
        state.dance_time = time.time()
        state.dance_step = 0
        state.action_duration = 30000

    elapsed = (time.time() - start_time) * 1000
    return {"name": cmd, "time_ms": round(elapsed, 1), "success": True}

# ============== HTTP SERVER ==============

class SimHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress logs

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path == '/api/status':
            self.send_json({'connected': state.connected, 'mode': 'simulation'})
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/api/connect':
            state.connected = True
            self.send_json({'connected': True, 'time_ms': 1, 'mode': 'simulation'})
        elif self.path == '/api/unlock':
            self.send_json({'success': True, 'message': 'Simulation mode', 'time_ms': 1})
        elif self.path.startswith('/api/cmd/'):
            cmd = self.path.split('/')[-1]
            result = execute_sim_command(cmd)
            self.send_json(result)
        elif self.path.startswith('/api/led/'):
            parts = self.path.split('/')
            if len(parts) >= 6:
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                state.led_color = [r, g, b]
            self.send_json({'success': True})
        elif self.path == '/api/reset':
            # Reset simulation to initial state
            state.command = "stand"
            state.velocity = [0.0, 0.0]
            state.target_pose = STAND.copy()
            state.dance_step = 0
            state.led_color = [0, 255, 0]
            self.send_json({'success': True, 'time_ms': 1})
        else:
            self.send_error(404)

def run_http_server(port=8891):
    """Run HTTP server in background thread."""
    server = HTTPServer(('', port), SimHandler)
    print(f"  Simulation API: http://localhost:{port}")
    server.serve_forever()

# ============== MAIN ==============

def main():
    print("\n" + "=" * 55)
    print("  GO1 SIMULATION SERVER")
    print("=" * 55)
    print("\n  This runs MuJoCo simulation with dashboard API.")
    print("  Open the dashboard and change port to 8891 for sim mode.")
    print("=" * 55)

    if not GO1_XML.exists():
        print(f"ERROR: Model not found: {GO1_XML}")
        return

    # Start HTTP server in background
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    # Load MuJoCo
    model = mujoco.MjModel.from_xml_path(str(GO1_XML))
    data = mujoco.MjData(model)
    model.opt.timestep = 0.002

    mujoco.mj_resetDataKeyframe(model, data, 0)

    print("\n  Starting MuJoCo viewer...")
    print("  Dashboard API ready on port 8891")
    print("=" * 55 + "\n")

    with mujoco.viewer.launch_passive(
        model, data,
        show_left_ui=False,
        show_right_ui=True,
    ) as viewer:
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5
        viewer.cam.lookat[:] = [0, 0, 0.25]

        start_time = time.time()
        last_print = 0

        while viewer.is_running():
            t0 = time.time()
            t = time.time() - start_time

            # Decay velocity
            state.velocity[0] *= 0.98
            state.velocity[1] *= 0.98

            # Get target pose based on current command
            if state.command == "dance1":
                # Cycle through dance poses
                elapsed = time.time() - state.dance_time
                step = int(elapsed * 2) % len(DANCE1_POSES)
                target = DANCE1_POSES[step]
            elif state.command == "dance2":
                elapsed = time.time() - state.dance_time
                step = int(elapsed * 2) % len(DANCE2_POSES)
                target = DANCE2_POSES[step]
            elif state.command == "wiggle":
                # Wiggle hips side to side
                elapsed = time.time() - state.dance_time
                wiggle = 0.3 * np.sin(elapsed * 8)
                target = STAND.copy()
                target[0] = wiggle
                target[3] = wiggle
                target[6] = wiggle
                target[9] = wiggle
            elif state.command == "ymcaMarch":
                # Marching in place - alternate diagonal pairs
                elapsed = time.time() - state.dance_time
                phase = (elapsed * 2) % 1.0  # 2 Hz march
                swing = np.sin(phase * np.pi)
                lift = 0.15
                target = STAND.copy()
                # Trot gait: FR+RL vs FL+RR
                if phase < 0.5:
                    # Lift FR and RL
                    target[1] -= lift * swing * 2  # FR thigh
                    target[2] += lift * 2 * swing * 2  # FR calf
                    target[10] -= lift * swing * 2  # RL thigh
                    target[11] += lift * 2 * swing * 2  # RL calf
                else:
                    # Lift FL and RR
                    target[4] -= lift * swing * 2  # FL thigh
                    target[5] += lift * 2 * swing * 2  # FL calf
                    target[7] -= lift * swing * 2  # RR thigh
                    target[8] += lift * 2 * swing * 2  # RR calf
            elif state.command == "ymcaDance":
                # Full YMCA dance sequence
                elapsed = time.time() - state.dance_time
                cycle_time = elapsed % 8.0  # 8 second cycle for Y-M-C-A
                if cycle_time < 2.0:
                    target = YMCA_Y.copy()
                elif cycle_time < 4.0:
                    target = YMCA_M.copy()
                elif cycle_time < 6.0:
                    target = YMCA_C.copy()
                else:
                    target = YMCA_A.copy()
            elif abs(state.velocity[0]) > 0.1 or abs(state.velocity[1]) > 0.1:
                target = cpg_gait(t, state.velocity[0], state.velocity[1])
            else:
                target = state.target_pose

            # Smooth transition to target
            current = data.ctrl[:12]
            alpha = 0.1  # Smoothing factor
            data.ctrl[:12] = current + alpha * (target - current)

            # Step physics
            mujoco.mj_step(model, data)

            # Camera tracks robot
            viewer.cam.lookat[:] = data.qpos[:3]
            viewer.sync()

            # Status print
            if time.time() - last_print > 1.0:
                pos = data.qpos[:3]
                led = state.led_color
                print(f"[SIM] cmd={state.command:12} pos=({pos[0]:.1f},{pos[1]:.1f}) LED=RGB({led[0]},{led[1]},{led[2]})")
                last_print = time.time()

            # Real-time sync
            dt = time.time() - t0
            if dt < 0.002:
                time.sleep(0.002 - dt)

    print("\nSimulation ended.")

if __name__ == "__main__":
    main()
