#!/usr/bin/env python3
"""
Go1 Quadruped - Keyboard Control for YMCA Dance

Controls:
  Arrow Keys  - Body lean (forward/back/left/right)
  W/S         - Body height up/down
  A/D         - Body yaw left/right
  Q/E         - Body roll left/right
  Space       - Reset to stand
  1-4         - Trigger dance poses (Y, M, C, A)
  Esc         - Quit

MUST be run with mjpython on macOS:
    .venv/bin/mjpython mo_simulation/run_go1_keyboard.py

Or from ARA-Robotic venv:
    /Users/dawod/ARA-Robotic/.venv/bin/mjpython mo_simulation/run_go1_keyboard.py
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path
import threading
import math

# Paths - use local Go1 model
SCRIPT_DIR = Path(__file__).parent.parent
GO1_XML = SCRIPT_DIR / "models/unitree_go1/scene.xml"

# Go1 has 12 joints: 4 legs x 3 joints each
# Order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
#        RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf

# Joint indices
FR_HIP, FR_THIGH, FR_CALF = 0, 1, 2
FL_HIP, FL_THIGH, FL_CALF = 3, 4, 5
RR_HIP, RR_THIGH, RR_CALF = 6, 7, 8
RL_HIP, RL_THIGH, RL_CALF = 9, 10, 11

# Default standing pose (from keyframe)
DEFAULT_POSE = np.array([
    0, 0.9, -1.8,  # FR
    0, 0.9, -1.8,  # FL
    0, 0.9, -1.8,  # RR
    0, 0.9, -1.8,  # RL
], dtype=np.float32)

# Config
CONFIG = {
    "simulation_dt": 0.002,
    "height_delta": 0.05,  # How much to change thigh angle
    "lean_delta": 0.1,     # How much to lean
    "yaw_delta": 0.15,     # How much to yaw
    "roll_delta": 0.1,     # How much to roll
}


class KeyboardState:
    """Track keyboard state for hold-to-move behavior."""

    def __init__(self):
        self.window = None
        self.reset_flag = False
        self.dance_pose = None  # 1, 2, 3, 4 for Y, M, C, A
        self.lock = threading.Lock()
        self.glfw = mujoco.viewer.glfw

    def key_callback(self, key):
        """Handle key callback."""
        with self.lock:
            if self.window is None:
                self.window = self.glfw.get_current_context()

            # Space = reset to stand
            if key == 32:  # Space
                self.reset_flag = True
            # Dance poses
            elif key == 49:  # 1 = Y pose
                self.dance_pose = 1
            elif key == 50:  # 2 = M pose
                self.dance_pose = 2
            elif key == 51:  # 3 = C pose
                self.dance_pose = 3
            elif key == 52:  # 4 = A pose
                self.dance_pose = 4

    def is_key_held(self, key_code):
        """Check if key is currently held."""
        with self.lock:
            if self.window is None:
                return False
            try:
                return self.glfw.get_key(self.window, key_code) == self.glfw.PRESS
            except:
                return False

    def get_adjustments(self):
        """Get pose adjustments based on held keys."""
        glfw = self.glfw
        adj = {
            'height': 0,
            'pitch': 0,
            'roll': 0,
            'yaw': 0,
        }

        # W/S - height
        if self.is_key_held(glfw.KEY_W):
            adj['height'] = CONFIG['height_delta']
        elif self.is_key_held(glfw.KEY_S):
            adj['height'] = -CONFIG['height_delta']

        # Arrow keys - lean (pitch/roll body)
        if self.is_key_held(glfw.KEY_UP):
            adj['pitch'] = CONFIG['lean_delta']
        elif self.is_key_held(glfw.KEY_DOWN):
            adj['pitch'] = -CONFIG['lean_delta']

        if self.is_key_held(glfw.KEY_LEFT):
            adj['roll'] = CONFIG['roll_delta']
        elif self.is_key_held(glfw.KEY_RIGHT):
            adj['roll'] = -CONFIG['roll_delta']

        # A/D - yaw
        if self.is_key_held(glfw.KEY_A):
            adj['yaw'] = CONFIG['yaw_delta']
        elif self.is_key_held(glfw.KEY_D):
            adj['yaw'] = -CONFIG['yaw_delta']

        # Q/E - roll
        if self.is_key_held(glfw.KEY_Q):
            adj['roll'] = CONFIG['roll_delta']
        elif self.is_key_held(glfw.KEY_E):
            adj['roll'] = -CONFIG['roll_delta']

        return adj


def compute_pose(base_pose, adjustments, dance_pose=None):
    """Compute target joint positions based on adjustments or dance pose."""
    pose = base_pose.copy()

    # Handle dance poses
    if dance_pose == 1:  # Y - stand tall, spread legs
        pose = np.array([
            0.3, 0.6, -1.2,   # FR - spread out
            -0.3, 0.6, -1.2,  # FL - spread out
            0.3, 0.6, -1.2,   # RR
            -0.3, 0.6, -1.2,  # RL
        ], dtype=np.float32)
        return pose

    elif dance_pose == 2:  # M - crouch low, wiggle prep
        pose = np.array([
            0, 1.2, -2.2,  # FR - crouch
            0, 1.2, -2.2,  # FL
            0, 1.2, -2.2,  # RR
            0, 1.2, -2.2,  # RL
        ], dtype=np.float32)
        return pose

    elif dance_pose == 3:  # C - lean to side (curved)
        pose = np.array([
            0.2, 0.7, -1.4,   # FR
            -0.1, 1.1, -2.0,  # FL - lower
            0.2, 0.7, -1.4,   # RR
            -0.1, 1.1, -2.0,  # RL - lower
        ], dtype=np.float32)
        return pose

    elif dance_pose == 4:  # A - straight stance
        pose = np.array([
            0, 0.8, -1.6,  # FR
            0, 0.8, -1.6,  # FL
            0, 0.8, -1.6,  # RR
            0, 0.8, -1.6,  # RL
        ], dtype=np.float32)
        return pose

    # Apply continuous adjustments
    height = adjustments['height']
    pitch = adjustments['pitch']
    roll = adjustments['roll']
    yaw = adjustments['yaw']

    # Height - adjust all thigh joints
    pose[FR_THIGH] -= height
    pose[FL_THIGH] -= height
    pose[RR_THIGH] -= height
    pose[RL_THIGH] -= height

    # Pitch (lean forward/back) - front vs back legs
    pose[FR_THIGH] += pitch
    pose[FL_THIGH] += pitch
    pose[RR_THIGH] -= pitch
    pose[RL_THIGH] -= pitch

    # Roll (lean left/right) - left vs right legs
    pose[FR_THIGH] -= roll
    pose[RR_THIGH] -= roll
    pose[FL_THIGH] += roll
    pose[RL_THIGH] += roll

    # Yaw (twist) - hip abduction
    pose[FR_HIP] += yaw
    pose[FL_HIP] += yaw
    pose[RR_HIP] -= yaw
    pose[RL_HIP] -= yaw

    return pose


def main():
    print("\n" + "=" * 60)
    print("GO1 QUADRUPED - Keyboard Control for YMCA Dance")
    print("Team YMCA - Robot Rave Hackathon 2026")
    print("=" * 60)
    print("\nControls:")
    print("  W/S         - Body height up/down")
    print("  Arrow Keys  - Lean forward/back/left/right")
    print("  A/D         - Yaw left/right")
    print("  Q/E         - Roll left/right")
    print("  1/2/3/4     - Dance poses Y/M/C/A")
    print("  Space       - Reset to stand")
    print("  Esc         - Quit")
    print("=" * 60 + "\n")

    # Check model exists
    if not GO1_XML.exists():
        print(f"Error: Go1 model not found at {GO1_XML}")
        print("Make sure ARA-Robotic project exists with mujoco_menagerie")
        return

    print(f"Loading MuJoCo model: {GO1_XML}")
    model = mujoco.MjModel.from_xml_path(str(GO1_XML))
    data = mujoco.MjData(model)
    model.opt.timestep = CONFIG["simulation_dt"]

    # Reset to home keyframe
    mujoco.mj_resetDataKeyframe(model, data, 0)

    # State
    target_pose = DEFAULT_POSE.copy()
    kb = KeyboardState()
    counter = 0

    def reset_to_stand():
        nonlocal target_pose
        mujoco.mj_resetDataKeyframe(model, data, 0)
        target_pose = DEFAULT_POSE.copy()
        kb.reset_flag = False
        kb.dance_pose = None
        print("Reset to stand!")

    def key_callback(key):
        kb.key_callback(key)

    print("Launching viewer...")

    with mujoco.viewer.launch_passive(
        model, data,
        show_left_ui=False,
        show_right_ui=True,
        key_callback=key_callback,
    ) as viewer:
        # Set camera
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0.0, 0.0, 0.3]

        start_time = time.time()
        last_status_time = 0

        while viewer.is_running():
            step_start = time.time()

            # Check for reset
            if kb.reset_flag:
                reset_to_stand()

            # Get adjustments
            adj = kb.get_adjustments()

            # Compute target pose
            target_pose = compute_pose(DEFAULT_POSE, adj, kb.dance_pose)

            # Clear dance pose after applying (one-shot)
            if kb.dance_pose:
                print(f"Dance pose: {'YMCA'[kb.dance_pose-1]}")
                kb.dance_pose = None

            # Apply control
            data.ctrl[:12] = target_pose

            # Step simulation
            mujoco.mj_step(model, data)
            counter += 1

            # Follow robot with camera
            viewer.cam.lookat[:] = data.qpos[:3]

            # Sync viewer
            viewer.sync()

            # Print status periodically
            current_time = time.time()
            if current_time - last_status_time > 1.0:
                elapsed = current_time - start_time
                height = data.qpos[2]
                print(f"[{elapsed:.1f}s] Height: {height:.3f}m | Pose: [{target_pose[1]:.2f}, {target_pose[2]:.2f}]")
                last_status_time = current_time

            # Maintain real-time
            elapsed = time.time() - step_start
            sleep_time = CONFIG["simulation_dt"] - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    print("\nDone! Let's make this dog dance!")


if __name__ == "__main__":
    main()
