#!/usr/bin/env python3
"""
Go1 Quadruped - Walking with CPG (Central Pattern Generator)

A proper walking gait using oscillators for each leg.
Diagonal legs move together (trot gait).

Controls:
  Up/Down     - Walk forward/backward
  Left/Right  - Turn left/right
  Space       - Stop
  1-4         - YMCA poses (Y, M, C, A)

MUST be run with mjpython on macOS:
    .venv/bin/mjpython mo_simulation/run_go1_keyboard.py
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path
import math

SCRIPT_DIR = Path(__file__).parent.parent
GO1_XML = SCRIPT_DIR / "models/unitree_go1/scene.xml"

# Joint order from XML: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
#                       RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf

# Standing pose from keyframe
STAND = np.array([
    0, 0.9, -1.8,   # FR: hip, thigh, calf
    0, 0.9, -1.8,   # FL
    0, 0.9, -1.8,   # RR
    0, 0.9, -1.8,   # RL
])

# Commands
velocity = [0.0, 0.0]  # [forward, turn]
dance_pose = None


def on_key(keycode):
    global velocity, dance_pose

    if keycode == 265:  # Up
        velocity[0] = 0.5
        dance_pose = None
        print(">>> WALK FORWARD")
    elif keycode == 264:  # Down
        velocity[0] = -0.5
        dance_pose = None
        print(">>> WALK BACKWARD")
    elif keycode == 263:  # Left
        velocity[1] = 0.5
        dance_pose = None
        print(">>> TURN LEFT")
    elif keycode == 262:  # Right
        velocity[1] = -0.5
        dance_pose = None
        print(">>> TURN RIGHT")
    elif keycode == 32:  # Space
        velocity = [0.0, 0.0]
        dance_pose = None
        print(">>> STOP")
    elif keycode == 49:  # 1 = Y
        dance_pose = 'Y'
        velocity = [0.0, 0.0]
        print(">>> POSE: Y")
    elif keycode == 50:  # 2 = M
        dance_pose = 'M'
        velocity = [0.0, 0.0]
        print(">>> POSE: M")
    elif keycode == 51:  # 3 = C
        dance_pose = 'C'
        velocity = [0.0, 0.0]
        print(">>> POSE: C")
    elif keycode == 52:  # 4 = A
        dance_pose = 'A'
        velocity = [0.0, 0.0]
        print(">>> POSE: A")


def cpg_gait(t, vx, wz):
    """
    Central Pattern Generator for trot gait.
    Diagonal pairs move together: (FR, RL) and (FL, RR)
    """
    if abs(vx) < 0.1 and abs(wz) < 0.1:
        return STAND.copy()

    pose = np.zeros(12)
    freq = 2.5  # Hz
    phase = t * freq * 2 * np.pi

    # Gait parameters - tuned for Go1
    lift_height = 0.15      # How much to lift foot (thigh angle)
    step_length = 0.2       # Forward/back swing (thigh angle)
    turn_amount = 0.15      # Hip abduction for turning

    # Leg phases: FR and RL together (0), FL and RR together (pi)
    leg_data = [
        (0, 1, 2, 0, -1),          # FR: indices, phase, side (right=-1)
        (3, 4, 5, np.pi, 1),       # FL: indices, phase, side (left=1)
        (6, 7, 8, np.pi, -1),      # RR: indices, phase, side
        (9, 10, 11, 0, 1),         # RL: indices, phase, side
    ]

    for hip_i, thigh_i, calf_i, leg_phase, side in leg_data:
        p = phase + leg_phase

        # Swing phase (lifting leg): when sin(p) > 0
        swing = max(0, np.sin(p))

        # Stance phase (on ground): when sin(p) < 0
        stance = max(0, -np.sin(p))

        # Thigh: oscillate for stepping + lift during swing
        thigh_swing = step_length * vx * np.sin(p)  # Forward/back
        thigh_lift = lift_height * swing             # Lift during swing

        # Calf: extend during swing to clear ground, flex during stance
        calf_swing = 0.3 * swing  # Extend knee during swing

        # Hip: for turning
        hip_turn = turn_amount * wz * side

        # Apply to pose
        pose[hip_i] = hip_turn
        pose[thigh_i] = STAND[1] + thigh_swing + thigh_lift
        pose[calf_i] = STAND[2] - calf_swing

    return pose


def get_dance_pose(name):
    """YMCA dance poses."""
    if name == 'Y':
        return np.array([
            0.4, 0.5, -1.2,    # FR spread
            -0.4, 0.5, -1.2,   # FL spread
            0.3, 0.5, -1.2,    # RR
            -0.3, 0.5, -1.2,   # RL
        ])
    elif name == 'M':
        return np.array([
            0, 1.3, -2.4,      # Crouch low
            0, 1.3, -2.4,
            0, 1.3, -2.4,
            0, 1.3, -2.4,
        ])
    elif name == 'C':
        return np.array([
            0.2, 0.6, -1.3,    # Right side high
            -0.2, 1.2, -2.2,   # Left side low
            0.2, 0.6, -1.3,
            -0.2, 1.2, -2.2,
        ])
    elif name == 'A':
        return np.array([
            0, 0.7, -1.5,      # Medium stance
            0, 0.7, -1.5,
            0, 0.7, -1.5,
            0, 0.7, -1.5,
        ])
    return STAND.copy()


def main():
    global velocity, dance_pose

    print("\n" + "=" * 55)
    print("  GO1 WALKING - CPG Trot Gait")
    print("=" * 55)
    print("\n  Up/Down    = Walk forward/backward")
    print("  Left/Right = Turn")
    print("  Space      = Stop")
    print("  1/2/3/4    = YMCA poses")
    print("\n>>> CLICK MUJOCO WINDOW, THEN PRESS ARROWS! <<<")
    print("=" * 55 + "\n")

    if not GO1_XML.exists():
        print(f"Model not found: {GO1_XML}")
        return

    model = mujoco.MjModel.from_xml_path(str(GO1_XML))
    data = mujoco.MjData(model)
    model.opt.timestep = 0.002

    mujoco.mj_resetDataKeyframe(model, data, 0)

    with mujoco.viewer.launch_passive(
        model, data,
        show_left_ui=False,
        show_right_ui=True,
        key_callback=on_key,
    ) as viewer:
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0.25]

        start_time = time.time()
        last_print = 0

        while viewer.is_running():
            t0 = time.time()
            t = time.time() - start_time

            # Decay velocity (spring back to zero)
            velocity[0] *= 0.98
            velocity[1] *= 0.98

            # Get target pose
            if dance_pose:
                target = get_dance_pose(dance_pose)
            else:
                target = cpg_gait(t, velocity[0], velocity[1])

            # Apply control
            data.ctrl[:12] = target

            # Step physics
            mujoco.mj_step(model, data)

            # Camera tracks robot
            viewer.cam.lookat[:] = data.qpos[:3]
            viewer.sync()

            # Status
            if time.time() - last_print > 0.5:
                pos = data.qpos[:3]
                status = "WALKING" if abs(velocity[0]) > 0.05 or abs(velocity[1]) > 0.05 else "STANDING"
                if dance_pose:
                    status = f"POSE: {dance_pose}"
                print(f"[{status}] pos=({pos[0]:.2f}, {pos[1]:.2f}) vel=({velocity[0]:.2f}, {velocity[1]:.2f})")
                last_print = time.time()

            # Real-time
            dt = time.time() - t0
            if dt < 0.002:
                time.sleep(0.002 - dt)

    print("\nDone!")


if __name__ == "__main__":
    main()
