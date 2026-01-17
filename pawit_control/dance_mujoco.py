#!/usr/bin/env python3
"""
Go1 Quadruped - Tuned Dance Controller (Stable)

FIXES:
1. Differential Steering: Inner legs take shorter steps, outer legs take longer steps.
2. Stability: Widened the default stance (Abduction) so it doesn't tip over.
3. Ground Clearance: Adjusted lift height and phase to prevent toe-stubbing.
"""

import time
import json
import numpy as np
import mujoco
import mujoco.viewer
import pygame
from pathlib import Path

# --- CONFIGURATION ---
SCRIPT_DIR = Path(__file__).parent.parent
GO1_XML = SCRIPT_DIR / "models/unitree_go1/scene.xml"
AUDIO_FILE = "alex_music/music/ymca_music.mp3"
JSON_FILE = "alex_music/music/ymca_music_structure.json"

# --- TUNING PARAMETERS ---
ROBOT_WIDTH = 0.35   # Distance between left/right feet
ROBOT_LENGTH = 0.40  # Distance between front/rear feet
HIP_OFFSET = 0.1     # Radians to widen stance (Stability)

# Base Stand Pose (Widened)
# FR, FL, RR, RL
# Hip, Thigh, Calf
STAND = np.array([
    -HIP_OFFSET, 0.9, -1.8,   # FR
     HIP_OFFSET, 0.9, -1.8,   # FL
    -HIP_OFFSET, 0.9, -1.8,   # RR
     HIP_OFFSET, 0.9, -1.8,   # RL
])

def get_dance_pose(letter):
    """Returns joint angles for Y-M-C-A poses."""
    # Y: Big arm spread
    if letter == 'Y':
        return np.array([
            0.5, 0.0, -1.0, 
            -0.5, 0.0, -1.0, 
            -HIP_OFFSET, 0.9, -1.8, 
            HIP_OFFSET, 0.9, -1.8
        ])
    # M: Deep crouch
    elif letter == 'M':
        return np.array([
            -HIP_OFFSET, 1.4, -2.5, 
            HIP_OFFSET, 1.4, -2.5, 
            -HIP_OFFSET, 1.4, -2.5, 
            HIP_OFFSET, 1.4, -2.5
        ])
    # C: Lean Right
    elif letter == 'C':
        return np.array([
            -0.2, 0.7, -1.4,  # Right side extends
            0.4, 1.2, -2.2,   # Left side crunches
            -0.2, 0.7, -1.4, 
            0.4, 1.2, -2.2
        ])
    # A: Tall narrow stance
    elif letter == 'A':
        return np.array([
            -0.05, 0.6, -1.4, 
            0.05, 0.6, -1.4, 
            -0.05, 0.6, -1.4, 
            0.05, 0.6, -1.4
        ])
    return STAND.copy()

def cpg_gait(t, vx, wz, bpm):
    """
    Stabilized Differential Drive CPG.
    """
    # 1. STOP CHECK
    if abs(vx) < 0.01 and abs(wz) < 0.01:
        return STAND.copy()

    pose = np.zeros(12)
    
    # 2. SYNC FREQUENCY
    # 1 Step per Beat (126 BPM -> 2.1 Hz Step Freq)
    # Trot Cycle = 2 Steps (L/R) -> 1.05 Hz Cycle Freq
    beats_per_sec = bpm / 60.0
    freq = beats_per_sec / 2.0 
    phase = t * freq * 2 * np.pi
    
    # 3. DIFFERENTIAL STEERING (Tank Turn Logic)
    # Calculate velocity for Left and Right sides separately
    # V_left = V_linear - (Width * Omega / 2)
    # V_right = V_linear + (Width * Omega / 2)
    
    vel_left = vx - (ROBOT_WIDTH * wz)
    vel_right = vx + (ROBOT_WIDTH * wz)
    
    velocities = [vel_right, vel_left, vel_right, vel_left] # FR, FL, RR, RL
    
    # 4. SASSY HIP SWAY
    # Side-to-side oscillation to shift weight
    hip_sway = 0.15 * np.sin(phase * 2) 

    # Gait Parameters
    lift_height = 0.12
    step_scale = 0.15  # Scaling factor for velocity -> stride length
    
    # Phase Offsets: FR & RL (0), FL & RR (PI) - Standard Trot
    offsets = [0, np.pi, np.pi, 0]
    
    # Signs for hips (Right side negative, Left side positive)
    side_signs = [-1, 1, -1, 1] 

    for i in range(4):
        p = phase + offsets[i]
        
        # Calculate Stride Length for this specific leg
        stride = velocities[i] * step_scale
        
        # Swing/Stance Math
        sin_p = np.sin(p)
        swing = max(0, sin_p)       # 0 to 1 during swing
        stance = max(0, -sin_p)     # 0 to 1 during stance
        
        # --- JOINT CALCULATIONS ---
        
        # Hip (Roll): Base Offset + Sassy Sway
        # We add sway * sign so they move in same global direction (left/right)
        pose[3*i+0] = (HIP_OFFSET * side_signs[i]) + (hip_sway * side_signs[i])

        # Thigh (Pitch): Base 0.9 + Forward/Back Swing + Lift
        # Sine wave moves leg forward/back. 'swing' adds the vertical hop.
        pose[3*i+1] = 0.9 + (stride * np.sin(p)) + (lift_height * swing)
        
        # Calf (Pitch): Base -1.8 - Knee Retraction
        # Retract knee during swing to clear ground
        pose[3*i+2] = -1.8 + (0.4 * swing)

    return pose

def main():
    print("\n=== GO1 STABLE DANCE CONTROLLER ===")
    
    if not Path(JSON_FILE).exists():
        print("Error: JSON file not found.")
        return

    with open(JSON_FILE) as f:
        data = json.load(f)
    events = data["events"]
    bpm = data.get("tempo", 128.0)
    print(f"Synced to BPM: {bpm:.1f}")

    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load(AUDIO_FILE)

    model = mujoco.MjModel.from_xml_path(str(GO1_XML))
    d_mj = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d_mj, 0)

    # Launch
    with mujoco.viewer.launch_passive(model, d_mj) as viewer:
        time.sleep(1.0)
        pygame.mixer.music.play()
        start_time = time.time()
        
        # Set camera angle
        viewer.cam.lookat[:] = [0,0,0.5]
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 45

        event_idx = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # 1. TIME SYNC
            music_pos = pygame.mixer.music.get_pos()
            now = (time.time() - start_time) if music_pos == -1 else (music_pos / 1000.0)

            # 2. EVENT LOOKUP
            while event_idx < len(events) - 1 and events[event_idx+1]["timestamp"] <= now:
                event_idx += 1
            ev = events[event_idx]
            
            # 3. BEHAVIOR SELECTION
            if ev["type"] == "CHORUS":
                # Pose Logic
                pose_map = ['Y', 'M', 'C', 'A']
                letter = pose_map[ev["bar_index"] % 4]
                target_pose = get_dance_pose(letter)
            else:
                # Walk Logic (Circle)
                # vx=0.3 (Forward), wz=0.5 (Turn Left)
                target_pose = cpg_gait(d_mj.time, vx=0.3, wz=0.5, bpm=bpm)

            # 4. SIMULATION STEP
            d_mj.ctrl[:12] = target_pose
            
            # Run multiple physics steps per frame for stability
            for _ in range(10): 
                mujoco.mj_step(model, d_mj)
                
            viewer.sync()
            viewer.cam.lookat[:] = d_mj.qpos[:3] # Track robot
            
            # Frame limiting
            elapsed = time.time() - step_start
            if elapsed < 0.016: # 60 FPS
                time.sleep(0.016 - elapsed)

    pygame.quit()

if __name__ == "__main__":
    main()