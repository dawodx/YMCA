#!/usr/bin/env python3
"""
Go1 Quadruped - Stationary Marching (YMCA)

- Keeps the robot in one spot (Walking in Place).
- Uses "Phase Locking" to stomp feet exactly on the beat.
- Alternates Diagonal pairs (Trot pattern) based on Kick vs Snare.
"""

from enum import Enum
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

# --- POSE CONSTANTS ---
# We widen the hips (0.15 rad) to make it very stable while standing on 2 legs
HIP_WIDTH = 0.15 
STAND = np.array([
    -HIP_WIDTH, 0.9, -1.8,   # FR
     HIP_WIDTH, 0.9, -1.8,   # FL
    -HIP_WIDTH, 0.9, -1.8,   # RR
     HIP_WIDTH, 0.9, -1.8,   # RL
])

"""
INTRO - the intro to the song
YOUNG_MAN = beginning of vocal section
YMCA - chorus beginning with "It's fun to stay at the Y-M-C-A" (approx 31/32 sec)
"""
Song_States = Enum('Song_States', ['INTRO', 'YOUNG_MAN', 'CHORUS', 'BRIDGE', 'OUTRO'])


def get_song_state_by_time_ms(t_ms):
    if t_ms < 28000:
        return Song_States.INTRO
    elif t_ms < 45000:
        return Song_States.YOUNG_MAN
    elif t_ms < 60000:
        return Song_States.CHORUS
    elif t_ms < 91000:
        return Song_States.YOUNG_MAN
    elif t_ms < 124000:
        return Song_States.CHORUS
    elif t_ms < 156000:
        return Song_States.YOUNG_MAN
    elif t_ms < 189000:
        return Song_States.CHORUS
    else: # 4 seconds onward
        return Song_States.OUTRO

def get_pose_by_letter(letter):
    """Static Poses for Chorus."""
    if letter == 'Y':
        return np.array([
            0.6, 0.0, -1.0, -0.6, 0.0, -1.0,
            -0.1, 0.9, -1.8, 0.1, 0.9, -1.8
        ])
    elif letter == 'M':
        # Deep crouch
        return np.array([-0.1, 1.5, -2.6, 0.1, 1.5, -2.6] * 2)
    elif letter == 'C':
        # Lean Right
        return np.array([-0.3, 0.7, -1.4, 0.5, 1.2, -2.2, -0.3, 0.7, -1.4, 0.5, 1.2, -2.2])
    elif letter == 'A':
        # Tall
        return np.array([-0.1, 0.6, -1.4, 0.1, 0.6, -1.4] * 2)
    return STAND.copy()

def get_beat_phase(now, events):
    """Calculates progress (0.0 -> 1.0) through the current beat."""
    idx = 0
    for i, e in enumerate(events):
        if e["timestamp"] > now:
            break
        idx = i
    
    # Safety checks
    if idx >= len(events) - 1: return events[-1], 0.0
    
    current_evt = events[idx]
    next_evt = events[idx + 1]
    
    beat_duration = next_evt["timestamp"] - current_evt["timestamp"]
    if beat_duration <= 0: beat_duration = 0.5
    
    phase = (now - current_evt["timestamp"]) / beat_duration
    return current_evt, np.clip(phase, 0.0, 1.0)

def march_in_place(phase, drum_type):
    """
    High-Step Marching in place.
    """
    pose = np.zeros(12)
    
    # 1. PARAMETERS
    lift_height = 0.15  # VERY HIGH STEP (Visually energetic)
    
    # 2. SELECT LEGS
    # KICK (Beat 1, 3) -> Lift Group A (FR + RL)
    # SNARE (Beat 2, 4) -> Lift Group B (FL + RR)
    if drum_type == "KICK":
        # 1 = Swing, 0 = Stance
        leg_mask = [1, 0, 0, 1] 
        sway_dir = 1 # Shift body weight to the RIGHT (stancing legs)
    else:
        leg_mask = [0, 1, 1, 0]
        sway_dir = -1 # Shift body weight to the LEFT
        
    # 3. PHASE CURVE
    # Sine wave 0 -> 1 -> 0 over the duration of the beat
    swing_curve = np.sin(phase * np.pi) 
    
    # 4. SWAY CURVE
    # We shift the Hips to balance over the planted feet
    hip_shift = 0.05 * swing_curve * sway_dir

    for i in range(4):
        # Base stance
        side_sign = -1 if i in [0, 2] else 1
        pose[3*i+0] = (side_sign * HIP_WIDTH) + hip_shift # Hip Roll (Balance)
        pose[3*i+1] = 0.9  # Thigh Pitch
        pose[3*i+2] = -1.8 # Calf Pitch

        # If this leg is swinging (Stepping up)
        if leg_mask[i] == 1:
            # Lift Thigh
            pose[3*i+1] -= (lift_height * swing_curve) 
            # Retract Calf (Bend knee)
            pose[3*i+2] += (lift_height * 2.0 * swing_curve) 

    return pose

def main():
    print("=== YMCA MARCHER ===")
    
    # Load
    if not Path(JSON_FILE).exists():
        print(f"Missing {JSON_FILE}")
        return
    with open(JSON_FILE) as f:
        data = json.load(f)
    events = data["events"]

    # Sim
    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load(AUDIO_FILE)
    
    model = mujoco.MjModel.from_xml_path(str(GO1_XML))
    d_mj = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d_mj, 0)

    with mujoco.viewer.launch_passive(model, d_mj) as viewer:
        time.sleep(1.0)
        pygame.mixer.music.play()
        start_time = time.time()
        
        viewer.cam.lookat[:] = [0, 0, 0.5]
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 90
        
        while viewer.is_running():
            step_start = time.time()
            
            # Sync
            music_pos = pygame.mixer.music.get_pos()
            now = (time.time() - start_time) if music_pos == -1 else (music_pos / 1000.0)
            song_state = get_song_state_by_time_ms(music_pos)
            print(song_state)

            # Brain
            evt, phase = get_beat_phase(now, events)
            
            # Body
            if evt["type"] == "CHORUS":
                # Pose
                poses = ['Y', 'M', 'C', 'A']
                target = get_pose_by_letter(poses[evt["bar_index"] % 4])
                txt = f"POSE: {poses[evt['bar_index'] % 4]}"
            else:
                # March
                target = march_in_place(phase, evt["drum"])
                txt = f"MARCH: {evt['drum']}"

            # Physics
            d_mj.ctrl[:12] = target
            for _ in range(10): # High frequency physics
                mujoco.mj_step(model, d_mj)
            
            viewer.sync()
            
            if phase < 0.05: # Print on beat start
                print(f"[{now:.2f}s] {txt}")

            # Frame rate
            elapsed = time.time() - step_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

    pygame.quit()

if __name__ == "__main__":
    main()