#!/usr/bin/env python3
"""
YMCA Final Choreographer

LOGIC FLOW:
1. INTRO (0-27s): Twist Left/Right on the beat.
2. YOUNG MAN (27-62s, etc):
   - ON ENTER: Send 'wiggleHips' ONCE.
   - ON BEAT: Flash LEDs (Green/Yellow) only. Do not interrupt the wiggle.
3. CHORUS (62-91s, etc):
   - ON BEAT: Cycle specific poses (LookUp, LookDown, Squat, Extend) matching Y-M-C-A.
"""

import time
import requests
import pygame
from enum import Enum
from pathlib import Path

# --- CONFIG ---
DASHBOARD_URL = "http://localhost:8891/api/cmd" # Ensure port matches dashboard
AUDIO_FILE = "alex_music/music/ymca_music.mp3"
BPM = 129.0 # Standard disco is usually 126-128

# --- STATES ---
class SongState(Enum):
    INTRO_1 = 1
    INTRO_2 = 2
    CHORUS = 3
    OUTRO = 4

# --- MOVES ---
MOVES_INTRO = ["twistLeft", "twistRight"]

def get_song_state(t_ms):
    """Timestamp map for YMCA."""
    if t_ms < 27500:
        return SongState.INTRO_1
    elif t_ms < 62000: # "Young man..."
        return SongState.INTRO_2
    elif t_ms < 92000: # "It's fun to stay..."
        return SongState.CHORUS
    elif t_ms < 125000: # Verse 2
        return SongState.INTRO_1
    elif t_ms < 155000: # Chorus 2
        return SongState.CHORUS
    elif t_ms < 200000: # Verse 3
        return SongState.INTRO_1
    elif t_ms < 220000: # Chorus 3
        return SongState.CHORUS
    else:
        return SongState.OUTRO

def send_command(cmd_name):
    """Fire and forget command."""
    try:
        url = f"{DASHBOARD_URL}/{cmd_name}"
        requests.post(url, timeout=0.02) # Ultra short timeout for rhythm
    except:
        pass 

def main():
    print(f"--- YMCA FINAL CLIENT ({BPM} BPM) ---")
    
    if not Path(AUDIO_FILE).exists():
        print(f"Error: {AUDIO_FILE} not found")
        return

    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load(AUDIO_FILE)

    input("Press Enter to START...")
    print("3... 2... 1... GO!")
    
    pygame.mixer.music.play()
    start_time = time.time()
    
    last_state = None
    # 60.0 / BPM is standard. 
    # If using 65.0, you are adding lag compensation, but 60 is mathematically correct for music.
    beat_interval = 60.0 / BPM 
    next_beat_time = 0.0
    beat_counter = 0
    intro_move_index = 0

    while True:
        if not pygame.mixer.music.get_busy():
            break
            
        # 1. TIMEKEEPING
        now = time.time() - start_time
        now_ms = now * 1000
        current_state = get_song_state(now_ms)
        
        # 2. STATE TRANSITIONS (Run ONCE when section changes)
        if current_state != last_state:
            print(f"\n[{now:.2f}s] === ENTERING {current_state.name} ===")
            
            if current_state == SongState.INTRO_1:
                send_command("stand")
                send_command("ledYellow")

            elif current_state == SongState.INTRO_2:
                # ACTION: Wiggle Hips (Once)
                # This puts the robot in a special mode that loops internally
                send_command("stand")
                send_command("wiggleHips")
                print(">> Command: wiggleHips (Loop Started)")

            elif current_state == SongState.CHORUS:
                # ACTION: Stop Wiggling, prepare for poses
                send_command("stand") # Reset pose
                send_command("ledPink")
                
            elif current_state == SongState.OUTRO:
                send_command("standDown")
                send_command("ledOff")
                
            last_state = current_state

        # 3. BEAT ACTIONS (Run EVERY BEAT)
        if now >= next_beat_time:
            
            # --- INTRO LOGIC ---
            if current_state == SongState.INTRO_1:
                # Twist back and forth
                cmd = MOVES_INTRO[intro_move_index % 2]
                send_command(cmd)
                print(f"[{now:.2f}s] Beat {beat_counter}: {cmd}")
                intro_move_index += 1

            # --- CHORUS LOGIC (Y-M-C-A) ---
            elif current_state == SongState.CHORUS:
                # Cycle through 4 poses
                cycle = beat_counter % 4
                
                if cycle == 0:
                    # Y - Squat
                    send_command("squat")
                    print(f"[{now:.2f}s] Y (Squat)")
                elif cycle == 1:
                    # M - Stand
                    send_command("stand")
                    print(f"[{now:.2f}s] M (Stand)")
                elif cycle == 2:
                    # C - Twist Left
                    send_command("twistLeft")
                    print(f"[{now:.2f}s] C (TwistLeft)")
                elif cycle == 3:
                    # A - Twist Right
                    send_command("twistRight")
                    print(f"[{now:.2f}s] A (TwistRight)")

            # Advance Clock
            next_beat_time += beat_interval
            beat_counter += 1
            
        time.sleep(0.01)

if __name__ == "__main__":
    main()