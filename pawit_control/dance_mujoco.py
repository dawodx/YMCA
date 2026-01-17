#!/usr/bin/env python3
"""
YMCA Smart Choreographer

1. State Machine: Determines song section (Intro, Verse, Chorus) based on time.
2. Beat Engine: Calculates 126 BPM beats in real-time.
3. Logic:
   - CHORUS: Triggers high-level 'dance1' or 'dance2'.
   - VERSE/INTRO: Cycles through low-level primitives (twist, lean) on the beat.
"""

import time
import requests
import pygame
from enum import Enum
from pathlib import Path

# --- CONFIG ---
DASHBOARD_URL = "http://localhost:8891/api/cmd" # Make sure port matches your dashboard
AUDIO_FILE = "alex_music/music/ymca_music.mp3"
BPM = 126.0

# --- DEFINITIONS ---
class SongState(Enum):
    INTRO = 1
    YOUNG_MAN = 2  # Verse
    CHORUS = 3
    OUTRO = 4

# Primitives to cycle through during verses
# We create "Move Sets" for variety
MOVES_INTRO = ["twistLeft", "twistRight"]
MOVES_VERSE = ["twistLeft", "twistRight", "squat", "extend"]
MOVES_OUTRO = ["twistLeft", "twistRight"]

def get_song_state(t_ms):
    """Returns the current section of the song based on your timestamps."""
    if t_ms < 28000:
        return SongState.INTRO
    elif t_ms < 45000:
        return SongState.YOUNG_MAN
    elif t_ms < 60000:
        return SongState.CHORUS
    elif t_ms < 91000:
        return SongState.YOUNG_MAN
    elif t_ms < 124000:
        return SongState.CHORUS
    elif t_ms < 156000:
        return SongState.YOUNG_MAN
    elif t_ms < 189000:
        return SongState.CHORUS
    else:
        return SongState.OUTRO

def send_command(cmd_name):
    """Sends command to the dashboard API."""
    try:
        url = f"{DASHBOARD_URL}/{cmd_name}"
        # Print mostly for debug, but keep console clean
        # print(f"--> {cmd_name}") 
        requests.post(url, timeout=0.05) 
    except:
        pass # Ignore timeouts to keep rhythm

def main():
    print(f"--- YMCA SMART CHOREOGRAPHER ({BPM} BPM) ---")
    
    if not Path(AUDIO_FILE).exists():
        print(f"Error: {AUDIO_FILE} not found")
        return

    # Setup Audio
    pygame.init()
    pygame.mixer.init()
    try:
        pygame.mixer.music.load(AUDIO_FILE)
    except Exception as e:
        print(f"Audio Error: {e}")
        return

    input("Press Enter to START...")
    print("3...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1... GO!")
    
    pygame.mixer.music.play()
    start_time = time.time()
    
    # State tracking
    last_state = None
    beat_interval = 65.0 / BPM # Seconds per beat
    next_beat_time = 0.0
    beat_counter = 0
    
    # For cycling moves
    move_index = 0

    while True:
        if not pygame.mixer.music.get_busy():
            print("Music finished.")
            break
            
        # 1. Get Time
        # pygame.mixer.music.get_pos() returns ms, converting to seconds
        # We use time.time() relative to start for smoother beat calc
        now = time.time() - start_time
        now_ms = now * 1000
        
        # 2. Get State
        current_state = get_song_state(now_ms)
        
        # 3. Handle State Changes (Transitions)
        if current_state != last_state:
            print(f"\n[{now:.1f}s] >>> SWITCHING TO {current_state.name}")
            
            if current_state == SongState.CHORUS:
                # Enter Chorus Mode
                send_command("ledPink")
                send_command("dance1") 
                
            elif current_state == SongState.YOUNG_MAN:
                # Enter Verse Mode
                send_command("ledGreen")
                send_command("stand") # Stop dancing, get ready to groove
                
            elif current_state == SongState.OUTRO:
                send_command("ledOff")
                send_command("standDown")
                
            last_state = current_state

        # 4. Handle Beats (Rhythmic Actions)
        if now >= next_beat_time:
            # We hit a beat!
            
            if current_state == SongState.INTRO:
                # Slow moves on beat
                cmd = MOVES_INTRO[move_index % len(MOVES_INTRO)]
                send_command(cmd)
                print(f"[{now:.2f}s] INTRO: {cmd}")
                move_index += 1
                
            elif current_state == SongState.YOUNG_MAN:
                # Verse moves (Twisting/Leaning)
                cmd = MOVES_VERSE[move_index % len(MOVES_VERSE)]
                send_command(cmd)
                print(f"[{now:.2f}s] VERSE: {cmd}")
                move_index += 1
                
            elif current_state == SongState.CHORUS:
                # In chorus, the 'dance1' command mostly handles itself.
                # But 'dance1' might time out after 5 seconds (depending on dashboard).
                # Let's re-trigger specific dance moves every 4 bars (approx 8s) or switch it up.
                if beat_counter % 16 == 0: # Every ~8 seconds
                    send_command("dance2")
                    print(f"[{now:.2f}s] CHORUS: Switch to Dance 2")
                elif beat_counter % 16 == 8:
                    send_command("dance1")
                    print(f"[{now:.2f}s] CHORUS: Switch to Dance 1")
                
                # Flash LEDs on the beat for extra flair
                if beat_counter % 2 == 0:
                    send_command("ledRed")
                else:
                    send_command("ledBlue")

            # Schedule next beat
            next_beat_time += beat_interval
            beat_counter += 1
            
        time.sleep(0.01) # High frequency loop

if __name__ == "__main__":
    main()