import librosa
import numpy as np
import json
import os

# ==========================================
#        MASTER CONFIGURATION - EDIT THIS
# ==========================================

# 1. RHYTHM OVERRIDES
MANUAL_BPM = 129.0   # Standard Village People BPM
USE_MANUAL_BPM = True 

# 2. STRUCTURE OVERRIDES (Start Time, End Time)
# Robot will do Y-M-C-A poses during these times.
# Robot will do CPG Walking during all other times.
YMCA_SECTIONS = [
    {"start": 45.0,  "end": 76.0,  "name": "CHORUS_1"}, # "It's fun to stay..."
    {"start": 110.0, "end": 140.0, "name": "CHORUS_2"},
    {"start": 175.0, "end": 200.0, "name": "CHORUS_OUTRO"}
]

# ==========================================

def analyze_manual_override(file_path, output_path=None):
    print(f"Loading {file_path}...")
    y, sr = librosa.load(file_path)

    # --- 1. ESTABLISH THE GRID ---
    if USE_MANUAL_BPM:
        print(f"⚠️ Forcing Manual BPM: {MANUAL_BPM}")
        
        # We still ask Librosa to find the FIRST beat to act as our anchor/offset
        tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr)
        first_beat_time = librosa.frames_to_time(beat_frames[0], sr=sr)
        
        print(f"Detected First Beat at: {first_beat_time:.2f}s")
        
        # Mathematically generate all subsequent beats
        # This prevents "drift" and ensures perfect 1-2-3-4 timing
        duration = librosa.get_duration(y=y, sr=sr)
        seconds_per_beat = 60.0 / MANUAL_BPM
        
        beat_times = []
        current_time = first_beat_time
        while current_time < duration:
            beat_times.append(current_time)
            current_time += seconds_per_beat
            
        tempo = MANUAL_BPM
    else:
        # Fallback to auto-detection
        print("Using Auto-Detection...")
        tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr)
        beat_times = librosa.frames_to_time(beat_frames, sr=sr)
        tempo = float(tempo) if np.ndim(tempo) == 0 else float(tempo[0])

    # --- 2. MAP EVENTS & DRUMS ---
    structured_events = []
    
    # We group beats into Bars (4 beats = 1 Bar)
    beats_per_bar = 4
    
    for i, t in enumerate(beat_times):
        # A. Determine Drum Type for Lights
        # Beat 0, 2 (1st and 3rd) = KICK (Heavy)
        # Beat 1, 3 (2nd and 4th) = SNARE (Snap)
        beat_in_bar = i % beats_per_bar
        drum_type = "KICK" if beat_in_bar % 2 == 0 else "SNARE"
        
        # B. Determine Section (Chorus vs Verse)
        is_chorus = False
        current_section_name = "VERSE"
        
        for section in YMCA_SECTIONS:
            if section["start"] <= t <= section["end"]:
                is_chorus = True
                current_section_name = "CHORUS"
                break
        
        # C. Calculate Bar Index
        bar_index = i // beats_per_bar

        # Only save the START of a bar as a "Structure Event" to keep JSON small?
        # NO, for lights we want EVERY beat.
        structured_events.append({
            "timestamp": float(t),
            "beat_index": i,          # Global beat count
            "beat_in_bar": beat_in_bar, # 0, 1, 2, 3
            "bar_index": bar_index,
            "type": current_section_name, # VERSE or CHORUS
            "drum": drum_type         # KICK or SNARE
        })

    # --- 3. SAVE ---
    result = {
        "source_file": file_path,
        "tempo": tempo,
        "total_beats": len(structured_events),
        "events": structured_events
    }

    if output_path is None:
        output_path = os.path.splitext(file_path)[0] + "_structure.json"
        
    with open(output_path, "w") as f:
        json.dump(result, f, indent=4)
        
    print(f"Generated {len(structured_events)} beat events.")
    print(f"Saved to {output_path}")

if __name__ == "__main__":
    analyze_manual_override("alex_music/music/ymca_music.mp3")