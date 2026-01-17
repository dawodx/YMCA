import librosa
import numpy as np
import json
import argparse
import os
import matplotlib.pyplot as plt
import librosa.display

def analyze_music(file_path, output_path=None, plot=False):
    print(f"Loading {file_path}...")
    y, sr = librosa.load(file_path)

    print("Analyzing rhythm...")
    # 1. Get the steady beat (The "Pulse")
    tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr)
    print(f"Estimated tempo: {float(tempo.item()):.2f} BPM")

    # 2. Get the strength of the audio at all points
    onset_env = librosa.onset.onset_strength(y=y, sr=sr)
    
    # 3. Sample the strength ONLY at the beat locations
    # This aligns the "energy" measure with the "rhythm"
    beat_times = librosa.frames_to_time(beat_frames, sr=sr)
    beat_strengths = onset_env[beat_frames]

    # Normalize beat strengths to 0.0 - 1.0 range
    if len(beat_strengths) > 0:
        beat_strengths = (beat_strengths - np.min(beat_strengths)) / (np.max(beat_strengths) - np.min(beat_strengths))

    choreography = []
    # Lower threshold because we are looking at average beat energy now
    strong_threshold = 0.32

    for i, (time, strength) in enumerate(zip(beat_times, beat_strengths)):
        move_type = "STEP" # Default waddle
        
        # If the beat is loud (chorus/kick drum), make it a distinct move
        if strength > strong_threshold:
            move_type = "JUMP" 
            
        choreography.append({
            "id": i,
            "timestamp": float(time),
            "strength": float(strength),
            "type": move_type
        })

    result = {
        "source_file": file_path,
        "tempo": float(tempo) if np.ndim(tempo) == 0 else float(tempo[0]),
        "events": choreography
    }

    if output_path is None:
        output_path = os.path.splitext(file_path)[0] + "_analysis.json"
        
    with open(output_path, "w") as f:
        json.dump(result, f, indent=4)
    print(f"Saved {len(choreography)} dance moves to {output_path}")

    if plot:
        plt.figure(figsize=(14, 5))
        librosa.display.waveshow(y, sr=sr, alpha=0.6)
        # Plot vertical lines at beat times
        plt.vlines(beat_times, -1, 1, color='r', alpha=0.5, linestyle='--', label='Beats')
        plt.title(f'Waveform and Beats: {os.path.basename(file_path)}')
        plt.tight_layout()
        plt.savefig(output_path.replace('.json', '.png'))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, required=True)
    parser.add_argument("--plot", action='store_true')
    args = parser.parse_args()
    
    if os.path.exists(args.file):
        analyze_music(args.file, plot=args.plot)