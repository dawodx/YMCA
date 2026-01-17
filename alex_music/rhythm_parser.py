
import librosa
import numpy as np
import json
import argparse
import os
import matplotlib.pyplot as plt
import librosa.display

def analyze_music(file_path, output_path=None, plot=False, stride=1, min_strength=0.0):
    """
    Analyzes the music file to extract beat timings and potentially structural events.
    args:
        file_path: Path to the audio file.
        output_path: Path to save the extracted JSON data.
    """
    print(f"Loading {file_path}...")
    # Load the audio as a waveform `y`
    # Store the sampling rate as `sr`
    y, sr = librosa.load(file_path)

    print("Analyzing rhythm...")
    # Run the default beat tracker
    tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr)
    
    # In newer librosa, tempo might be an array
    if np.ndim(tempo) > 0:
        tempo = tempo[0]
        
    print(f"Estimated tempo: {tempo:.2f} BPM")

    # Convert the frame indices of beat events into timestamps
    beat_times = librosa.frames_to_time(beat_frames, sr=sr)

    # Calculate onset strength to determine 'energy' of each beat
    onset_env = librosa.onset.onset_strength(y=y, sr=sr)
    
    # Get strength at beat locations roughly
    # We need to map beat frames to onset envelope frames. 
    # Librosa's beat_track returns frames in the same timeline as onset_env usually.
    # beat_frames are indices into the STFT frames.
    
    beat_strengths = onset_env[beat_frames]
    
    # Normalize strengths 0-1
    if len(beat_strengths) > 0:
        beat_strengths = (beat_strengths - np.min(beat_strengths)) / (np.max(beat_strengths) - np.min(beat_strengths))

    # Apply filtering
    filtered_times = []
    filtered_strengths = []
    
    for i, (time, strength) in enumerate(zip(beat_times, beat_strengths)):
        if strength < min_strength:
            continue
        
        filtered_times.append(time)
        filtered_strengths.append(strength)
        
    # Apply stride
    filtered_times = filtered_times[::stride]
    filtered_strengths = filtered_strengths[::stride]
    
    beat_times = np.array(filtered_times)
    beat_strengths = np.array(filtered_strengths)

    # Structure the data
    choreography = []
    
    # Simple logic: High energy beats might be special moves
    strong_threshold = 0.6
    
    for i, (time, strength) in enumerate(zip(beat_times, beat_strengths)):
        move_type = "STEP"
        if strength > strong_threshold:
            move_type = "JUMP" # Placeholder for a strong move
            
        choreography.append({
            "id": i,
            "timestamp": float(time),
            "strength": float(strength),
            "type": move_type
        })

    result = {
        "source_file": file_path,
        "tempo": float(tempo),
        "total_beats": len(choreography),
        "events": choreography
    }

    if output_path is None:
        base_name = os.path.splitext(file_path)[0]
        output_path = base_name + "_analysis.json"
    
    with open(output_path, "w") as f:
        json.dump(result, f, indent=4)
        
    print(f"Analysis saved to {output_path}")

    if plot:
        print("Generating plot...")
        plt.figure(figsize=(14, 5))
        librosa.display.waveshow(y, sr=sr, alpha=0.6)
        plt.vlines(beat_times, -1, 1, color='r', alpha=0.9, linestyle='--', label='Beats')
        plt.legend()
        plt.title(f'Waveform and Beats: {os.path.basename(file_path)}\n(Stride: {stride}, Min Strength: {min_strength})')
        plt.tight_layout()
        
        plot_path = output_path.replace('.json', '.png')
        plt.savefig(plot_path)
        print(f"Plot saved to {plot_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze a song for robot choreography.")
    parser.add_argument("--file", type=str, required=True, help="Path to the audio file")
    parser.add_argument("--out", type=str, help="Output JSON path")
    parser.add_argument("--plot", action='store_true', help="Generate a plot of the analysis")
    parser.add_argument("--stride", type=int, default=1, help="Take every Nth beat (default: 1)")
    parser.add_argument("--min-strength", type=float, default=0.0, help="Minimum beat strength (0.0-1.0) to keep")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.file):
        print(f"Error: File {args.file} not found.")
        exit(1)
        
    analyze_music(args.file, args.out, args.plot, args.stride, args.min_strength)
