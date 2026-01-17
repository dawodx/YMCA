
import librosa
import numpy as np
import json
import argparse
import os
import matplotlib.pyplot as plt
import librosa.display
from sklearn.cluster import KMeans
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import silhouette_score
import warnings
warnings.filterwarnings('ignore') # Suppress sklearn warnings for cleaner output

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

    # --- Feature Extraction ---
    print("Extracting features (Loudness, Spectral Centroid, Pitch)...")
    
    # 1. Loudness (RMS)
    rms = librosa.feature.rms(y=y)[0]
    beat_rms = rms[beat_frames] if len(rms) >= len(beat_frames) else \
               np.interp(beat_frames, np.arange(len(rms)), rms)

    # 2. Spectral Centroid (Brightness)
    cent = librosa.feature.spectral_centroid(y=y, sr=sr)[0]
    beat_cent = cent[beat_frames] if len(cent) >= len(beat_frames) else \
                np.interp(beat_frames, np.arange(len(cent)), cent)
    
    # 3. Pitch (Fundamental Frequency via PYIN)
    # Note: PYIN can be slow on full tracks. 
    # For efficiency we might want to limit fmin/fmax or frame length, 
    # but defaults are usually okay for offline processing.
    f0, _, _ = librosa.pyin(y, fmin=librosa.note_to_hz('C2'), fmax=librosa.note_to_hz('C7'), sr=sr)
    
    # Fill pairs where pitch is undefined (NaN) with 0 or interpolate
    # Using 0 for unvoiced to distinguish from low pitch
    f0 = np.nan_to_num(f0) 
    
    beat_pitch = f0[beat_frames] if len(f0) >= len(beat_frames) else \
                 np.interp(beat_frames, np.arange(len(f0)), f0)

    # --- Normalization ---
    def normalize(data):
        if len(data) == 0: return data
        d_min = np.min(data)
        d_max = np.max(data)
        if d_max == d_min: return np.zeros_like(data)
        return (data - d_min) / (d_max - d_min)

    beat_strengths = normalize(beat_strengths)
    beat_rms = normalize(beat_rms)
    beat_cent = normalize(beat_cent)
    beat_pitch = normalize(beat_pitch) # Normalizing pitch might lose absolute note info, but good for relative "high/low" motion mapping
    
    # Normalize strengths 0-1
    if len(beat_strengths) > 0:
        beat_strengths = (beat_strengths - np.min(beat_strengths)) / (np.max(beat_strengths) - np.min(beat_strengths))

    # Apply filtering
    filtered_indices = []
    
    for i, strength in enumerate(beat_strengths):
        if strength < min_strength:
            continue
        filtered_indices.append(i)
        
    filtered_indices = np.array(filtered_indices)
    
    # Apply stride
    filtered_indices = filtered_indices[::stride]
    
    # Gather final arrays
    beat_times = beat_times[filtered_indices]
    beat_strengths = beat_strengths[filtered_indices]
    beat_rms = beat_rms[filtered_indices]
    beat_cent = beat_cent[filtered_indices]
    beat_pitch = beat_pitch[filtered_indices]

    # Structure the data
    choreography = []
    
    # --- Clustering ---
    print("Grouping beats using K-Means Clustering...")
    
    # improved feature vector construction
    features = np.stack((beat_strengths, beat_rms, beat_cent, beat_pitch), axis=1)
    
    # Scale features just in case, though they are already 0-1 normalized manually.
    # Using MinMaxScaler specifically to ensure everything is strictly 0-1 for KMeans
    scaler = MinMaxScaler()
    X = scaler.fit_transform(features)
    
    best_k = 0
    best_score = -1
    best_labels = []
    
    # "Try a few and see which works best"
    print("Optimizing number of clusters (K)...")
    possible_k = range(3, 9) # Try 3 to 8 clusters
    
    # Store results to print for user
    results = {}
    
    for k in possible_k:
        if len(X) < k: # Safety check for very short songs
             continue
             
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=10)
        labels = kmeans.fit_predict(X)
        score = silhouette_score(X, labels)
        results[k] = score
        
        if score > best_score:
            best_score = score
            best_k = k
            best_labels = labels
            
    print(f"\nCluster Performance (Silhouette Score):")
    for k, score in results.items():
        marker = "*" if k == best_k else ""
        print(f"  K={k}: {score:.4f} {marker}")
        
    print(f"\nSelected Optimal K={best_k} with Score={best_score:.4f}")

    # Structure the data
    choreography = []
    
    for i, (time, strength, rms_val, cent_val, pitch_val, group) in enumerate(zip(beat_times, beat_strengths, beat_rms, beat_cent, beat_pitch, best_labels)):
        choreography.append({
            "id": i,
            "timestamp": float(time),
            "strength": float(strength),
            "loudness": float(rms_val),
            "spectral_centroid": float(cent_val),
            "pitch": float(pitch_val),
            "group_id": int(group)
        })
    
    result = {
        "source_file": file_path,
        "tempo": float(tempo),
        "total_beats": len(choreography),
        "k_clusters": int(best_k),
        "clustering_score": float(best_score),
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
