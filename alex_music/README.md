# Alex's Audio Analysis Corner

This folder contains tools to analyze music files and generate timing data for the robot's dance moves.

## Tools

### `rhythm_parser.py`

Analyzes an audio file and exports a JSON file containing beat timestamps and energy levels.

**Usage:**

```bash
# Make sure you are in the virtual environment or have librosa installed
pip install -r ../requirements.txt

# Run the parser
python rhythm_parser.py --file path/to/song.mp3

# Visualize the beats
python3 beat_visualizer.py --json music/ymca_clustered_analysis.json --audio music/ymca_music.mp3 
```

**Output:**

A JSON file (e.g., `song_analysis.json`) with:
- `tempo`: Estimated BPM
- `events`: List of beats with timestamp, strength, and suggested move type.

## Files
- `music/`: Folder for music files.
