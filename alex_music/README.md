# Alex's Zone - Music to Moves

## Your Mission
1. Analyze YMCA song for beat/timing
2. Create choreography that syncs to the music
3. Output `Choreography` object for Patwic's player

## Tasks

### Task 1: Audio Analysis
- [ ] Load YMCA audio file
- [ ] Detect BPM (should be ~127)
- [ ] Find beat timestamps

### Task 2: YMCA Detection
- [ ] Find when "Y-M-C-A" is sung in chorus
- [ ] Get exact timestamps for each letter
- [ ] Find verse vs chorus sections

### Task 3: Choreography Creation
- [ ] Map song sections to moves
- [ ] Create `Choreography` object with timed `Move`s
- [ ] Match Trump's dancing style to the beat

## Interface You Need to Provide

```python
# alex_music/choreography_generator.py

class ChoreographyGenerator:
    def __init__(self, audio_file: str):
        self.audio_file = audio_file

    def analyze(self):
        """Analyze the song, find beats and YMCA timestamps"""
        pass

    def generate(self) -> Choreography:
        """Generate full choreography for the song"""
        pass

    def get_beat_times(self) -> List[float]:
        """Get all beat timestamps"""
        pass
```

## Choreography Ideas

### During Verses
- `TRUMP_SWAY` on every beat
- `FIST_PUMP` on strong beats (1 and 3)

### During "Y-M-C-A" Chorus
- `Y_POSE` when "Y" is sung
- `M_POSE` when "M" is sung
- `C_POSE` when "C" is sung
- `A_POSE` when "A" is sung

### Between Letters
- `WIGGLE`
- `SPIN_RIGHT` or `SPIN_LEFT`

## Useful Libraries
- `librosa` - Audio analysis, beat detection
- `aubio` - Real-time beat tracking
- `pydub` - Audio file handling

## Files to Create
- `audio_analyzer.py` - Beat detection, BPM
- `ymca_detector.py` - Find YMCA timestamps in song
- `choreography_generator.py` - Create the dance routine

## Sample Output

```python
Choreography(
    moves=[
        Move(MoveType.TRUMP_SWAY, timestamp=0.0, duration=1.0),
        Move(MoveType.FIST_PUMP, timestamp=1.0, duration=0.5),
        # ... during chorus:
        Move(MoveType.Y_POSE, timestamp=62.5, duration=0.8),
        Move(MoveType.M_POSE, timestamp=63.3, duration=0.8),
        Move(MoveType.C_POSE, timestamp=64.1, duration=0.8),
        Move(MoveType.A_POSE, timestamp=64.9, duration=0.8),
        # ...
    ],
    song_bpm=127
)
```
