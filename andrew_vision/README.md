# Andrew's Zone - Computer Vision to Movement Translation

## Your Mission
1. Capture video/images of Trump dancing YMCA
2. Extract body poses and movements using computer vision
3. Translate human poses to Go1 robot movements

## Tasks

### Task 1: Pose Estimation
- [ ] Set up MediaPipe or OpenPose for body tracking
- [ ] Extract keypoints from Trump YMCA dance videos
- [ ] Output pose data as JSON/numpy arrays

### Task 2: Movement Analysis
- [ ] Identify key dance movements (fist pump, sway, etc.)
- [ ] Calculate timing between poses
- [ ] Detect YMCA letter formations

### Task 3: Robot Translation
- [ ] Map human body keypoints to Go1 joint angles
- [ ] Scale movements for quadruped (human → dog)
- [ ] Output `Move` objects for Pawit's choreography player

## Interface You Need to Provide

```python
# andrew_vision/pose_extractor.py

class PoseExtractor:
    def __init__(self, video_path: str):
        self.video_path = video_path

    def extract_poses(self) -> List[HumanPose]:
        """Extract poses from video frames"""
        pass

    def detect_ymca_letters(self) -> List[Tuple[float, str]]:
        """Detect when Y, M, C, A poses occur, return (timestamp, letter)"""
        pass


# andrew_vision/movement_translator.py

class MovementTranslator:
    def __init__(self):
        pass

    def human_to_robot(self, human_pose: HumanPose) -> Move:
        """Convert human pose to robot Move command"""
        pass

    def generate_choreography(self, poses: List[HumanPose]) -> Choreography:
        """Generate full robot choreography from human poses"""
        pass
```

## Human to Robot Mapping

```
Human Body          →    Go1 Robot
-----------------------------------------
Torso lean         →    Body pitch/roll
Arm raise (Y)      →    Stand tall + spread legs
Crouch (M)         →    Lower body height
Side bend (C)      →    Roll to side
Straight (A)       →    Neutral stance
Fist pump          →    Quick pitch oscillation
Side sway          →    Roll left/right
```

## Useful Libraries

- `mediapipe` - Google's pose estimation (fast, works on CPU)
- `opencv-python` - Video processing
- `numpy` - Pose math
- `ultralytics` - YOLO pose estimation

## Quick Start

```bash
pip install mediapipe opencv-python numpy

# Run pose extraction
python andrew_vision/pose_extractor.py --video trump_ymca.mp4
```

## Files to Create

- `pose_extractor.py` - Extract poses from video
- `movement_translator.py` - Convert human → robot moves
- `ymca_detector.py` - Detect YMCA letter poses

## Resources

- MediaPipe Pose: https://google.github.io/mediapipe/solutions/pose
- Trump YMCA videos: Search YouTube for reference
- Human pose keypoints: 33 landmarks (MediaPipe)

## Output Format

Your output should be a `Choreography` object that Pawit can use:

```python
from shared.types import Move, MoveType, Choreography

# Example output
choreography = Choreography(
    moves=[
        Move(MoveType.TRUMP_SWAY, timestamp=0.0, duration=2.0),
        Move(MoveType.FIST_PUMP, timestamp=2.0, duration=0.5),
        Move(MoveType.Y_POSE, timestamp=10.5, duration=1.0),
        # ... detected from video
    ],
    song_bpm=127
)
```
