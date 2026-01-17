# Team YMCA - Robot Rave Hackathon 2026

Making a Unitree Go1 robot dog dance to YMCA like Trump!

## Quick Start

```bash
# Clone the repo
git clone https://github.com/dawodx/YMCA.git
cd YMCA

# Run setup (creates venv + installs dependencies)
./setup.sh

# Activate environment
source .venv/bin/activate

# Launch the disco UI
python launcher.py
# Then go to http://localhost:8889

# Or run simulation directly
.venv/bin/mjpython mo_simulation/run_go1_keyboard.py
```

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   ALEX          │     │    PAWIT        │     │      MO         │
│   Music Analysis│────▶│    Robot Control │────▶│   Simulation    │
│                 │     │    & Moves       │     │   & Real Robot  │
│  - Beat detect  │     │  - Move library  │     │  - Mujoco sim   │
│  - YMCA timing  │     │  - Choreography  │     │  - Go1 connect  │
│  - Move triggers│     │  - SDK interface │     │  - Sim2Real     │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                       ▲                        │
        │               ┌───────┴────────┐               │
        │               │    ANDREW      │               │
        │               │  Vision → Move │               │
        │               │  - Pose extract│               │
        │               │  - Trump dance │               │
        │               │  - Human→Robot │               │
        │               └────────────────┘               │
        └────────────────────────┬───────────────────────┘
                                 │
                          ┌──────▼──────┐
                          │   SHARED    │
                          │  - Types    │
                          │  - Config   │
                          │  - Constants│
                          └─────────────┘
```

## Team Responsibilities

| Member | Folder | Focus |
|--------|--------|-------|
| Mo | `mo_simulation/` | Mujoco sim, real robot connection |
| Pawit | `pawit_control/` | Robot moves, control, choreography |
| Alex | `alex_music/` | Music analysis, beat detection, move timing |
| Andrew | `andrew_vision/` | Computer vision, pose extraction, human→robot translation |

## Project Structure

```
YMCA/
├── setup.sh                 # Quick setup script
├── launcher.py              # Web server for disco UI
├── launcher.html            # Disco-themed launcher
├── requirements.txt         # Python dependencies
├── models/
│   └── unitree_go1/         # Go1 robot model (MuJoCo)
├── shared/                  # Shared types & config
│   ├── types.py             # Move, Choreography, RobotState
│   └── config.py            # Robot settings
├── mo_simulation/           # Mo's zone
│   └── run_go1_keyboard.py  # Keyboard control demo
├── pawit_control/           # Pawit's zone
├── alex_music/              # Alex's zone
└── andrew_vision/           # Andrew's zone
```

## Useful Resources

### Unitree Go1
- **Unitree SDK (Python)**: https://github.com/unitreerobotics/unitree_legged_sdk
- **Go1 ROS Package**: https://github.com/unitreerobotics/unitree_ros
- **Go1 Documentation**: https://support.unitree.com/home/en/developer/Quick_start
- **Go1 URDF/MJCF**: Included in `models/unitree_go1/`

### MuJoCo Simulation
- **MuJoCo Docs**: https://mujoco.readthedocs.io/
- **MuJoCo Menagerie** (robot models): https://github.com/google-deepmind/mujoco_menagerie
- **MuJoCo Python**: https://mujoco.readthedocs.io/en/stable/python.html

### Computer Vision (Andrew)
- **MediaPipe Pose**: https://google.github.io/mediapipe/solutions/pose
- **OpenPose**: https://github.com/CMU-Perceptual-Computing-Lab/openpose

### Audio Analysis (Alex)
- **Librosa** (beat detection): https://librosa.org/doc/latest/index.html
- **YMCA by Village People**: ~127 BPM

### Real Robot Connection
```bash
# Network setup
# 1. Connect Go1 via Ethernet
# 2. Set your IP: 192.168.123.xxx
# 3. Go1 IP: 192.168.123.161

# Clone Unitree SDK
git clone https://github.com/unitreerobotics/unitree_legged_sdk
cd unitree_legged_sdk
mkdir build && cd build
cmake ..
make
```

## Shared Interface

Everyone uses types from `shared/types.py`:

```python
from shared.types import Move, MoveType, Choreography

# Example: Create a dance move
move = Move(
    move_type=MoveType.TRUMP_SWAY,
    timestamp=0.0,
    duration=2.0,
    intensity=1.0
)

# Example: Create choreography
dance = Choreography(
    moves=[move, ...],
    song_bpm=127
)
```

## Tools & Dashboards

### Launcher (http://localhost:8889)
```bash
python launcher.py
```
Disco-themed UI to launch all tools.

### Choreography Editor (http://localhost:8894)
```bash
python mo_simulation/go1_choreographer.py
```
Visual timeline editor for dance choreography:
- Waveform visualization of YMCA music
- Drag & drop move markers on timeline
- Zoom in/out like Adobe Premiere
- Click timeline to seek & add moves
- Debug mode: Press SPACE to flag timestamps while playing
- Save/Load choreography JSON files
- Sections: INTRO, YOUNG MAN, CHORUS mapped to timeline
- Real-time move highlighting during playback

### Visual Dashboard (http://localhost:8890)
```bash
python mo_simulation/go1_dashboard.py
```
Control panel for the real Go1 robot:
- All robot commands (Dance1, Dance2, JumpYaw, poses)
- LED color picker
- Sequence recording & playback
- Action log with timing stats
- Connect to robot via WiFi

### Pose Builder (http://localhost:8892)
```bash
python mo_simulation/go1_pose_builder.py
```
Create custom poses with joint sliders and save them.

### Parameter Editor (http://localhost:8893)
```bash
python mo_simulation/go1_param_editor.py
```
Adjust command parameters (intensity, speed, duration) and save as defaults.

### MuJoCo Simulation
```bash
.venv/bin/mjpython mo_simulation/run_go1_keyboard.py
```
Keyboard-controlled simulation for testing.

## Controls (Keyboard Demo)

| Key | Action |
|-----|--------|
| W/S | Body height up/down |
| Arrow Keys | Lean forward/back/left/right |
| A/D | Yaw left/right |
| 1/2/3/4 | YMCA dance poses |
| Space | Reset to stand |
| Esc | Quit |

## Git Workflow

1. Work in YOUR folder
2. Commit often with clear messages
3. Don't modify other people's folders without asking
4. `shared/` changes need team discussion
5. Pull before you push!

```bash
git pull
# ... make changes ...
git add -A
git commit -m "Description of changes"
git push
```

---

**SOTA COCA Hackathon London 2026**

LET'S MAKE THIS DOG DANCE! 🐕🎵
