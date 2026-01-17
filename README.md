# Team YMCA - Robot Rave Hackathon 2026

Making a Unitree Go1 robot dog dance to YMCA like Trump!

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   ALEX          │     │    PATWIC        │     │      MO         │
│   Music Analysis│────▶│    Robot Control │────▶│   Simulation    │
│                 │     │    & Moves       │     │   & Real Robot  │
│  - Beat detect  │     │  - Move library  │     │  - Mujoco sim   │
│  - YMCA timing  │     │  - Choreography  │     │  - Go1 connect  │
│  - Move triggers│     │  - SDK interface │     │  - Sim2Real     │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                       │                        │
        └───────────────────────┴────────────────────────┘
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
| Patwic | `patwic_control/` | Robot moves, control, choreography |
| Alex | `alex_music/` | Music analysis, beat detection, move timing |

## Data Flow

1. **Alex** analyzes YMCA song → outputs timestamped move commands
2. **Patwic** defines moves → exposes move API that takes commands
3. **Mo** runs simulation → connects to real Go1 when ready

## Shared Interface (everyone uses this!)

```python
# shared/types.py - THE CONTRACT

class Move:
    name: str        # "fist_pump", "y_pose", "trump_sway", etc.
    timestamp: float # when to execute (seconds from song start)
    duration: float  # how long the move takes
    intensity: float # 0.0 to 1.0

class Choreography:
    moves: List[Move]
    song_bpm: int    # YMCA is ~127 BPM
```

## Quick Start

```bash
# Clone and install
git clone <repo>
cd YMCA
pip install -r requirements.txt

# Each person works in their folder
cd mo_simulation/      # Mo
cd patwic_control/     # Patwic
cd alex_music/         # Alex
```

## Integration Points

- `shared/types.py` - Common data structures (DON'T CHANGE without team agreement!)
- `shared/config.py` - Robot settings, timing constants
- `main.py` - Brings everything together (we build this last)

## Git Workflow

1. Work in YOUR folder
2. Commit often with clear messages
3. Don't modify other people's folders without asking
4. `shared/` changes need team discussion

LET'S MAKE THIS DOG DANCE!
