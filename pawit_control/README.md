# Pawit's Zone - Robot Control & Moves

## Your Mission
1. Define all the dance moves
2. Implement move execution logic
3. Build the choreography player

## Tasks

### Task 1: Move Library
- [ ] Implement each `MoveType` from `shared/types.py`
- [ ] Define joint angles / body positions for each move
- [ ] Test moves in isolation

### Task 2: Move Execution
- [ ] Create `move_executor.py`
- [ ] Take a `Move` object → translate to robot commands
- [ ] Handle intensity parameter (bigger/smaller movements)

### Task 3: Choreography Player
- [ ] Create `player.py` that takes a `Choreography`
- [ ] Play moves at correct timestamps
- [ ] Sync with music playback

## Interface You Need to Provide

```python
# pawit_control/move_executor.py

class MoveExecutor:
    def __init__(self, robot_interface):
        """Takes Mo's robot interface"""
        self.robot = robot_interface

    def execute(self, move: Move):
        """Execute a single move on the robot"""
        pass


# pawit_control/player.py

class ChoreographyPlayer:
    def __init__(self, executor: MoveExecutor):
        self.executor = executor

    def play(self, choreography: Choreography):
        """Play full dance routine synced to timestamps"""
        pass
```

## The Moves to Implement

### YMCA Poses
- `Y_POSE` - Stand tall, body raised high
- `M_POSE` - Lower body, wiggle
- `C_POSE` - Curved lean to side
- `A_POSE` - Straight stance, slight pitch

### Trump Moves
- `FIST_PUMP` - Quick pitch up/down
- `TRUMP_SWAY` - Side to side lean (the classic!)
- `WIGGLE` - Rapid yaw oscillation

### Basic Moves
- `STAND_TALL` / `CROUCH_LOW`
- `LEAN_LEFT` / `LEAN_RIGHT`
- `SPIN_LEFT` / `SPIN_RIGHT`
- `STEP_FORWARD` / `STEP_BACK`

## Files to Create
- `moves.py` - Move definitions (angles, positions)
- `move_executor.py` - Execute moves on robot
- `player.py` - Choreography playback
