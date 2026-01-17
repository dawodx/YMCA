# Mo's Zone - Simulation & Real Robot Connection

## Your Mission
1. Set up Mujoco simulation for Go1
2. Connect simulation to Patwic's control interface
3. Bridge to real Go1 robot when ready

## Tasks

### Task 1: Mujoco Setup
- [ ] Get Go1 MJCF/URDF model loaded in Mujoco
- [ ] Basic simulation running
- [ ] Visualization working

### Task 2: Simulation Interface
- [ ] Create `simulator.py` that Patwic's code can call
- [ ] Implement `execute_move(move: Move)` function
- [ ] Return `RobotState` after each move

### Task 3: Real Robot Bridge
- [ ] Connect to real Go1 via UDP
- [ ] Same interface as simulation (swap in/out easily)
- [ ] Safety checks before sending commands

## Interface You Need to Provide

```python
# mo_simulation/simulator.py

class RobotInterface:
    def connect(self) -> bool:
        """Connect to sim or real robot"""
        pass

    def execute_move(self, move: Move) -> RobotState:
        """Execute a move and return new state"""
        pass

    def get_state(self) -> RobotState:
        """Get current robot state"""
        pass

    def disconnect(self):
        """Clean shutdown"""
        pass
```

## Resources
- Mujoco: https://mujoco.readthedocs.io/
- Go1 URDF: https://github.com/unitreerobotics/unitree_ros
- Unitree SDK: https://github.com/unitreerobotics/unitree_legged_sdk

## Files to Create
- `simulator.py` - Main simulation class
- `real_robot.py` - Real Go1 connection
- `robot_interface.py` - Abstract interface both implement
