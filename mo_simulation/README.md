# Mo's Zone - Simulation & Real Robot Connection

## Quick Start

```bash
# Run the Go1 keyboard control simulation
.venv/bin/mjpython mo_simulation/run_go1_keyboard.py
```

## Controls (Keyboard)
| Key | Action |
|-----|--------|
| W/S | Body height up/down |
| Arrow Keys | Lean forward/back/left/right |
| A/D | Yaw left/right |
| Q/E | Roll left/right |
| 1/2/3/4 | Dance poses Y/M/C/A |
| Space | Reset to stand |
| Esc | Quit |

## Your Mission
1. Get Go1 simulation working with keyboard control
2. Add more dance moves / refine existing ones
3. Connect to real Go1 robot when ready

## Go1 Joint Structure

```
Go1 has 12 joints (4 legs x 3 joints each):

FR (Front Right):  FR_hip, FR_thigh, FR_calf  [0, 1, 2]
FL (Front Left):   FL_hip, FL_thigh, FL_calf  [3, 4, 5]
RR (Rear Right):   RR_hip, RR_thigh, RR_calf  [6, 7, 8]
RL (Rear Left):    RL_hip, RL_thigh, RL_calf  [9, 10, 11]

Joint types:
- hip (abduction): side-to-side leg spread, range [-0.863, 0.863] rad
- thigh: forward/back leg swing, range [-0.686, 4.501] rad
- calf (knee): knee bend, range [-2.818, -0.888] rad
```

## Connecting to Real Go1

### Network Setup
1. Connect to Go1 via Ethernet cable
2. Set your IP to `192.168.123.xxx` (e.g., 192.168.123.100)
3. Go1's IP is `192.168.123.161`

### Unitree SDK Options

**Option 1: unitree_legged_sdk (C++/Python)**
```bash
git clone https://github.com/unitreerobotics/unitree_legged_sdk
cd unitree_legged_sdk
mkdir build && cd build
cmake ..
make
```

**Option 2: Go1 Python High-Level Control**
```python
# High-level commands (easier, less control)
from ucl.highlevel import HighLevelInterface

robot = HighLevelInterface()
robot.stand()
robot.move(vx=0.5, vy=0, vyaw=0)  # walk forward
```

**Option 3: Low-Level Control (full joint control)**
```python
# Low-level for precise dance moves
from ucl.lowlevel import LowLevelInterface

robot = LowLevelInterface()
robot.set_joint_positions([...])  # 12 joint angles
```

### Interface to Implement

```python
# mo_simulation/robot_interface.py

class RobotInterface:
    """Abstract interface - both sim and real implement this"""

    def connect(self) -> bool:
        """Connect to robot (sim or real)"""
        pass

    def set_joint_positions(self, positions: np.ndarray):
        """Set 12 joint positions in radians"""
        pass

    def get_joint_positions(self) -> np.ndarray:
        """Get current 12 joint positions"""
        pass

    def get_body_state(self) -> dict:
        """Get body position, orientation, velocity"""
        pass

    def disconnect(self):
        """Clean shutdown"""
        pass
```

## Files

- `run_go1_keyboard.py` - Keyboard control simulation (DONE)
- `robot_interface.py` - Abstract interface (TODO)
- `sim_robot.py` - MuJoCo simulation implementation (TODO)
- `real_robot.py` - Real Go1 UDP connection (TODO)

## Resources

- Go1 Model: `models/unitree_go1/`
- Unitree SDK: https://github.com/unitreerobotics/unitree_legged_sdk
- Go1 Docs: https://support.unitree.com/home/en/developer/Quick_start

## Next Steps

1. Test keyboard control in simulation
2. Refine dance poses (adjust joint angles in `compute_pose()`)
3. Create `robot_interface.py` abstract class
4. Implement `real_robot.py` when you have the Go1 connected
