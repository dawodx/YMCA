"""
SHARED CONFIGURATION
====================
Robot settings and constants used by all team members
"""

# Unitree Go1 Network Settings
GO1_IP = "192.168.123.161"  # Default Go1 IP when connected via Ethernet
GO1_PORT = 8082

# Mujoco Simulation Settings
SIM_TIMESTEP = 0.002  # 2ms simulation step
SIM_REALTIME = True   # Run simulation in real-time

# Robot Physical Limits (don't exceed these!)
MAX_BODY_HEIGHT = 0.12   # meters
MIN_BODY_HEIGHT = -0.10  # meters
MAX_ROLL = 0.4           # radians
MAX_PITCH = 0.4          # radians
MAX_YAW = 0.4            # radians
MAX_VELOCITY = 0.5       # m/s
MAX_YAW_SPEED = 2.0      # rad/s

# Dance Settings
DEFAULT_MOVE_DURATION = 0.5  # seconds
TRANSITION_TIME = 0.1        # time between moves
