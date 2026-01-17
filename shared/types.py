"""
SHARED TYPES - THE CONTRACT BETWEEN ALL TEAM MEMBERS
=====================================================
DO NOT MODIFY WITHOUT TEAM AGREEMENT!

This file defines the interfaces that connect:
- Alex's music analysis
- Pawit's robot control
- Mo's simulation
"""

from dataclasses import dataclass
from typing import List
from enum import Enum


class MoveType(Enum):
    """Available dance moves - Pawit defines these, everyone uses them"""
    # YMCA letter poses
    Y_POSE = "y_pose"
    M_POSE = "m_pose"
    C_POSE = "c_pose"
    A_POSE = "a_pose"

    # Trump signature moves
    FIST_PUMP = "fist_pump"
    TRUMP_SWAY = "trump_sway"
    WIGGLE = "wiggle"

    # Basic moves
    STAND_TALL = "stand_tall"
    CROUCH_LOW = "crouch_low"
    LEAN_LEFT = "lean_left"
    LEAN_RIGHT = "lean_right"
    SPIN_LEFT = "spin_left"
    SPIN_RIGHT = "spin_right"
    STEP_FORWARD = "step_forward"
    STEP_BACK = "step_back"

    # Neutral
    RESET = "reset"


@dataclass
class Move:
    """A single dance move command"""
    move_type: MoveType
    timestamp: float  # seconds from song start
    duration: float   # how long the move takes
    intensity: float = 1.0  # 0.0 to 1.0, affects amplitude


@dataclass
class Choreography:
    """Full dance routine - list of timed moves"""
    moves: List[Move]
    song_name: str = "YMCA"
    song_bpm: int = 127  # YMCA is approximately 127 BPM
    total_duration: float = 0.0  # calculated from moves

    def __post_init__(self):
        if self.moves:
            last_move = max(self.moves, key=lambda m: m.timestamp + m.duration)
            self.total_duration = last_move.timestamp + last_move.duration


@dataclass
class RobotState:
    """Current state of the robot - Mo provides this from sim/real"""
    body_height: float  # -0.1 to 0.1 meters
    roll: float         # radians
    pitch: float        # radians
    yaw: float          # radians
    velocity_x: float   # m/s
    velocity_y: float   # m/s
    yaw_speed: float    # rad/s
    is_standing: bool
    is_connected: bool


# YMCA Song timing constants (Alex will refine these)
YMCA_BPM = 127
YMCA_BEAT_DURATION = 60.0 / YMCA_BPM  # ~0.47 seconds per beat

# Approximate timestamps for "Y-M-C-A" in the chorus (Alex will get exact times)
YMCA_CHORUS_STARTS = [
    60.0,   # First chorus (approximate)
    120.0,  # Second chorus (approximate)
]
