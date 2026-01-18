"""
Shared Go1 Robot Commands

This module defines all available commands for the Go1 robot.
Both the dashboard and choreographer import from here to stay in sync.
"""

from pathlib import Path
import json

SCRIPT_DIR = Path(__file__).parent
CUSTOM_COMMANDS_FILE = SCRIPT_DIR / "custom_commands.json"
SAVED_POSES_FILE = SCRIPT_DIR / "saved_poses.json"

# All available commands with metadata
COMMANDS = {
    # Modes
    "stand": {"name": "Stand", "category": "mode", "color": "#4CAF50", "duration": 500},
    "standUp": {"name": "Stand Up", "category": "mode", "color": "#4CAF50", "duration": 1000},
    "standDown": {"name": "Sit Down", "category": "mode", "color": "#4CAF50", "duration": 1000},
    "walk": {"name": "Walk Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "run": {"name": "Run Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "climb": {"name": "Climb Mode", "category": "mode", "color": "#2196F3", "duration": 300},
    "damping": {"name": "Damping", "category": "mode", "color": "#FF9800", "duration": 500},
    "recoverStand": {"name": "Recovery", "category": "mode", "color": "#FF9800", "duration": 2000},

    # Dances (only 1 & 2 work on Go1 Pro)
    "dance1": {"name": "Dance 1", "category": "dance", "color": "#E91E63", "duration": 5000},
    "dance2": {"name": "Dance 2", "category": "dance", "color": "#E91E63", "duration": 5000},

    # Special moves (from controller: L1/L2/R1 + button combos)
    "jumpYaw": {"name": "Jump Yaw", "category": "special", "color": "#FF5722", "duration": 2000},
    "straightHand1": {"name": "Straight Hand", "category": "special", "color": "#FF5722", "duration": 2000},
    "backflip": {"name": "Backflip", "category": "special", "color": "#f44336", "duration": 3000},
    "wiggleHips": {"name": "Wiggle Hips", "category": "special", "color": "#FF5722", "duration": 2000},
    "sit": {"name": "Sit", "category": "special", "color": "#FF5722", "duration": 1500},
    "pray": {"name": "Bow", "category": "special", "color": "#FF5722", "duration": 2000},
    "stretch": {"name": "Stretch", "category": "special", "color": "#FF5722", "duration": 2000},
    "sideRoll": {"name": "Side Roll", "category": "special", "color": "#FF5722", "duration": 3000},

    # Movement
    "forward": {"name": "Forward", "category": "move", "color": "#00BCD4", "duration": 200},
    "backward": {"name": "Backward", "category": "move", "color": "#00BCD4", "duration": 200},
    "left": {"name": "Strafe L", "category": "move", "color": "#00BCD4", "duration": 200},
    "right": {"name": "Strafe R", "category": "move", "color": "#00BCD4", "duration": 200},
    "turnLeft": {"name": "Turn L", "category": "move", "color": "#00BCD4", "duration": 200},
    "turnRight": {"name": "Turn R", "category": "move", "color": "#00BCD4", "duration": 200},

    # Body pose
    "lookUp": {"name": "Look Up", "category": "pose", "color": "#607D8B", "duration": 400},
    "lookDown": {"name": "Look Down", "category": "pose", "color": "#607D8B", "duration": 400},
    "leanLeft": {"name": "Lean L", "category": "pose", "color": "#607D8B", "duration": 400},
    "leanRight": {"name": "Lean R", "category": "pose", "color": "#607D8B", "duration": 400},
    "twistLeft": {"name": "Twist L", "category": "pose", "color": "#607D8B", "duration": 400},
    "twistRight": {"name": "Twist R", "category": "pose", "color": "#607D8B", "duration": 400},
    "squat": {"name": "Squat", "category": "pose", "color": "#607D8B", "duration": 400},
    "extend": {"name": "Extend", "category": "pose", "color": "#607D8B", "duration": 400},

    # Delays for choreography
    "wait500": {"name": "Wait 0.5s", "category": "wait", "color": "#9E9E9E", "duration": 500},
    "wait1000": {"name": "Wait 1s", "category": "wait", "color": "#9E9E9E", "duration": 1000},
    "wait2000": {"name": "Wait 2s", "category": "wait", "color": "#9E9E9E", "duration": 2000},

    # LED colors
    "ledRed": {"name": "LED Red", "category": "led", "color": "#f44336", "duration": 100, "rgb": [255, 0, 0]},
    "ledGreen": {"name": "LED Green", "category": "led", "color": "#4CAF50", "duration": 100, "rgb": [0, 255, 0]},
    "ledBlue": {"name": "LED Blue", "category": "led", "color": "#2196F3", "duration": 100, "rgb": [0, 0, 255]},
    "ledYellow": {"name": "LED Yellow", "category": "led", "color": "#ffd700", "duration": 100, "rgb": [255, 215, 0]},
    "ledPink": {"name": "LED Pink", "category": "led", "color": "#E91E63", "duration": 100, "rgb": [255, 20, 147]},
    "ledCyan": {"name": "LED Cyan", "category": "led", "color": "#00BCD4", "duration": 100, "rgb": [0, 188, 212]},
    "ledWhite": {"name": "LED White", "category": "led", "color": "#FFFFFF", "duration": 100, "rgb": [255, 255, 255]},
    "ledOff": {"name": "LED Off", "category": "led", "color": "#333333", "duration": 100, "rgb": [0, 0, 0]},

    # YMCA Dance Poses
    "ymcaY": {"name": "Y Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaM": {"name": "M Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaC": {"name": "C Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaA": {"name": "A Pose", "category": "ymca", "color": "#FFD700", "duration": 1500},
    "ymcaMarch": {"name": "March", "category": "ymca", "color": "#FF9800", "duration": 2000},
    "ymcaDance": {"name": "YMCA Dance!", "category": "ymca", "color": "#E91E63", "duration": 30000},
}


def get_saved_poses():
    """Load saved poses from saved_poses.json"""
    if SAVED_POSES_FILE.exists():
        try:
            with open(SAVED_POSES_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {}


def get_all_commands():
    """Get all commands including custom ones and saved poses."""
    all_cmds = COMMANDS.copy()

    # Load custom commands if they exist
    if CUSTOM_COMMANDS_FILE.exists():
        try:
            with open(CUSTOM_COMMANDS_FILE, 'r') as f:
                custom = json.load(f)
                # Add category if missing
                for cmd_id, cmd_info in custom.items():
                    if "category" not in cmd_info:
                        cmd_info["category"] = "custom"
                    if "name" not in cmd_info:
                        cmd_info["name"] = cmd_id
                    if "color" not in cmd_info:
                        cmd_info["color"] = "#9C27B0"
                    if "duration" not in cmd_info:
                        cmd_info["duration"] = 500
                all_cmds.update(custom)
        except:
            pass

    # Load saved poses as commands
    saved_poses = get_saved_poses()
    for pose_name, pose_data in saved_poses.items():
        pose_id = f"pose_{pose_name.replace(' ', '_').lower()}"
        all_cmds[pose_id] = {
            "name": pose_name,
            "category": "saved_pose",
            "color": "#9C27B0",
            "duration": 500,
            "pose_data": pose_data,
        }

    return all_cmds


def get_commands_by_category():
    """Get commands grouped by category (for choreographer UI)."""
    all_cmds = get_all_commands()
    categories = {}

    # Define display order and names
    category_names = {
        "mode": "Modes",
        "dance": "Dance",
        "special": "Special",
        "move": "Movement",
        "pose": "Poses",
        "wait": "Timing",
        "led": "LED",
        "ymca": "YMCA",
        "custom": "Custom Cmds",
        "saved_pose": "Custom Poses",
    }

    # Define category order
    category_order = ["Modes", "Dance", "Special", "Movement", "Poses", "Timing", "LED", "YMCA", "Custom Cmds", "Custom Poses"]

    for cmd_id, cmd_info in all_cmds.items():
        cat = cmd_info.get("category", "other")
        display_name = category_names.get(cat, cat.title())
        if display_name not in categories:
            categories[display_name] = []
        categories[display_name].append(cmd_id)

    # Return in order, with any extra categories at the end
    ordered = {}
    for cat in category_order:
        if cat in categories:
            ordered[cat] = categories[cat]
    for cat in categories:
        if cat not in ordered:
            ordered[cat] = categories[cat]

    return ordered


def add_custom_command(cmd_id, name, category="custom", color="#9C27B0", duration=1000, **kwargs):
    """Add a custom command and save to file."""
    custom = {}
    if CUSTOM_COMMANDS_FILE.exists():
        try:
            with open(CUSTOM_COMMANDS_FILE, 'r') as f:
                custom = json.load(f)
        except:
            pass

    custom[cmd_id] = {
        "name": name,
        "category": category,
        "color": color,
        "duration": duration,
        **kwargs
    }

    with open(CUSTOM_COMMANDS_FILE, 'w') as f:
        json.dump(custom, f, indent=2)

    return custom[cmd_id]
