#!/usr/bin/env python3
"""
Go1 Real Robot Control - YMCA Dance!

WARNING: This controls the REAL robot!
Make sure you have space around the robot.

Controls (same as simulation):
    1/2/3/4  - YMCA dance poses
    W/S      - Body height up/down
    A/D      - Yaw left/right
    Space    - Reset to stand
    Q        - Quit

Usage:
    python mo_simulation/run_go1_real.py
"""

import sys
import os
import time
import threading

# Add SDK path
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', 'unitree_legged_sdk', 'lib', 'python', 'amd64')
sys.path.append(SDK_PATH)

try:
    import robot_interface as sdk
except ImportError:
    print("ERROR: SDK not found! Build it first:")
    print("  cd unitree_legged_sdk && mkdir build && cd build")
    print("  cmake -DPYTHON_BUILD=TRUE .. && make")
    sys.exit(1)

# Robot settings
GO1_IP = "192.168.123.161"
GO1_PORT = 8082
LOCAL_PORT = 8080
HIGHLEVEL = 0xee

# Dance poses (body euler angles and height)
POSES = {
    'stand': {'euler': [0, 0, 0], 'height': 0},
    'Y': {'euler': [0, -0.2, 0], 'height': 0.1},      # Lean back, tall
    'M': {'euler': [0, 0.1, 0], 'height': -0.15},     # Lean forward, low
    'C': {'euler': [0.3, 0, 0.1], 'height': 0},       # Roll + yaw
    'A': {'euler': [0, 0, 0], 'height': 0.05},        # Straight, slight lift
    'sway_left': {'euler': [0.25, 0, 0], 'height': 0},
    'sway_right': {'euler': [-0.25, 0, 0], 'height': 0},
    'pump_up': {'euler': [0, -0.15, 0], 'height': 0.08},
    'pump_down': {'euler': [0, 0.1, 0], 'height': -0.05},
}


class Go1Controller:
    def __init__(self):
        self.udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, GO1_IP, GO1_PORT)
        self.cmd = sdk.HighCmd()
        self.state = sdk.HighState()
        self.udp.InitCmdData(self.cmd)

        self.running = True
        self.current_pose = 'stand'
        self.height_offset = 0
        self.yaw_offset = 0

    def update(self):
        """Send command to robot"""
        self.udp.Recv()
        self.udp.GetRecv(self.state)

        # Get pose
        pose = POSES.get(self.current_pose, POSES['stand'])

        # Set command
        self.cmd.mode = 1  # Forced stand
        self.cmd.euler = [
            pose['euler'][0],
            pose['euler'][1],
            pose['euler'][2] + self.yaw_offset
        ]
        self.cmd.bodyHeight = pose['height'] + self.height_offset

        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def set_pose(self, pose_name):
        """Set dance pose"""
        if pose_name in POSES:
            self.current_pose = pose_name
            print(f"Pose: {pose_name}")

    def trump_sway(self, duration=2.0):
        """Do the Trump sway!"""
        cycles = int(duration / 0.8)
        for _ in range(cycles):
            self.set_pose('sway_left')
            time.sleep(0.4)
            self.set_pose('sway_right')
            time.sleep(0.4)
        self.set_pose('stand')

    def fist_pump(self, times=3):
        """Fist pump motion"""
        for _ in range(times):
            self.set_pose('pump_up')
            time.sleep(0.2)
            self.set_pose('pump_down')
            time.sleep(0.2)
        self.set_pose('stand')

    def ymca_sequence(self):
        """Do the full YMCA!"""
        print("\n🎵 Y!")
        self.set_pose('Y')
        time.sleep(1.0)

        print("🎵 M!")
        self.set_pose('M')
        time.sleep(1.0)

        print("🎵 C!")
        self.set_pose('C')
        time.sleep(1.0)

        print("🎵 A!")
        self.set_pose('A')
        time.sleep(1.0)

        self.set_pose('stand')
        print("🎵 YMCA complete!")


def input_thread(controller):
    """Handle keyboard input"""
    print("\nControls:")
    print("  1/2/3/4 - Y/M/C/A poses")
    print("  5       - Full YMCA sequence")
    print("  6       - Trump sway")
    print("  7       - Fist pump")
    print("  W/S     - Height up/down")
    print("  A/D     - Yaw left/right")
    print("  Space   - Reset")
    print("  Q       - Quit")
    print()

    while controller.running:
        try:
            key = input().strip().lower()

            if key == 'q':
                controller.running = False
            elif key == '1':
                controller.set_pose('Y')
            elif key == '2':
                controller.set_pose('M')
            elif key == '3':
                controller.set_pose('C')
            elif key == '4':
                controller.set_pose('A')
            elif key == '5':
                controller.ymca_sequence()
            elif key == '6':
                controller.trump_sway()
            elif key == '7':
                controller.fist_pump()
            elif key == 'w':
                controller.height_offset = min(0.1, controller.height_offset + 0.03)
                print(f"Height: {controller.height_offset:.2f}")
            elif key == 's':
                controller.height_offset = max(-0.15, controller.height_offset - 0.03)
                print(f"Height: {controller.height_offset:.2f}")
            elif key == 'a':
                controller.yaw_offset += 0.1
                print(f"Yaw: {controller.yaw_offset:.2f}")
            elif key == 'd':
                controller.yaw_offset -= 0.1
                print(f"Yaw: {controller.yaw_offset:.2f}")
            elif key == ' ' or key == '':
                controller.set_pose('stand')
                controller.height_offset = 0
                controller.yaw_offset = 0
                print("Reset!")

        except EOFError:
            break


def main():
    print("=" * 50)
    print("Go1 Real Robot Control - YMCA Dance!")
    print("=" * 50)
    print("\n⚠️  WARNING: This controls the REAL robot!")
    print("Make sure the robot has space to move.\n")

    input("Press Enter to connect to Go1...")

    controller = Go1Controller()

    # Start input thread
    input_t = threading.Thread(target=input_thread, args=(controller,))
    input_t.daemon = True
    input_t.start()

    print("\n✓ Connected! Robot is in stand mode.")
    print("Type commands and press Enter.\n")

    # Main control loop
    try:
        while controller.running:
            controller.update()
            time.sleep(0.002)  # 500Hz control loop
    except KeyboardInterrupt:
        pass

    print("\nShutting down...")
    controller.cmd.mode = 0
    controller.udp.SetSend(controller.cmd)
    controller.udp.Send()
    print("Done!")


if __name__ == "__main__":
    main()
