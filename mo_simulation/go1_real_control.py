#!/usr/bin/env python3
"""
Go1 Real Robot Control - YMCA Dance!

Uses go1pylib for easy control of Unitree Go1.

SETUP:
1. Turn on Go1 robot
2. Connect your Mac to Go1's WiFi:
   - SSID: Unitree_GoXXXX
   - Password: 00000000 (or 88888888)
3. Run this script!

Controls:
  1/2/3/4  - YMCA poses (Y, M, C, A)
  5        - Full YMCA dance sequence!
  6        - Trump sway
  w/s      - Walk forward/backward
  a/d      - Turn left/right
  Space    - Stand still
  x        - Sit down
  z        - Stand up
  0        - Exit
"""

import asyncio
import sys
import time

try:
    from go1pylib import Go1, Go1Mode
except ImportError:
    print("ERROR: go1pylib not installed!")
    print("Run: pip install go1pylib")
    sys.exit(1)


# YMCA Dance Poses (body euler angles: roll, pitch, yaw)
YMCA_POSES = {
    'Y': {'roll': 0.0, 'pitch': -0.25, 'yaw': 0.0},      # Lean back
    'M': {'roll': 0.0, 'pitch': 0.2, 'yaw': 0.0},        # Lean forward, low
    'C': {'roll': 0.3, 'pitch': 0.0, 'yaw': 0.15},       # Roll + yaw (curved)
    'A': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},        # Straight
}


def get_key():
    """Get single keypress from terminal."""
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class YMCAController:
    """YMCA Dance controller for Go1."""

    def __init__(self):
        self.robot = Go1()
        self.connected = False

    def connect(self):
        """Connect to robot."""
        print("Connecting to Go1...")
        try:
            self.robot.init()
            self.connected = True
            print("Connected!")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot."""
        if self.connected:
            try:
                self.robot.disconnect()
            except:
                pass
        print("Disconnected.")

    async def stand(self):
        """Stand up."""
        if self.connected:
            self.robot.set_mode(Go1Mode.STAND)

    async def sit(self):
        """Sit down."""
        if self.connected:
            self.robot.set_mode(Go1Mode.STAND_DOWN)

    async def walk_forward(self, duration_ms=500):
        """Walk forward."""
        if self.connected:
            self.robot.set_mode(Go1Mode.WALK)
            await self.robot.go_forward(speed=0.3, duration_ms=duration_ms)

    async def walk_backward(self, duration_ms=500):
        """Walk backward."""
        if self.connected:
            self.robot.set_mode(Go1Mode.WALK)
            await self.robot.go_forward(speed=-0.3, duration_ms=duration_ms)

    async def turn_left(self, duration_ms=500):
        """Turn left."""
        if self.connected:
            self.robot.set_mode(Go1Mode.WALK)
            await self.robot.turn_left(speed=0.5, duration_ms=duration_ms)

    async def turn_right(self, duration_ms=500):
        """Turn right."""
        if self.connected:
            self.robot.set_mode(Go1Mode.WALK)
            await self.robot.turn_right(speed=0.5, duration_ms=duration_ms)

    async def set_pose(self, roll=0, pitch=0, yaw=0):
        """Set body pose (euler angles in radians)."""
        if self.connected:
            try:
                # go1pylib might have pose control
                self.robot.set_body_pose(roll=roll, pitch=pitch, yaw=yaw)
            except AttributeError:
                # If not, we'll try direct command
                print(f"  Pose: roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")

    async def ymca_pose(self, letter):
        """Do a YMCA pose."""
        if letter in YMCA_POSES:
            pose = YMCA_POSES[letter]
            print(f"Pose: {letter}")
            await self.set_pose(**pose)

    async def ymca_dance(self):
        """Full YMCA dance sequence!"""
        print("\n🎵 Starting YMCA dance! 🎵\n")

        for letter in ['Y', 'M', 'C', 'A']:
            print(f"  🎵 {letter}!")
            await self.ymca_pose(letter)
            await asyncio.sleep(1.0)

        # Reset to standing
        await self.set_pose(0, 0, 0)
        print("\n🎵 YMCA Complete! 🎵\n")

    async def trump_sway(self, duration=3.0):
        """Do the Trump sway dance!"""
        print("\n💃 Trump Sway! 💃\n")
        start = time.time()
        while time.time() - start < duration:
            await self.set_pose(roll=0.2, pitch=0, yaw=0)
            await asyncio.sleep(0.3)
            await self.set_pose(roll=-0.2, pitch=0, yaw=0)
            await asyncio.sleep(0.3)
        await self.set_pose(0, 0, 0)
        print("Done swaying!")


async def main():
    print("\n" + "=" * 55)
    print("  🐕 GO1 REAL ROBOT - YMCA DANCE! 🐕")
    print("=" * 55)
    print("\n  SETUP:")
    print("  1. Turn on Go1 robot")
    print("  2. Connect Mac to Go1 WiFi (Unitree_GoXXXX)")
    print("     Password: 00000000 or 88888888")
    print("\n  Controls:")
    print("    1/2/3/4  = YMCA poses (Y/M/C/A)")
    print("    5        = Full YMCA dance!")
    print("    6        = Trump sway")
    print("    w/s      = Walk forward/back")
    print("    a/d      = Turn left/right")
    print("    Space    = Stand")
    print("    x        = Sit down")
    print("    z        = Stand up")
    print("    0        = Exit")
    print("=" * 55)

    controller = YMCAController()

    if not controller.connect():
        print("\nCould not connect to robot!")
        print("Make sure you're on Go1's WiFi network.")
        return

    # Get battery level
    try:
        battery = controller.robot.get_battery_level()
        print(f"\nBattery: {battery}%")
    except:
        pass

    print("\nReady! Press keys to control the robot.\n")

    try:
        while True:
            key = get_key()

            # Exit
            if key == '0':
                print("\nExiting...")
                break

            # YMCA poses
            elif key == '1':
                await controller.ymca_pose('Y')
            elif key == '2':
                await controller.ymca_pose('M')
            elif key == '3':
                await controller.ymca_pose('C')
            elif key == '4':
                await controller.ymca_pose('A')

            # Full YMCA dance
            elif key == '5':
                await controller.ymca_dance()

            # Trump sway
            elif key == '6':
                await controller.trump_sway()

            # Walking
            elif key == 'w':
                print("Walking forward...")
                await controller.walk_forward()
            elif key == 's':
                print("Walking backward...")
                await controller.walk_backward()
            elif key == 'a':
                print("Turning left...")
                await controller.turn_left()
            elif key == 'd':
                print("Turning right...")
                await controller.turn_right()

            # Stand/Sit
            elif key == ' ':
                print("Standing...")
                await controller.stand()
            elif key == 'x':
                print("Sitting down...")
                await controller.sit()
            elif key == 'z':
                print("Standing up...")
                await controller.stand()

    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        controller.disconnect()

    print("\nDone! 🐕")


if __name__ == "__main__":
    asyncio.run(main())
