#!/usr/bin/env python3
"""
Go1 Real Robot Control - Standard SDK Movements

Uses go1pylib for easy control with all standard Unitree movements.

SETUP:
1. Turn on Go1 robot
2. Connect your Mac to Go1's WiFi:
   - SSID: Unitree_GoXXXX
   - Password: 00000000 (or 88888888)
3. Run this script!
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


def get_key():
    """Get single keypress from terminal (non-blocking style)."""
    import tty
    import termios
    import select

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        # Check if input is available (with short timeout)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch = sys.stdin.read(1)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class Go1Controller:
    """Controller for Go1 using go1pylib with all standard SDK movements."""

    def __init__(self):
        self.robot = Go1()
        self.connected = False
        self.current_mode = None

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
        print("Disconnected.")

    # ============== MODE COMMANDS (sync - these work immediately) ==============

    def stand(self):
        """Default stand."""
        self.robot.set_mode(Go1Mode.STAND)
        self.current_mode = "STAND"
        print("Mode: STAND")

    def stand_up(self):
        """Stand up from sitting."""
        self.robot.set_mode(Go1Mode.STAND_UP)
        self.current_mode = "STAND_UP"
        print("Mode: STAND UP")

    def stand_down(self):
        """Sit down."""
        self.robot.set_mode(Go1Mode.STAND_DOWN)
        self.current_mode = "STAND_DOWN"
        print("Mode: STAND DOWN (sit)")

    def walk_mode(self):
        """Enter walk mode."""
        self.robot.set_mode(Go1Mode.WALK)
        self.current_mode = "WALK"
        print("Mode: WALK")

    def run_mode(self):
        """Enter run mode (faster)."""
        self.robot.set_mode(Go1Mode.RUN)
        self.current_mode = "RUN"
        print("Mode: RUN (fast)")

    def climb_mode(self):
        """Enter climb/stair mode."""
        self.robot.set_mode(Go1Mode.CLIMB)
        self.current_mode = "CLIMB"
        print("Mode: CLIMB (stairs)")

    def damping(self):
        """Enter damping mode (soft/limp)."""
        self.robot.set_mode(Go1Mode.DAMPING)
        self.current_mode = "DAMPING"
        print("Mode: DAMPING (soft)")

    def recovery(self):
        """Recovery stand (from fallen)."""
        self.robot.set_mode(Go1Mode.RECOVER_STAND)
        self.current_mode = "RECOVER"
        print("Mode: RECOVERY STAND")

    def dance1(self):
        """Dance routine 1."""
        self.robot.set_mode(Go1Mode.DANCE1)
        self.current_mode = "DANCE1"
        print("Mode: DANCE 1!")

    def dance2(self):
        """Dance routine 2."""
        self.robot.set_mode(Go1Mode.DANCE2)
        self.current_mode = "DANCE2"
        print("Mode: DANCE 2!")

    def straight_hand(self):
        """Straight hand pose."""
        self.robot.set_mode(Go1Mode.STRAIGHT_HAND1)
        self.current_mode = "STRAIGHT_HAND"
        print("Mode: STRAIGHT HAND")

    # ============== MOVEMENT COMMANDS (async - need await) ==============

    async def go_forward(self, speed=0.4):
        """Walk forward - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_forward(speed=speed, duration_ms=100)

    async def go_backward(self, speed=0.4):
        """Walk backward - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_backward(speed=speed, duration_ms=100)

    async def go_left(self, speed=0.3):
        """Strafe left - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_left(speed=speed, duration_ms=100)

    async def go_right(self, speed=0.3):
        """Strafe right - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_right(speed=speed, duration_ms=100)

    async def turn_left(self, speed=0.6):
        """Turn left - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.turn_left(speed=speed, duration_ms=100)

    async def turn_right(self, speed=0.6):
        """Turn right - short burst."""
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.turn_right(speed=speed, duration_ms=100)

    # ============== BODY POSE COMMANDS ==============

    def look_up(self):
        """Look up (pitch up)."""
        self.robot.look_up()
        print("Look up")

    def look_down(self):
        """Look down (pitch down)."""
        self.robot.look_down()
        print("Look down")

    def lean_left(self):
        """Lean left (roll)."""
        self.robot.lean_left()
        print("Lean left")

    def lean_right(self):
        """Lean right (roll)."""
        self.robot.lean_right()
        print("Lean right")

    def twist_left(self):
        """Twist left (yaw)."""
        self.robot.twist_left()
        print("Twist left")

    def twist_right(self):
        """Twist right (yaw)."""
        self.robot.twist_right()
        print("Twist right")

    def squat_down(self):
        """Lower body height."""
        self.robot.squat_down()
        print("Squat down")

    def extend_up(self):
        """Raise body height."""
        self.robot.extend_up()
        print("Extend up")

    def reset_body(self):
        """Reset body pose to neutral."""
        self.robot.reset_body()
        print("Reset body pose")

    # ============== LED COMMANDS ==============

    def led_red(self):
        """Set LED to red."""
        self.robot.set_led_color(255, 0, 0)
        print("LED: RED")

    def led_green(self):
        """Set LED to green."""
        self.robot.set_led_color(0, 255, 0)
        print("LED: GREEN")

    def led_blue(self):
        """Set LED to blue."""
        self.robot.set_led_color(0, 0, 255)
        print("LED: BLUE")

    def led_off(self):
        """Turn LED off."""
        self.robot.set_led_color(0, 0, 0)
        print("LED: OFF")


async def main():
    print("\n" + "=" * 60)
    print("  GO1 SDK CONTROL - Standard Unitree Movements")
    print("=" * 60)
    print("\n  SETUP:")
    print("  1. Turn on Go1 robot")
    print("  2. Connect Mac to Go1 WiFi (Unitree_GoXXXX)")
    print("     Password: 00000000 or 88888888")
    print("\n  MOVEMENT (hold key for continuous):")
    print("    w/s      = Forward/Backward")
    print("    a/d      = Turn left/right")
    print("    q/e      = Strafe left/right")
    print("\n  MODES:")
    print("    Space    = Stand (stop)")
    print("    z        = Stand up")
    print("    x        = Sit down (stand down)")
    print("    c        = Recovery stand")
    print("    v        = Damping (soft/limp)")
    print("    r        = Run mode (fast)")
    print("    t        = Climb/stair mode")
    print("\n  SPECIAL MOVES:")
    print("    1        = Dance 1")
    print("    2        = Dance 2")
    print("    3        = Straight hand pose")
    print("\n  BODY POSE:")
    print("    i/k      = Look up/down (pitch)")
    print("    j/l      = Lean left/right (roll)")
    print("    u/o      = Twist left/right (yaw)")
    print("    [/]      = Squat down / Extend up")
    print("    b        = Reset body pose")
    print("\n  LED:")
    print("    7/8/9    = Red/Green/Blue")
    print("\n    0        = EXIT")
    print("=" * 60)

    robot = Go1Controller()

    if not robot.connect():
        print("\nCould not connect to robot!")
        print("Make sure you're on Go1's WiFi network.")
        return

    # Start in standing mode
    await asyncio.sleep(0.5)
    robot.stand()

    print("\nReady! Press keys to control the robot.\n")

    try:
        while True:
            key = get_key()

            if key is None:
                await asyncio.sleep(0.01)  # Small delay when no input
                continue

            # Exit
            if key == '0':
                print("\nExiting...")
                break

            # Movement (async)
            elif key == 'w':
                print("Forward")
                await robot.go_forward(0.5)
            elif key == 's':
                print("Backward")
                await robot.go_backward(0.5)
            elif key == 'a':
                print("Turn left")
                await robot.turn_left(0.7)
            elif key == 'd':
                print("Turn right")
                await robot.turn_right(0.7)
            elif key == 'q':
                print("Strafe left")
                await robot.go_left(0.4)
            elif key == 'e':
                print("Strafe right")
                await robot.go_right(0.4)

            # Modes (sync - instant)
            elif key == ' ':
                robot.stand()
            elif key == 'z':
                robot.stand_up()
            elif key == 'x':
                robot.stand_down()
            elif key == 'c':
                robot.recovery()
            elif key == 'v':
                robot.damping()
            elif key == 'r':
                robot.run_mode()
            elif key == 't':
                robot.climb_mode()

            # Special moves
            elif key == '1':
                robot.dance1()
            elif key == '2':
                robot.dance2()
            elif key == '3':
                robot.straight_hand()

            # Body pose
            elif key == 'i':
                robot.look_up()
            elif key == 'k':
                robot.look_down()
            elif key == 'j':
                robot.lean_left()
            elif key == 'l':
                robot.lean_right()
            elif key == 'u':
                robot.twist_left()
            elif key == 'o':
                robot.twist_right()
            elif key == '[':
                robot.squat_down()
            elif key == ']':
                robot.extend_up()
            elif key == 'b':
                robot.reset_body()

            # LED
            elif key == '7':
                robot.led_red()
            elif key == '8':
                robot.led_green()
            elif key == '9':
                robot.led_blue()

    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        robot.stand()
        await asyncio.sleep(0.2)
        robot.disconnect()

    print("\nDone!")


if __name__ == "__main__":
    asyncio.run(main())
