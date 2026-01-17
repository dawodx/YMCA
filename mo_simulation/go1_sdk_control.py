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
        if select.select([sys.stdin], [], [], 0.02)[0]:
            ch = sys.stdin.read(1)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class Go1Controller:
    """Controller for Go1 using go1pylib."""

    def __init__(self):
        self.robot = Go1()
        self.connected = False

    def connect(self):
        """Connect to robot via MQTT."""
        print("Connecting to Go1 via MQTT...")
        try:
            self.robot.init()
            self.connected = True
            print("Connected!")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    # ============== MODE COMMANDS ==============

    def set_mode(self, mode, name):
        """Set mode and print."""
        self.robot.set_mode(mode)
        print(f"Mode: {name}")

    def stand(self):
        self.set_mode(Go1Mode.STAND, "STAND")

    def stand_up(self):
        self.set_mode(Go1Mode.STAND_UP, "STAND UP")

    def stand_down(self):
        self.set_mode(Go1Mode.STAND_DOWN, "SIT DOWN")

    def walk_mode(self):
        self.set_mode(Go1Mode.WALK, "WALK")

    def run_mode(self):
        self.set_mode(Go1Mode.RUN, "RUN")

    def climb_mode(self):
        self.set_mode(Go1Mode.CLIMB, "CLIMB")

    def damping(self):
        self.set_mode(Go1Mode.DAMPING, "DAMPING")

    def recovery(self):
        self.set_mode(Go1Mode.RECOVER_STAND, "RECOVERY")

    def dance1(self):
        self.set_mode(Go1Mode.DANCE1, "DANCE 1")

    def dance2(self):
        self.set_mode(Go1Mode.DANCE2, "DANCE 2")

    def straight_hand(self):
        self.set_mode(Go1Mode.STRAIGHT_HAND1, "STRAIGHT HAND")

    def jump_yaw(self):
        # Send raw MQTT command for jumpYaw
        self.robot.mqtt.client.publish("controller/action", "jumpYaw", qos=1)
        print("Mode: JUMP YAW")

    # ============== MOVEMENT (async) ==============

    async def go_forward(self):
        print("Forward")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_forward(speed=0.5, duration_ms=150)

    async def go_backward(self):
        print("Backward")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_backward(speed=0.5, duration_ms=150)

    async def go_left(self):
        print("Strafe left")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_left(speed=0.4, duration_ms=150)

    async def go_right(self):
        print("Strafe right")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.go_right(speed=0.4, duration_ms=150)

    async def turn_left(self):
        print("Turn left")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.turn_left(speed=0.6, duration_ms=150)

    async def turn_right(self):
        print("Turn right")
        self.robot.set_mode(Go1Mode.WALK)
        await self.robot.turn_right(speed=0.6, duration_ms=150)

    # ============== BODY POSE (async, requires STAND mode) ==============

    async def look_up(self):
        print("Look up")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.look_up(speed=0.5, duration_ms=300)

    async def look_down(self):
        print("Look down")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.look_down(speed=0.5, duration_ms=300)

    async def lean_left(self):
        print("Lean left")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.lean_left(speed=0.5, duration_ms=300)

    async def lean_right(self):
        print("Lean right")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.lean_right(speed=0.5, duration_ms=300)

    async def twist_left(self):
        print("Twist left")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.twist_left(speed=0.5, duration_ms=300)

    async def twist_right(self):
        print("Twist right")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.twist_right(speed=0.5, duration_ms=300)

    async def squat_down(self):
        print("Squat down")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.squat_down(speed=0.5, duration_ms=300)

    async def extend_up(self):
        print("Extend up")
        self.robot.set_mode(Go1Mode.STAND)
        await asyncio.sleep(0.1)
        await self.robot.extend_up(speed=0.5, duration_ms=300)

    async def reset_body(self):
        print("Reset body")
        await self.robot.reset_body()

    # ============== LED ==============

    def led_red(self):
        self.robot.set_led_color(255, 0, 0)
        print("LED: RED")

    def led_green(self):
        self.robot.set_led_color(0, 255, 0)
        print("LED: GREEN")

    def led_blue(self):
        self.robot.set_led_color(0, 0, 255)
        print("LED: BLUE")


async def main():
    print("\n" + "=" * 60)
    print("  GO1 SDK CONTROL - Standard Unitree Movements")
    print("=" * 60)
    print("\n  SETUP:")
    print("  1. Connect Mac to Go1 WiFi")
    print("  2. For DANCE modes: Press L1+L2+START on controller first!")
    print("     (This enables SDK Mode 2 required for special moves)")
    print("\n  MOVEMENT:")
    print("    w/s      = Forward/Backward")
    print("    a/d      = Turn left/right")
    print("    q/e      = Strafe left/right")
    print("\n  MODES:")
    print("    Space    = Stand (stop)")
    print("    z        = Stand up")
    print("    x        = Sit down")
    print("    c        = Recovery stand")
    print("    v        = Damping (soft)")
    print("    f        = Walk mode")
    print("    r        = Run mode")
    print("    t        = Climb mode")
    print("\n  SPECIAL (need L1+L2+START first!):")
    print("    1        = Dance 1")
    print("    2        = Dance 2")
    print("    3        = Straight hand")
    print("    4        = Jump Yaw")
    print("\n  BODY POSE (in stand mode):")
    print("    i/k      = Look up/down")
    print("    j/l      = Lean left/right")
    print("    u/o      = Twist left/right")
    print("    [/]      = Squat/Extend")
    print("    b        = Reset body")
    print("\n  LED: 7=Red, 8=Green, 9=Blue")
    print("\n    0        = EXIT")
    print("=" * 60)

    robot = Go1Controller()

    if not robot.connect():
        print("\nCould not connect!")
        print("Make sure you're on Go1's WiFi.")
        return

    await asyncio.sleep(0.5)
    robot.stand()

    print("\nReady! Press keys to control.\n")

    try:
        while True:
            key = get_key()

            if key is None:
                await asyncio.sleep(0.01)
                continue

            # Exit
            if key == '0':
                print("\nExiting...")
                break

            # Movement
            elif key == 'w':
                await robot.go_forward()
            elif key == 's':
                await robot.go_backward()
            elif key == 'a':
                await robot.turn_left()
            elif key == 'd':
                await robot.turn_right()
            elif key == 'q':
                await robot.go_left()
            elif key == 'e':
                await robot.go_right()

            # Modes
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
            elif key == 'f':
                robot.walk_mode()
            elif key == 'r':
                robot.run_mode()
            elif key == 't':
                robot.climb_mode()

            # Special (need L1+L2+START first!)
            elif key == '1':
                robot.dance1()
            elif key == '2':
                robot.dance2()
            elif key == '3':
                robot.straight_hand()
            elif key == '4':
                robot.jump_yaw()

            # Body pose
            elif key == 'i':
                await robot.look_up()
            elif key == 'k':
                await robot.look_down()
            elif key == 'j':
                await robot.lean_left()
            elif key == 'l':
                await robot.lean_right()
            elif key == 'u':
                await robot.twist_left()
            elif key == 'o':
                await robot.twist_right()
            elif key == '[':
                await robot.squat_down()
            elif key == ']':
                await robot.extend_up()
            elif key == 'b':
                await robot.reset_body()

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

    print("\nDone!")


if __name__ == "__main__":
    asyncio.run(main())
