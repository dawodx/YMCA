#!/usr/bin/env python3
"""
Test Go1 Connection via WiFi
Run this to verify you can talk to the real robot!

Usage:
    python mo_simulation/test_go1_connection.py
"""

import sys
import os
import time
import socket

# Add SDK path
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', 'unitree_legged_sdk', 'lib', 'python', 'amd64')
sys.path.append(SDK_PATH)

GO1_IP = "192.168.123.161"
GO1_PORT = 8082
LOCAL_PORT = 8080

def check_network():
    """Check if we can reach the Go1"""
    print("=" * 50)
    print("Go1 Connection Test")
    print("=" * 50)
    print(f"\nTarget: {GO1_IP}:{GO1_PORT}")

    # Try to create a socket connection
    print("\n[1] Testing network connectivity...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2)

    try:
        # Try to send a UDP packet
        sock.sendto(b"test", (GO1_IP, GO1_PORT))
        print(f"    ✓ Can send UDP to {GO1_IP}")
    except Exception as e:
        print(f"    ✗ Network error: {e}")
        print("\n    Make sure you're connected to Go1's WiFi!")
        print("    WiFi SSID: Unitree_Go... or similar")
        print("    Password: 00000000 or 88888888")
        return False
    finally:
        sock.close()

    return True


def check_sdk():
    """Check if SDK is available"""
    print("\n[2] Checking SDK...")

    try:
        import robot_interface as sdk
        print("    ✓ SDK loaded successfully")
        return sdk
    except ImportError as e:
        print(f"    ✗ SDK not found: {e}")
        print("\n    Build the SDK first:")
        print("    cd unitree_legged_sdk")
        print("    mkdir build && cd build")
        print("    cmake -DPYTHON_BUILD=TRUE ..")
        print("    make")
        return None


def test_connection(sdk):
    """Try to connect and read state"""
    print("\n[3] Connecting to Go1...")

    HIGHLEVEL = 0xee

    try:
        udp = sdk.UDP(HIGHLEVEL, LOCAL_PORT, GO1_IP, GO1_PORT)
        cmd = sdk.HighCmd()
        state = sdk.HighState()
        udp.InitCmdData(cmd)

        print("    ✓ UDP connection established")

        # Try to receive state
        print("\n[4] Reading robot state...")

        for i in range(10):
            udp.Recv()
            udp.GetRecv(state)
            time.sleep(0.1)

        # Print state info
        print(f"\n    IMU Roll:  {state.imu.rpy[0]:.3f} rad")
        print(f"    IMU Pitch: {state.imu.rpy[1]:.3f} rad")
        print(f"    IMU Yaw:   {state.imu.rpy[2]:.3f} rad")

        print("\n" + "=" * 50)
        print("✓ SUCCESS! Go1 is connected and responding!")
        print("=" * 50)

        return True

    except Exception as e:
        print(f"    ✗ Connection failed: {e}")
        return False


def main():
    # Step 1: Check network
    if not check_network():
        print("\n✗ Network check failed. Connect to Go1 WiFi first!")
        return

    # Step 2: Check SDK
    sdk = check_sdk()
    if sdk is None:
        print("\n✗ SDK not available. Build it first!")
        return

    # Step 3: Test connection
    if test_connection(sdk):
        print("\nYou're ready to make this dog dance!")
        print("Run: python mo_simulation/run_go1_real.py")
    else:
        print("\n✗ Could not connect to Go1")
        print("Make sure the robot is on and you're on its WiFi")


if __name__ == "__main__":
    main()
