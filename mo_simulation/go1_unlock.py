#!/usr/bin/env python3
"""
Go1 SDK Mode Unlocker

SSH into the Go1 and kill the process that blocks special moves.
After running this, dance1/dance2/jumpYaw will work without L1+L2+START!

WARNING: This modifies robot behavior. Use at your own risk.
"""

import subprocess
import sys

# Go1 Raspberry Pi credentials
GO1_IP = "192.168.12.1"
GO1_USER = "pi"
GO1_PASS = "123"


def run_ssh_command(command):
    """Run command on Go1 via SSH."""
    ssh_cmd = f"sshpass -p '{GO1_PASS}' ssh -o StrictHostKeyChecking=no {GO1_USER}@{GO1_IP} '{command}'"
    result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr, result.returncode


def main():
    print("\n" + "=" * 50)
    print("  GO1 SDK MODE UNLOCKER")
    print("=" * 50)
    print(f"\n  Connecting to Go1 at {GO1_IP}...")
    print("  User: pi, Password: 123")
    print("\n  This will kill mqttControlNode to unlock:")
    print("  - dance1, dance2")
    print("  - jumpYaw")
    print("  - straightHand1")
    print("=" * 50)

    # Check if sshpass is installed
    result = subprocess.run("which sshpass", shell=True, capture_output=True)
    if result.returncode != 0:
        print("\nERROR: sshpass not installed!")
        print("Install with: brew install hudochenkov/sshpass/sshpass")
        sys.exit(1)

    input("\nPress ENTER to unlock SDK mode (Ctrl+C to cancel)...")

    print("\n[1/3] Testing SSH connection...")
    stdout, stderr, code = run_ssh_command("echo 'Connected!'")
    if code != 0:
        print(f"ERROR: SSH failed - {stderr}")
        print("Make sure you're connected to Go1's WiFi!")
        sys.exit(1)
    print("  OK - Connected to Go1")

    print("\n[2/3] Checking running processes...")
    stdout, stderr, code = run_ssh_command("pgrep -a mqttControlNode || echo 'Not running'")
    print(f"  mqttControlNode: {stdout.strip()}")

    stdout, stderr, code = run_ssh_command("pgrep -a appTransit || echo 'Not running'")
    print(f"  appTransit: {stdout.strip()}")

    print("\n[3/3] Killing mqttControlNode to unlock special moves...")
    stdout, stderr, code = run_ssh_command("sudo pkill -f mqttControlNode; echo 'Done'")
    if "Done" in stdout:
        print("  OK - mqttControlNode killed!")
    else:
        print(f"  Warning: {stderr}")

    print("\n" + "=" * 50)
    print("  SDK MODE UNLOCKED!")
    print("=" * 50)
    print("\n  Now run: python mo_simulation/go1_sdk_control.py")
    print("  Press 1 for Dance 1, 2 for Dance 2, 4 for Jump Yaw!")
    print("\n  NOTE: Robot will need restart to restore normal operation.")
    print("=" * 50 + "\n")


if __name__ == "__main__":
    main()
