# Go1 Real Robot Connection (WiFi - No Ethernet Needed!)

## Step 1: Connect to Go1's WiFi

1. **Turn on your Go1** (stand it up, press power button)
2. **Look for WiFi network** on your Mac:
   - SSID contains: `Unitree_Go` or similar
   - Password: `00000000` (eight zeros) OR `88888888` (eight eights)
3. **Connect your Mac** to this WiFi network

## Step 2: Check Connection

Once connected, your Mac should get an IP like `192.168.12.x` or `192.168.123.x`

```bash
# Check your IP
ifconfig | grep "192.168"

# Ping the robot
ping 192.168.123.161
```

If ping works, you're connected!

## Step 3: Run Test Script

```bash
cd /Users/dawod/YMCA
source .venv/bin/activate
python mo_simulation/test_go1_connection.py
```

## Network Architecture

```
Your Mac (192.168.123.xxx)
        │
        │  WiFi
        ▼
┌─────────────────────────────────────┐
│        Go1 Internal Network         │
│                                     │
│  Main Board:    192.168.123.161     │  ◄── SDK connects here!
│  Raspberry Pi:  192.168.123.161     │
│  Nano Head:     192.168.123.13      │
│  Nano Body 1:   192.168.123.14      │
│  Nano Body 2:   192.168.123.15      │
└─────────────────────────────────────┘
```

## SDK Commands (High-Level)

```python
# Modes
cmd.mode = 0  # Idle, default stand
cmd.mode = 1  # Forced stand (locked position)
cmd.mode = 2  # Walk continuously
cmd.mode = 5  # Position stand down
cmd.mode = 6  # Position stand up

# Gait types (when mode=2)
cmd.gaitType = 0  # Idle
cmd.gaitType = 1  # Trot
cmd.gaitType = 2  # Trot running

# Body control
cmd.euler = [roll, pitch, yaw]  # radians, for body tilt
cmd.bodyHeight = 0.1            # -0.2 to 0.1 meters
cmd.velocity = [vx, vy]         # -1 to 1 m/s
cmd.yawSpeed = 0.5              # rad/s
```

## Troubleshooting

**Can't find WiFi?**
- Make sure Go1 is fully powered on (takes ~30 seconds)
- Try both password options (00000000 or 88888888)

**Ping fails?**
- Check you're on the Go1 WiFi, not your home WiFi
- Try `192.168.12.1` instead of `192.168.123.161`

**SDK import fails?**
- Make sure you built the Python wrapper (see below)

## Building SDK Python Wrapper

```bash
cd unitree_legged_sdk
mkdir build && cd build
cmake -DPYTHON_BUILD=TRUE ..
make
```

Then copy `robot_interface*.so` to your project.
