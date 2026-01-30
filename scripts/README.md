# Flight Control Scripts - Quick Reference

This directory contains Python scripts for autonomous UAV control using ROS2 and MAVROS.

---

## Running the Scripts

### Method 1: SITL + MAVROS (Recommended for Testing)

**Simple setup - 3 terminals required**

#### Terminal 1: Launch ArduPilot SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

#### Terminal 2: Launch MAVROS
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

#### Terminal 3: Run Script
```bash
cd ~/your_workspace/scripts

# Example 1: Simple Takeoff
python3 01_simple_takeoff.py

# Example 2: Setpoint Position
python3 02_setpoint_position.py

# Example 3: Waypoint Navigation
python3 03_waypoint_navigation.py

# Example 4: Square Pattern
python3 04_square_pattern.py
```

#### Terminal 4 (Optional): Monitor Topics
```bash
# Monitor state
ros2 topic echo /mavros/state

# Monitor position
ros2 topic echo /mavros/local_position/pose

# Monitor mission progress (Script 03)
ros2 topic echo /mavros/mission/reached
```

---

### Method 2: SITL + MAVROS + Gazebo (Optional - For Visualization)

**Use this if you want to see the drone in 3D simulation**

**4 terminals required**

#### Terminal 1: Launch Gazebo
```bash
cd ~/ardupilot_gazebo
gz sim -v4 -r iris_runway.world
```

#### Terminal 2: Launch ArduPilot SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

#### Terminal 3: Launch MAVROS
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

#### Terminal 4: Run Script
```bash
cd ~/your_workspace/scripts
python3 01_simple_takeoff.py
```

**Note:** Gazebo is only for visualization. Scripts work the same with or without it.

---

## Script Descriptions

### 01_simple_takeoff.py
**What it does:**
- Takes off to 15 meters
- Hovers for 10 seconds
- Lands automatically

**Duration:** ~30 seconds

---

### 02_setpoint_position.py
**What it does:**
- Takes off to 10 meters
- Flies to position (10, 20, 10) - 10m East, 20m North
- Holds for 10 seconds
- Returns to (0, 0, 10)
- Lands

**Duration:** ~60 seconds

---

### 03_waypoint_navigation.py
**What it does:**
- Reads GPS waypoints from `data/waypoints.txt`
- Arms and takes off in GUIDED mode
- Uploads mission waypoints
- Switches to AUTO mode
- Executes waypoint mission
- Returns to launch (RTL) after last waypoint

**Duration:** Depends on waypoint count and distances

**File required:** `data/waypoints.txt`

---

### 04_square_pattern.py
**What it does:**
- Takes off to 10 meters
- Flies a 10m × 10m square pattern
- Pauses 2 seconds at each corner
- Returns to center
- Lands

**Pattern:**
```
    2-------3      Altitude: 10m
    |       |      Side: 10m
    |   O   |      O = takeoff/landing
    |       |
    1-------4
```

**Duration:** ~90 seconds

---

## Verification

**Check MAVROS connection before running scripts:**
```bash
ros2 topic echo /mavros/state
# Should show: connected: true
```

**Check topics are publishing:**
```bash
ros2 topic hz /mavros/state
ros2 topic hz /mavros/local_position/pose
```

---

## Stopping a Script

**During execution:**
- Press `Ctrl+C` in the script terminal

**Manual landing (if needed):**
```bash
# In MAVProxy (Terminal 1)
mode land
```

**Emergency disarm (use carefully):**
```bash
# In MAVProxy
disarm force
```

---

## Troubleshooting

### "MAVROS not connected"
```bash
# Check MAVROS is running
ros2 node list | grep mavros

# Check connection
ros2 topic echo /mavros/state

# Restart MAVROS if needed (Terminal 2)
```

### "Arming rejected"
**Check MAVProxy console for pre-arm errors**
```bash
# In MAVProxy (Terminal 1)
arm check
```

Common issues:
- EKF not initialized (wait 10 seconds after SITL starts)
- Not in GUIDED mode (scripts handle this)

### "Position timeout" (Scripts 02, 04)
- Increase timeout in script
- Check position topic: `ros2 topic echo /mavros/local_position/pose`

### "Mission upload failed" (Script 03)
- Verify `data/waypoints.txt` exists
- Check file format (see below)

---

## Waypoint File Format

**File:** `data/waypoints.txt`

```
# GPS Waypoint File
# Format: latitude, longitude, altitude (meters)
# Lines starting with # are comments

-35.363261, 149.165787, 10
-35.362811, 149.165787, 10
-35.362811, 149.165237, 15
-35.362811, 149.164687, 15
-35.363261, 149.165237, 10
```

**Format:**
- Latitude: Decimal degrees
- Longitude: Decimal degrees  
- Altitude: Meters relative to home (takeoff point)

---

## Coordinate System

All scripts use **ENU (East-North-Up)** frame:

```
    Z (Up)
    |
    |
    |_______ Y (North)
   /
  /
 X (East)
```

**Position (10, 20, 5) means:**
- 10 meters East
- 20 meters North
- 5 meters Up from takeoff point

---

## File Structure

```
scripts/
├── README.md                    # This file
├── 01_simple_takeoff.py
├── 02_setpoint_position.py
├── 03_waypoint_navigation.py
├── 04_square_pattern.py
└── data/
    └── waypoints.txt
```

---

## Next Steps

After running the examples:
- Modify altitudes and positions
- Create new waypoint files
- Combine patterns
- Add sensor integration

---

## Additional Resources

- **Module 06 Documentation:** Full explanations and concepts
- **MAVROS Wiki:** http://wiki.ros.org/mavros
- **ArduPilot Docs:** https://ardupilot.org/copter/

---

**Ready to fly!** Start with `01_simple_takeoff.py` 🚁
