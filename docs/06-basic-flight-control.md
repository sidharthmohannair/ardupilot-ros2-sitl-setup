# Module 06: Basic Flight Control with ROS2

## 1. Objectives

In this module, you will learn to control the simulated UAV using **ROS2 Python scripts**. This module provides practical examples that demonstrate autonomous flight control through the MAVROS interface.

* **Basic Control:** Understand arming, mode switching, and takeoff commands.
* **Position Control:** Send position setpoints to navigate the drone.
* **Mission Planning:** Upload and execute GPS waypoint missions.
* **Practical Examples:** Four progressively complex flight control scripts.

**Estimated Time:** 60-90 minutes

---

## 2. Prerequisites

* **Completed:** Module 01-05 [Full system setup and integration](../docs/01-ardupilot-sitl.md).
* **Running:** ArduPilot SITL + Gazebo + MAVROS (full simulation stack).
* **Knowledge:** Basic understanding of ROS2 topics and services.
* **Python:** Familiarity with Python 3 syntax (basic level sufficient).

---

## 3. Understanding Flight Control Concepts

Before writing flight control code, it's important to understand the key concepts.

### 3.1 Coordinate Frames

MAVROS uses the **ENU (East-North-Up)** coordinate frame:

```
        Z (Up)
        |
        |
        |_________ Y (North)
       /
      /
     X (East)
```

**Position coordinates:**
* **X-axis:** Positive = East, Negative = West
* **Y-axis:** Positive = North, Negative = South
* **Z-axis:** Positive = Up, Negative = Down

**Example:** Position (10, 20, 15) means:
* 10 meters East
* 20 meters North
* 15 meters Up from the takeoff point

---

### 3.2 Local vs Global Coordinates

#### **Local Coordinates (Used in Examples 1, 2, 4)**
* Relative to the drone's **takeoff position**
* Origin (0, 0, 0) = where the drone was armed
* Simple and intuitive for nearby navigation
* Topic: `/mavros/setpoint_position/local`

#### **Global Coordinates (Used in Example 3)**
* Absolute GPS coordinates (latitude, longitude, altitude)
* Required for waypoint missions
* Works over long distances
* Topic: `/mavros/mission/push`

---

### 3.3 Flight Modes

**Key ArduPilot Copter modes used in these examples:**

| Mode | Purpose | Usage |
|------|---------|-------|
| **STABILIZE** | Manual flight with stabilization | Default startup mode |
| **GUIDED** | Accept position/velocity commands from computer | Required for autonomous control |
| **AUTO** | Follow pre-programmed waypoint mission | Waypoint navigation |
| **LAND** | Automatic landing | Safe landing procedure |
| **RTL** | Return to Launch | Emergency return home |

---

### 3.4 Flight Control Sequence

Every autonomous flight follows this sequence:

```
1. Check MAVROS connection → Wait if not connected
2. Change to GUIDED mode → Wait for confirmation
3. Arm motors → Wait for armed status
4. Send flight commands → Takeoff, navigate, etc.
5. Land → Switch to LAND mode or use land command
6. Disarm → Automatic after landing
```

**Safety Note:** ArduPilot has built-in safety checks. If arming fails, check:
* GPS lock (for outdoor/real flights)
* Battery status
* Pre-arm check messages in console

---

## 4. Example 1: Simple Takeoff

**Objective:** Understand the basic flight control sequence.

**What it does:**
1. Connects to MAVROS
2. Changes to GUIDED mode
3. Arms the motors
4. Takes off to 15 meters
5. Hovers for 10 seconds
6. Lands automatically

**File:** `scripts/01_simple_takeoff.py`

### 4.1 Key Concepts Demonstrated

* Subscribing to `/mavros/state` for drone status
* Calling services: arming, mode change, takeoff
* Waiting for state changes before proceeding
* Using timers for hover duration

### 4.2 Expected Behavior

**In Gazebo:**
* Propellers start spinning after arm
* Drone lifts off smoothly to 15m
* Hovers steadily for 10 seconds
* Descends and lands at takeoff position

**In Terminal:**
```
[INFO] Drone controller started
[INFO] Waiting for MAVROS connection...
[INFO] Connected to MAVROS
[INFO] Changing to GUIDED mode...
[INFO] Mode changed to GUIDED
[INFO] Arming motors...
[INFO] Motors armed successfully
[INFO] Taking off to 15.0m...
[INFO] Takeoff command sent
[INFO] Hovering for 10 seconds...
[INFO] Landing...
[INFO] Mission complete
```

---

## 5. Example 2: Setpoint Position Control

**Objective:** Learn to send position commands for navigation.

**What it does:**
1. Takes off to 10 meters
2. Moves to position (10, 20, 10) - 10m East, 20m North
3. Holds position for 10 seconds
4. Returns to start position (0, 0, 10)
5. Lands

**File:** `scripts/02_setpoint_position.py`

### 5.1 Key Concepts Demonstrated

* Publishing to `/mavros/setpoint_position/local`
* Position setpoints are **latched** (drone continues to target even if publishing stops)
* Monitoring current position via `/mavros/local_position/pose`
* Waiting until drone reaches target before next command

### 5.2 Understanding Setpoint Commands

**Setpoint publishing:**
```python
# Send target position
msg = PoseStamped()
msg.pose.position.x = 10.0  # East
msg.pose.position.y = 20.0  # North
msg.pose.position.z = 10.0  # Up
self.setpoint_pub.publish(msg)
```

**Important:** Setpoints are "latched" - ArduPilot remembers the last command and continues navigating to it even if you stop publishing.

### 5.3 Expected Behavior

**In Gazebo:**
* Drone takes off vertically to 10m
* Flies diagonally toward (10, 20, 10)
* Hovers at target for 10 seconds
* Returns to (0, 0, 10) above takeoff point
* Descends and lands

---

## 6. Example 3: Waypoint Mission Navigation

**Objective:** Upload and execute a GPS-based waypoint mission.

**What it does:**
1. Reads GPS waypoints from `data/waypoints.txt`
2. Uploads mission to ArduPilot
3. Takes off to mission start altitude
4. Switches to AUTO mode
5. Executes waypoint mission
6. Lands at final waypoint

**File:** `scripts/03_waypoint_navigation.py`

### 6.1 Key Concepts Demonstrated

* Reading waypoints from file
* Using `/mavros/mission/push` service to upload mission
* Converting GPS coordinates to waypoint format
* Monitoring mission progress
* AUTO mode for waypoint execution

### 6.2 Waypoint File Format

**File:** `scripts/data/waypoints.txt`

```
# GPS Waypoint File
# Format: latitude, longitude, altitude (relative to home in meters)
# Lines starting with # are comments

-35.363261, 149.165237, 10
-35.363761, 149.165237, 10
-35.363761, 149.165737, 15
-35.363261, 149.165737, 15
-35.363261, 149.165237, 10
```

**Each waypoint:**
* **Latitude:** GPS latitude in decimal degrees
* **Longitude:** GPS longitude in decimal degrees
* **Altitude:** Height in meters relative to takeoff point

### 6.3 Expected Behavior

**In Gazebo:**
* Drone takes off automatically
* Flies to each GPS waypoint in sequence
* Maintains specified altitude at each point
* Lands at final waypoint location

**Note:** In simulation, the home position is typically at ArduPilot's default location (-35.363261, 149.165237). The example waypoints form a square pattern around this point.

---

## 7. Example 4: Square Pattern Flight

**Objective:** Fly a precise geometric pattern using position feedback.

**What it does:**
1. Takes off to 10 meters
2. Flies a square pattern:
   - (10, 0, 10) - East corner
   - (10, 10, 10) - Northeast corner
   - (0, 10, 10) - North corner
   - (0, 0, 10) - Back to start
3. Pauses 2 seconds at each corner
4. Lands at origin

**File:** `scripts/04_square_pattern.py`

### 7.1 Key Concepts Demonstrated

* Sequential position commands
* **Position feedback** - waiting until drone reaches each corner
* Using tolerance for position checking (within 0.5m = reached)
* Precise geometric navigation

### 7.2 Position Feedback Logic

**How it works:**
```python
# Send target position
send_position(10, 0, 10)

# Wait until drone is close to target
while distance_to_target > 0.5:  # Within 0.5m tolerance
    # Keep checking current position
    current = get_current_position()
    distance = calculate_distance(current, target)

# Drone reached! Pause at corner
time.sleep(2)

# Move to next corner
send_position(10, 10, 10)
```

### 7.3 Expected Behavior

**In Gazebo:**
* Drone takes off vertically to 10m
* Flies to first corner (10, 0, 10)
* Pauses 2 seconds (you'll see it hover steadily)
* Continues to next corner
* Repeats for all 4 corners
* Returns to center (0, 0, 10)
* Lands at takeoff position

**Pattern visualization (top view):**
```
    North (Y)
      ^
      |
  2---+---3      Numbers = corner sequence
  |       |      Each side = 10 meters
  |   O   |      O = takeoff/landing point
  |       |      
  1-------4

Start → East
```

---

## 8. Running the Scripts

**For detailed terminal commands and step-by-step instructions, see [scripts/README.md](../scripts/README.md)**

### 8.1 Quick Start

**Method 1: SITL + MAVROS (Recommended - 3 terminals)**

```bash
# Terminal 1: ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console

# Terminal 2: MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Run script
cd ~/your_workspace/scripts
python3 01_simple_takeoff.py
```

**Method 2: Add Gazebo for Visualization (Optional - 4 terminals)**

Add Terminal 1 before SITL:
```bash
# Terminal 1: Gazebo (optional)
cd ~/ardupilot_gazebo
gz sim -v4 -r iris_runway.world

# Then: SITL, MAVROS, Scripts (as above)
```

**Verification:**
```bash
# Check MAVROS connection
ros2 topic echo /mavros/state
# Should show: connected: true
```

---

### 8.2 Running Example Scripts

All scripts follow the same pattern - just change the script name:

```bash
cd ~/your_workspace/scripts

# Example 1: Simple Takeoff
python3 01_simple_takeoff.py

# Example 2: Position Control  
python3 02_setpoint_position.py

# Example 3: Waypoint Mission
python3 03_waypoint_navigation.py

# Example 4: Square Pattern
python3 04_square_pattern.py
```

---

### 8.3 Monitoring (Optional Terminal)

```bash
# Monitor state
ros2 topic echo /mavros/state

# Monitor position
ros2 topic echo /mavros/local_position/pose

# Monitor mission progress (Script 03 only)
ros2 topic echo /mavros/mission/reached
```

---

### 8.4 Safety Notes

**Stopping a script:**
- Press `Ctrl+C` in script terminal
- Manually land: Type `mode land` in MAVProxy console

**Emergency stop:**
- In MAVProxy: `disarm force` (use carefully)

---

## 9. Verification Checklist

After running each example, confirm:

### Example 1: Simple Takeoff
* [ ] Drone arms successfully
* [ ] Takeoff reaches approximately 15m altitude
* [ ] Hovers steadily for 10 seconds
* [ ] Lands at takeoff position
* [ ] No errors in terminal output

### Example 2: Setpoint Position
* [ ] Drone navigates to (10, 20, 10)
* [ ] Position hold is stable for 10 seconds
* [ ] Returns to (0, 0, 10) correctly
* [ ] Landing is at original takeoff point

### Example 3: Waypoint Navigation
* [ ] Waypoints upload successfully
* [ ] AUTO mode engages
* [ ] Drone visits all waypoints in sequence
* [ ] Maintains correct altitude at each point
* [ ] Mission completes without errors

### Example 4: Square Pattern
* [ ] All four corners are reached precisely
* [ ] 2-second pause visible at each corner
* [ ] Square pattern is clearly visible in Gazebo
* [ ] Final landing at center (0, 0)

---

## 10. Troubleshooting

### **Error: "MAVROS not connected"**

**Cause:** MAVROS is not running or not connected to SITL.

**Solution:**
1. Check MAVROS terminal for errors
2. Verify SITL is running: `ps aux | grep arducopter`
3. Check connection: `ros2 topic echo /mavros/state`
4. Restart MAVROS if needed

---

### **Error: "Arming failed"**

**Cause:** Pre-arm checks failed in ArduPilot.

**Solution:**
1. Check SITL console for pre-arm messages
2. Common issues:
   - Not in GUIDED mode (script handles this)
   - GPS not locked (shouldn't happen in simulation)
   - Parameters not loaded correctly
3. Try manual arm in MAVProxy: `arm throttle`

---

### **Error: "Drone doesn't move to setpoint"**

**Cause:** Not in GUIDED mode or setpoint not published correctly.

**Solution:**
1. Verify mode: `ros2 topic echo /mavros/state` shows `mode: GUIDED`
2. Check setpoint topic: `ros2 topic hz /mavros/setpoint_position/local`
3. Verify position topic: `ros2 topic echo /mavros/local_position/pose`
4. If stuck, restart simulation and try again

---

### **Error: "Mission upload failed" (Example 3)**

**Cause:** Waypoint format incorrect or mission service not ready.

**Solution:**
1. Verify waypoints.txt format (no extra spaces, valid GPS coordinates)
2. Check mission service: `ros2 service list | grep mission`
3. Try uploading manually through MAVProxy: `wp load waypoints.txt`

---

### **Error: "Position feedback timeout" (Example 4)**

**Cause:** Drone cannot reach target position or feedback not working.

**Solution:**
1. Increase timeout value in script (default 30 seconds)
2. Increase position tolerance (default 0.5m)
3. Check wind parameters in simulation (high wind = harder to reach)
4. Verify local position topic: `ros2 topic echo /mavros/local_position/pose`

---

### **Drone behavior is unstable**

**Cause:** PID tuning or simulation issues.

**Solution:**
1. Reset simulation (restart Gazebo and SITL)
2. Check real-time factor in Gazebo (should be close to 1.0)
3. Reduce system load (close other applications)
4. Default ArduPilot parameters should work in simulation

---

### **Script exits immediately**

**Cause:** Python dependencies missing or import errors.

**Solution:**
1. Check ROS2 environment is sourced: `printenv | grep ROS`
2. Verify MAVROS messages are installed:
```bash
ros2 interface list | grep mavros_msgs
```
3. Install if missing:
```bash
sudo apt install ros-humble-mavros-msgs
```

---

## 11. Understanding the Code

### 11.1 Common Code Patterns

All scripts follow similar patterns. Understanding these will help you write your own flight control code.

#### **Pattern 1: Waiting for State Changes**
```python
# Wait until mode changes to GUIDED
while self.current_mode != "GUIDED":
    rclpy.spin_once(self, timeout_sec=0.1)
    if timeout_exceeded:
        return False
```

#### **Pattern 2: Calling Services**
```python
# Create request
request = CommandBool.Request()
request.value = True  # True = arm, False = disarm

# Call service
future = self.arming_client.call_async(request)
rclpy.spin_until_future_complete(self, future)

# Check result
if future.result().success:
    return True
```

#### **Pattern 3: Publishing Setpoints**
```python
# Create message
msg = PoseStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = "map"
msg.pose.position.x = target_x
msg.pose.position.y = target_y
msg.pose.position.z = target_z

# Publish
self.setpoint_pub.publish(msg)
```

### 11.2 Modifying the Scripts

Want to change behavior? Here are safe modifications:

**Change takeoff altitude:**
```python
# In 01_simple_takeoff.py
self.takeoff(altitude=20.0)  # Change from 15.0 to 20.0
```

**Change target positions:**
```python
# In 02_setpoint_position.py
self.send_position(15, 25, 12)  # Different target
```

**Add more waypoints:**
```python
# In 04_square_pattern.py
waypoints = [
    (10, 0, 10),
    (10, 10, 10),
    (5, 15, 10),   # Add new points
    (0, 10, 10),
    (0, 0, 10)
]
```

---

## 12. Next Steps

After completing this module, you can:

### **Beginner Level:**
* Modify example scripts with different altitudes and positions
* Combine multiple patterns (triangle, pentagon, circle)
* Add more waypoints to missions
* Experiment with different hover times

### **Intermediate Level:**
* Create velocity-based control (use `/mavros/setpoint_velocity/cmd_vel`)
* Implement obstacle avoidance logic
* Add real-time telemetry logging to CSV
* Create a simple GUI for mission planning

### **Advanced Level:**
* Multi-drone coordination and formation flight
* Integration with computer vision (follow object)
* Path planning algorithms (A*, RRT)
* Sensor fusion and SLAM integration

---

## 13. Additional Resources

### **Official Documentation:**
* MAVROS: https://github.com/mavlink/mavros
* ArduPilot: https://ardupilot.org/copter/
* ROS2: https://docs.ros.org/en/humble/

### **Coordinate Frame Reference:**
* REP-103 (ROS coordinate frames): https://www.ros.org/reps/rep-0103.html
* MAVLink coordinate systems: https://mavlink.io/en/services/mission.html

### **Flight Modes:**
* ArduCopter flight modes: https://ardupilot.org/copter/docs/flight-modes.html

---

## 14. Summary

You have now completed Module 06 and learned:

* Basic flight control sequence (connect, mode, arm, fly, land)
* Position control using setpoints (local coordinates)
* GPS waypoint mission execution (global coordinates)
* Position feedback for precise navigation
* ROS2 service calls and topic publishing
* Safe flight practices and error handling

**Congratulations!** You now have a complete UAV simulation and control environment. You can use these examples as templates for your own autonomous flight applications.

---

[← Back Module 05](../docs/05-gazebo-ardupilot.md) | **Module 06 Complete - Ready for Autonomous Development!** | [Next: Flight Control Scripts](../scripts/README.md)

---
<p align="center">
  Made with ❤️ for the drone community<br>
  <a href="https://github.com/simtofly">SimToFly</a> • 
  <a href="https://github.com/sidharthmohannair">@sidharthmohannair</a>
</p>