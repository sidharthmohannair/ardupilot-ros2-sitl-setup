# Module 05: ArduPilot-Gazebo Integration

## 1. Objectives

In this module, you will integrate **ArduPilot SITL** with **Gazebo Harmonic** using the `ardupilot_gazebo` plugin. This creates a complete UAV simulation environment with flight control, physics, and visualization.

* **Plugin Installation:** Build and install the ardupilot_gazebo plugin.
* **Environment Configuration:** Set up environment variables for Gazebo to locate plugins and models.
* **Full System Test:** Launch ArduPilot SITL with Gazebo visualization.
* **Validation:** Perform a complete system test with 3D visualization and telemetry.

**Estimated Time:** 25-30 minutes

---

## 2. Prerequisites

* **Completed:** Module 01 [ArduPilot SITL installed](../docs/01-ardupilot-sitl.md).
* **Completed:** Module 02 [ROS 2 installed](../docs/02-ros2-system.md).
* **Completed:** Module 03 [MAVROS installed](../docs/03-mavros-bridge.md).
* **Completed:** Module 04 [Gazebo installed](../docs/04-gazebo-simulator.md).
* **Verified:** ArduPilot SITL can run independently.
* **Verified:** Gazebo can launch and render 3D graphics.

---

## 3. Understanding the Integration

The integration consists of three main components:

1. **ArduPilot SITL**: Flight controller simulator (provides autopilot logic)
2. **Gazebo Plugin**: Connects ArduPilot to Gazebo via JSON protocol
3. **Gazebo Simulator**: Provides physics, rendering, and sensor simulation

**Communication Flow:**
```
ArduPilot SITL <--JSON--> Gazebo Plugin <--Internal--> Gazebo Physics/Rendering
```

**Important:** This plugin is independent of ROS2. ROS2 connectivity comes later through MAVROS.

---

## 4. Install Plugin Dependencies

### 4.1 Update Package Lists

```bash
sudo apt update
```

### 4.2 Install Build Tools and Libraries

For Gazebo Harmonic (libgz-sim8-dev):

```bash
sudo apt install libgz-sim8-dev rapidjson-dev -y
```

### 4.3 Install Optional Dependencies (Camera/Video Streaming)

These are optional but recommended for camera simulation:

```bash
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```

---

## 5. Build ardupilot_gazebo Plugin

### 5.1 Clone the Repository

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
```

### 5.2 Set Gazebo Version Environment Variable

```bash
export GZ_VERSION=harmonic
```

**Note:** This tells the build system we're using Gazebo Harmonic.

### 5.3 Build the Plugin

```bash
mkdir build

cd build

cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo

make -j4
```

**Expected Output:**
```
-- Configuring done
-- Generating done
-- Build files have been written to: /home/username/ardupilot_gazebo/build
[ 25%] Building CXX object CMakeFiles/ArduPilotPlugin.dir/src/ArduPilotPlugin.cc.o
[ 50%] Building CXX object CMakeFiles/ArduCopterIRLockPlugin.dir/src/ArduCopterIRLockPlugin.cc.o
...
[100%] Built target ArduPilotPlugin
```

**Build Time:** Approximately 2-5 minutes depending on your system.

**Common Build Warnings:** You may see some compiler warnings. These are usually safe to ignore as long as the build completes successfully.

---

## 6. Configure Environment Variables

### 6.1 Add Plugin and Model Paths

Gazebo needs to know where to find the plugin and models. Add these to your `~/.bashrc`:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

### 6.2 Reload Environment

```bash
source ~/.bashrc
```

### 6.3 Verify Environment Variables

```bash
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
echo $GZ_SIM_RESOURCE_PATH
```

**Expected Output:**
```
/home/username/ardupilot_gazebo/build:
/home/username/ardupilot_gazebo/models:/home/username/ardupilot_gazebo/worlds:
```

---

## 7. Verification: Full System Test

Now we'll test the complete integration. You'll need **two separate terminal windows**.

### Terminal 1: Launch Gazebo with Iris Quadcopter

```bash
cd ~/ardupilot_gazebo
gz sim -v4 -r iris_runway.world
```

### Terminal 2: Launch ArduPilot SITL

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

**In the MAVProxy console (this same Terminal 2), try these commands:**

```
mode guided

arm throttle

takeoff 5
```

**Expected Behavior:**

1. **Mode Guided:** Console shows `GUIDED>` prompt
2. **Arm Throttle:** Propellers start spinning in Gazebo
3. **Takeoff 5:** Quadcopter lifts off to 5 meters altitude in Gazebo

**In Gazebo Window:**

* You should see the Iris quadcopter physically take off
* Propellers should be visibly spinning
* Physics simulation should look smooth and realistic

---

## 8. Full Integration with MAVROS

If you want to test the complete stack (ArduPilot + Gazebo + ROS2), open a **third terminal**:

### Terminal 3: Launch MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

**Expected Output:**
```
[INFO] ...: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[INFO] ...: IMU: High resolution IMU detected!
```

### Terminal 4: Verify ROS2 Topics

In a fourth terminal:

```bash
# List all MAVROS topics
ros2 topic list | grep mavros

# Check connection state
ros2 topic echo /mavros/state

# Check current position
ros2 topic echo /mavros/local_position/pose

# Check battery status
ros2 topic echo /mavros/battery
```

**Expected Output:**
You should see `connected: true` in `/mavros/state` and live telemetry data from the simulated quadcopter in Gazebo.

---

## 9. Verification Checklist

Before considering the integration complete, confirm:

* [ ] Plugin dependencies installed (`libgz-sim8-dev`, `rapidjson-dev`, etc.).
* [ ] `ardupilot_gazebo` repository cloned.
* [ ] Plugin built successfully with `make -j4`.
* [ ] Environment variables set in `~/.bashrc`.
* [ ] Gazebo launches with `iris_runway.world`.
* [ ] Iris quadcopter model visible in Gazebo.
* [ ] ArduPilot SITL connects to Gazebo (JSON received).
* [ ] Basic commands work (`mode guided`, `arm throttle`, `takeoff`).
* [ ] Quadcopter physically responds in Gazebo 3D view.
* [ ] (Optional) MAVROS connects and publishes topics.

---

## 10. Troubleshooting

**Error: "Failed to load plugin"**

* **Cause:** Environment variables not set or plugin not built.
* **Solution:**
```bash
# Verify plugin exists
ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so

# Re-source environment
source ~/.bashrc

# Rebuild if necessary
cd ~/ardupilot_gazebo/build
make clean
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

**Error: "Waiting for JSON from SITL" (never connects)**

* **Cause:** Gazebo not running or plugin not loaded.
* **Solution:**
1. Ensure Gazebo is running first (Terminal 1).
2. Check Gazebo terminal for plugin loading messages.
3. Verify model has ArduPilot plugin in SDF file:
```bash
grep -i ardupilot ~/ardupilot_gazebo/worlds/iris_runway.world
```

**Error: "Model not found" in Gazebo**

* **Cause:** `GZ_SIM_RESOURCE_PATH` not set correctly.
* **Solution:**
```bash
# Check variable
echo $GZ_SIM_RESOURCE_PATH

# If empty, add again
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

# Make permanent
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

**Error: CMake configuration fails with "gz-sim not found"**

* **Cause:** Wrong Gazebo dev package installed.
* **Solution:**
```bash
# For Gazebo Harmonic, use:
sudo apt install libgz-sim8-dev

# For Gazebo Garden, use:
sudo apt install libgz-sim7-dev
```

**Quadcopter Doesn't Respond to Commands**

* **Cause:** Connection issue or ArduPilot not receiving commands.
* **Solution:**
1. Check ArduPilot console for errors.
2. Verify connection status: `link` command in MAVProxy.
3. Try re-arming: `mode guided` then `arm throttle`.
4. Check Gazebo physics is running (not paused).

**Gazebo Shows Model but Physics Seems Frozen**

* **Cause:** Simulation paused or time step issues.
* **Solution:**
1. In Gazebo, click the "Play" button (triangle icon in bottom left).
2. Check real-time factor in Gazebo (should be close to 1.0).
3. Reduce scene complexity if real-time factor is low (<0.5).

**Build Error: "rapidjson not found"**

* **Cause:** rapidjson-dev package not installed.
* **Solution:**
```bash
sudo apt install rapidjson-dev
cd ~/ardupilot_gazebo/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

---

## 11. Testing Different Vehicle Models

The `ardupilot_gazebo` repository includes multiple vehicle models:

### Quadcopter (Iris)
```bash
# Terminal 1
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.world

# Terminal 2
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### Plane (Zephyr)
```bash
# Terminal 1
gz sim -v4 -r ~/ardupilot_gazebo/worlds/zephyr_runway.world

# Terminal 2
cd ~/ardupilot/ArduPlane
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console
```

### Rover
```bash
# Terminal 1
gz sim -v4 -r ~/ardupilot_gazebo/worlds/runway.world

# Terminal 2
cd ~/ardupilot/Rover
sim_vehicle.py -v Rover -f gazebo-rover --model JSON --map --console
```

---

## 12. Performance Optimization

If simulation is running slowly:

1. **Close Unnecessary Applications**
```bash
# Check system resources
htop
```

2. **Reduce Gazebo Graphics Quality**
   * In Gazebo GUI: Edit → Graphics Quality → Low

3. **Disable Optional Sensors**
   * Edit world files to remove unnecessary sensors (cameras, LIDAR, etc.)

4. **Use Headless Mode** (no GUI)
```bash
# Launch Gazebo server only
gz sim -v4 -r -s worlds/iris_runway.world
```

---

## 13. Summary

You now have a complete UAV simulation environment:

* **ArduPilot SITL:** Flight controller simulation
* **Gazebo Harmonic:** 3D physics and visualization
* **MAVROS (Optional):** ROS2 integration for topics and services

This setup allows you to:
* Test autonomous flight algorithms safely
* Develop and debug UAV control code
* Visualize flight behavior in 3D
* Integrate with ROS2 for advanced robotics applications

---

## 14. Next Steps

With the complete simulation environment running, you can:

* **Develop Custom Flight Modes:** Modify ArduPilot source code and test in simulation
* **Create Custom Worlds:** Design your own Gazebo environments and obstacles
* **Write ROS2 Nodes:** Develop autonomous flight applications using MAVROS topics
* **Multi-Vehicle Simulation:** Launch multiple drones in the same Gazebo world
* **Sensor Integration:** Add cameras, LIDAR, or other sensors to your models

---

[← Back: Module 04 - Gazebo Simulator](../docs/04-gazebo-simulator.md) | **Congratulations! Complete Simulation Environment Ready!**