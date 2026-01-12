# Module 03: MAVROS Setup

## 1. Objectives

In this module, you will install and configure **MAVROS** (MAVLink to ROS bridge). MAVROS acts as the translator that converts ArduPilot's MAVLink telemetry into ROS 2 topics.

* **Installation:** Install MAVROS and MAVROS Extras packages.
* **GeographicLib:** Install the mandatory dataset (Critical step).
* **Integration:** Establish a communication bridge between ArduPilot SITL and ROS 2.
* **Verification:** Verify that ROS 2 topics are receiving live flight data.

**Estimated Time:** 20 minutes

---

## 2. Prerequisites

* **Completed:** Module 01 [ArduPilot SITL installed](../docs/01-ardupilot-sitl.md).
* **Completed:** Module 02 [ROS 2 installed](../docs/02-ros2-system.md).
* **Network:** Active internet connection (required for map datasets).

---

## 3. Install MAVROS

We will install the binary packages for ROS 2 Humble.

### 3.1 Update Package Lists

First, update your apt cache to ensure you have the latest package information:

```bash
sudo apt update
```

This step is critical to avoid 404 errors when installing packages.

### 3.2 Install MAVROS Packages

Open a terminal and run:

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

**Expected Output:**
```
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following NEW packages will be installed:
  ros-humble-mavros ros-humble-mavros-extras ...
```

The installation will include additional dependencies like:
- `geographiclib-tools`
- `ros-humble-libmavconn`
- `ros-humble-mavlink`
- `ros-humble-mavros-msgs`

### 3.3 Install GeographicLib Datasets (Critical)

**WARNING:** MAVROS requires specific magnetic and geoid datasets to convert GPS coordinates. As per the official documentation, *not having the dataset available will shutdown the mavros_node.*

We will download and run the installation script directly:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod +x install_geographiclib_datasets.sh

sudo ./install_geographiclib_datasets.sh
```

**System Activity:**

* The script will download three datasets (`Geoids`, `Gravity`, `Magnetic`).
* Depending on internet speed, this may take 2–5 minutes.
* **Wait** until you see the message: `Processing... done.` for all datasets.
* You may remove the script file after installation: `rm install_geographiclib_datasets.sh`

**Expected Output:**
```
Installing GeographicLib geoids egm96-5
Installing GeographicLib gravity egm96
Installing GeographicLib magnetic emm2015
```

---

## 4. The Bridge Concept

Before launching, understand how the components connect:

1. **ArduPilot SITL** sends MAVLink heartbeat packets to UDP port `14550`.
2. **MAVROS** listens on UDP port `14550`.
3. **ROS 2** publishes this data as topics (e.g., `/mavros/state`).

---

## 5. Verification: The "Headless" System Test

We will now run the full communication pipeline. You will need **three separate terminal windows**.

### Terminal 1: Launch SITL

Start the virtual drone.

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

*Wait until the simulator is ready and you see the `STABILIZE>` prompt.*

### Terminal 2: Launch MAVROS

Start the bridge using the official `apm.launch` file.

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

**Understanding the Connection URL:**

* `udp://`: Use UDP protocol.
* `:14550`: **Bind Port**. MAVROS listens here for data from SITL.
* `@127.0.0.1:14555`: **Remote Port**. MAVROS sends data back to SITL here.

**Expected Output:**
You should see a stream of logs. Look for these success messages:

```text
[INFO] ...: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[INFO] ...: IMU: High resolution IMU detected!
```

### Terminal 3: Verify ROS 2 Topics

Check if the data is actually reaching ROS 2.

1. **Echo Drone State:**
Run this command to see the live connection status:
```bash
ros2 topic echo /mavros/state
```


**Expected Output:**
```yaml
header:
  stamp:
    sec: 1768243238
    nanosec: 899383721
  frame_id: ''
connected: true
armed: false
guided: false
manual_input: true
mode: STABILIZE
system_status: 3
```


**Key Check:** Ensure `connected: true` is visible.

2. **List All MAVROS Topics** (Optional):
```bash
ros2 topic list | grep mavros
```

You should see many topics including:
```
/mavros/state
/mavros/imu/data
/mavros/local_position/pose
/mavros/battery
/mavros/rc/in
...
```

---

## 6. Verification Checklist

Before proceeding to Module 04, confirm:

* [ ] Package lists updated with `sudo apt update`.
* [ ] MAVROS and Extras packages installed.
* [ ] GeographicLib datasets installed (Script ran successfully).
* [ ] Terminal 1: SITL running (Map/Console visible).
* [ ] Terminal 2: MAVROS launched with `CON: Got HEARTBEAT`.
* [ ] Terminal 3: `ros2 topic echo /mavros/state` shows `connected: true`.

---

## 7. Troubleshooting

**Error: "404 Not Found" when installing MAVROS packages**

* **Cause:** Outdated apt cache or temporary repository sync issue.
* **Solution:** Clean and update apt cache:
```bash
sudo apt clean
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

**Error: "GeographicLib exception: Geoid model not loaded"**

* **Cause:** Step 3.3 was skipped or the download failed.
* **Solution:** Re-run the `wget` and installation script commands.

**Error: "UDP bind: Address already in use"**

* **Cause:** Another instance of MAVROS or QGroundControl is running.
* **Solution:** Close QGroundControl and any other terminals running MAVROS.

**Error: "connected: false" in Terminal 3**

* **Cause:** MAVROS is running, but it cannot see ArduPilot.
* **Solution:**
1. Ensure SITL is running in Terminal 1.
2. Verify the `fcu_url` port is `14550`.
3. If using WSL2, check Windows Firewall settings.

**Error: "Package 'ros-humble-mavros' has no installation candidate"**

* **Cause:** ROS 2 repository not configured properly.
* **Solution:** 
1. Verify ROS 2 Humble is installed: `printenv | grep ROS`
2. Check if ROS 2 repository is in sources:
```bash
cat /etc/apt/sources.list.d/ros2.list
```
3. Re-add ROS 2 repository if needed (refer to Module 02).

---

[← Back Module 02](../docs/02-ros2-system.md) | [Next: Module 04 - Gazebo Installation](../docs/04-gazebo-simulator.md)