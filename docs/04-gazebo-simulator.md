# Module 04: Gazebo Installation

## 1. Objectives

In this module, you will install **Gazebo Harmonic**, a modern 3D robotics simulator that provides physics simulation and visualization for UAVs.

* **Installation:** Install Gazebo Harmonic simulator and required dependencies.
* **Verification:** Launch Gazebo and verify rendering and physics performance.
* **Environment Setup:** Configure environment variables for Gazebo plugins and models.
* **Independence:** Ensure Gazebo runs independently without ArduPilot or ROS2 dependencies.

**Estimated Time:** 15-20 minutes

---

## 2. Prerequisites

* **Completed:** Module 01 [ArduPilot SITL installed](../docs/01-ardupilot-sitl.md).
* **Completed:** Module 02 [ROS 2 installed](../docs/02-ros2-system.md).
* **Completed:** Module 03 [MAVROS installed](../docs/03-mavros-bridge.md).
* **System:** Ubuntu 22.04 (Jammy) with functional graphics support.
* **Network:** Active internet connection (required for packages and models).

**Note:** If running in a virtual machine or WSL2, OpenGL support may be limited. Native Ubuntu installation is recommended for best graphics performance.

---

## 3. Understanding Gazebo Versions

Gazebo has evolved through multiple generations:

* **Gazebo Classic** (Gazebo 11): Legacy version, end-of-life.
* **Gazebo Ignition/New Gazebo**: Modern generation with versions named alphabetically:
  - Fortress → Garden → Harmonic → Ionic

**For our setup:**
* **ROS2 Humble** officially supports **Gazebo Fortress**
* **ArduPilot** recommends **Gazebo Garden or Harmonic**
* **Our choice:** **Gazebo Harmonic** (best compatibility with ArduPilot and newer features)

---

## 4. Install Gazebo Harmonic

### 4.1 Update Package Lists

```bash
sudo apt update
```

### 4.2 Install Prerequisites

```bash
sudo apt install curl lsb-release gnupg -y
```

### 4.3 Add Gazebo Repository

Add the official Gazebo package repository:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

### 4.4 Update Package Lists Again

```bash
sudo apt update
```

### 4.5 Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
```

**Expected Output:**
```
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following NEW packages will be installed:
  gz-harmonic gz-cmake3 gz-common5 gz-fuel-tools9 gz-gui8 gz-launch7
  gz-math7 gz-msgs10 gz-physics7 gz-plugin2 gz-rendering8 gz-sensors8
  gz-sim8 gz-tools2 gz-transport13 gz-utils2 ...
```

This installation will include all Gazebo Harmonic packages and their dependencies.

---

## 5. Verification: Launch Gazebo

### 5.1 Launch Default World

Test that Gazebo is working correctly by launching the default "shapes" world:

```bash
gz sim -v4 -r shapes.sdf
```

**Command Breakdown:**
* `gz sim`: Launch Gazebo simulator
* `-v4`: Verbose output level 4 (useful for troubleshooting)
* `-r`: Run simulation immediately (instead of paused)
* `shapes.sdf`: Default world file with basic geometric shapes

**Expected Output in Terminal:**
```
[GUI] [Msg] Loading config [/home/username/.gz/sim/8/gui.config]
[GUI] [Msg] Render engine [ogre2] loaded
[Msg] Loaded level [1]
[Msg] Serving world controls on [/world/shapes/control]
[Msg] Serving GUI information on [/world/shapes/gui/info]
[Msg] World [shapes] started [/world/shapes/control/state]
```

**Expected Visual Output:**
A Gazebo window should open displaying:
* A ground plane
* Various 3D geometric shapes (sphere, box, cylinder, etc.)
* Physics simulation running (shapes may be falling or interacting)

**Performance Check:**
* Window opens within 5-10 seconds
* Shapes render clearly
* Physics simulation runs smoothly (30+ FPS)

### 5.2 Close Gazebo

Press `Ctrl+C` in the terminal to stop Gazebo, or close the Gazebo window.

---

## 6. Verification Checklist

Before proceeding to Module 05, confirm:

* [ ] Prerequisites installed (`lsb-release`, `wget`, `gnupg`).
* [ ] Gazebo repository added successfully.
* [ ] Gazebo Harmonic installed (`gz-harmonic` package).
* [ ] Gazebo launches with `gz sim -v4 -r shapes.sdf`.
* [ ] Gazebo window displays 3D shapes correctly.
* [ ] Physics simulation runs smoothly without errors.
* [ ] Gazebo can be closed cleanly with `Ctrl+C`.

---

## 7. Troubleshooting

**Error: "Failed to load shared library"**

* **Cause:** Missing dependencies or incomplete installation.
* **Solution:** Reinstall Gazebo:
```bash
sudo apt remove gz-harmonic
sudo apt autoremove
sudo apt update
sudo apt install gz-harmonic
```

**Error: "Render engine not found" or Black Window**

* **Cause:** Graphics driver issues or insufficient OpenGL support.
* **Solution:**
1. Check OpenGL version: `glxinfo | grep "OpenGL version"`
2. Update graphics drivers:
```bash
sudo ubuntu-drivers autoinstall
```
3. For NVIDIA users:
```bash
sudo apt install nvidia-driver-535
```
4. Reboot system: `sudo reboot`

**Error: "Unable to create the rendering window"**

* **Cause:** Running in virtual machine or WSL2 without proper graphics support.
* **Solution:**
1. **For VMware/VirtualBox:** Enable 3D acceleration in VM settings.
2. **For WSL2:** Use WSLg (Windows 11 22H2 or later) or install VcXsrv X server.
3. **Best Solution:** Use native Ubuntu installation for full graphics support.

**Slow Performance / Low FPS**

* **Cause:** Insufficient GPU resources or complex scene.
* **Solution:**
1. Close other graphics-intensive applications.
2. Reduce Gazebo rendering quality in GUI settings.
3. Use simpler worlds for testing.
4. Check system resources: `top` or `htop`.

**Error: "Package 'gz-harmonic' has no installation candidate"**

* **Cause:** Gazebo repository not added correctly.
* **Solution:** Remove and re-add repository:
```bash
sudo rm /etc/apt/sources.list.d/gazebo-stable.list
sudo rm /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```
Then repeat steps 4.2-4.5.

**Gazebo Launches but No Models Visible**

* **Cause:** Model download incomplete (first launch downloads models).
* **Solution:** Wait 2-3 minutes for initial model downloads to complete. Check terminal output for download progress.

---

## 8. Additional Information

### Gazebo Command Reference

```bash
# Launch Gazebo with verbosity
gz sim -v4

# Launch specific world
gz sim -v4 -r /path/to/world.sdf

# List available Gazebo commands
gz --help

# Check Gazebo version
gz sim --version
```

### Important Directories

* **Gazebo config:** `~/.gz/sim/8/`
* **Downloaded models:** `~/.gz/fuel/fuel.gazebosim.org/`
* **System models:** `/usr/share/gz/gz-sim8/`

### WSL2 and Headless Users

If you are using WSL2 or a headless system:
* Gazebo requires X server for graphics (VcXsrv, Xming, or native WSLg)
* For truly headless testing, you can skip Gazebo verification and proceed to Module 05
* Module 05 integration testing will confirm if Gazebo works correctly with ArduPilot

---

## 9. Next Steps

Now that Gazebo is installed and verified, you can proceed to:
* **Module 05:** ArduPilot–Gazebo Integration (bridge ArduPilot SITL with Gazebo simulation)

The next module will install the `ardupilot_gazebo` plugin that connects ArduPilot's flight controller simulation to Gazebo's physics and visualization.

---

[← Back: Module 03 - MAVROS Bridge](../docs/03-mavros-bridge.md) | [Next: Module 05 - ArduPilot-Gazebo Integration](../docs/05-gazebo-ardupilot.md)

---
<p align="center">
  Made with ❤️ for the drone community<br>
  <a href="https://github.com/simtofly">SimToFly</a> • 
  <a href="https://github.com/sidharthmohannair">@sidharthmohannair</a>
</p>