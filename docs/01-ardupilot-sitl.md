# Module 01: ArduPilot SITL Installation

## 1. Objectives

In this module, you will install and launch your first virtual drone.

* **System Setup:** Update Ubuntu and install essential development tools.
* **Source Code:** Clone the ArduPilot repository.
* **Dependencies:** Install the required compilers and Python packages.
* **Simulation:** Build and launch the SITL simulator with the Console and Map.

**Estimated Time:** 15–30 minutes

---

## 2. Prerequisites

Before starting, ensure you have:
* **Operating System:** Ubuntu 22.04 LTS.
* **Workspace:** All commands will be executed in the **Home Directory** (`~`).
* **Internet:** Active connection required.

---

## 3. System Update & Essential Tools

Before cloning ArduPilot, we must ensure the system is up to date and has the basic tools required for development.

### 3.1 Update System
Open a terminal (**Ctrl+Alt+T**) and run:

```bash
sudo apt update

sudo apt upgrade -y
```

### 3.2 Install Developer Tools

Install these core packages to avoid "command not found" errors later.

```bash
sudo apt install -y \
    build-essential \
    git \
    python3 \
    python3-pip \
    python3-dev \
    curl \
    wget \
    nano \
    net-tools \
    tree
```

---

## 4. Clone ArduPilot Repository

We will clone the repository directly to your home directory.

### 4.1 Navigate to Home

```bash
cd ~
```

### 4.2 Clone Source Code

```bash
git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot
```

*Note: This downloads the full source history and may take several minutes.*

### 4.3 Verify Clone

Verify the directories were created.

```bash
ls
```

**Expected Output:** You should see directories such as `ArduCopter`, `ArduPlane`, `Tools`, and `libraries`.

---

## 5. Install ArduPilot Dependencies

ArduPilot provides a script to install the necessary compilers and tools.

### 5.1 Run Installation Script

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

**System Activity:**

* This downloads approximately 2GB of packages.
* Wait for the message: `Finished installing prerequisites`.

### 5.2 Reload Your Profile

The installation script modifies your environment. Reload it to apply changes to the current terminal:

```bash
. ~/.profile
```

### 5.3 Configure System Path (Permanent Fix)

To ensure the simulation commands work in every new terminal (and to avoid "command not found" errors in the future), we will manually add the ArduPilot tools to your `.bashrc` file.

```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc

echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc

source ~/.bashrc
```

---

## 6. Initialize Submodules

ArduPilot uses external libraries called submodules.

### 6.1 Configure Git Protocol

If you are on a restricted network, force git to use HTTPS to prevent connection errors.

```bash
git config --global url.https://.insteadOf git://
```

### 6.2 Checkout Stable Version

We will use **Copter-4.5.7** (Stable Release).

```bash
git checkout Copter-4.5.7
```

**Expected Output:**

```text
Note: switching to 'Copter-4.5.7'.
HEAD is now at ... ArduCopter: release 4.5.7
```

### 6.3 Update Submodules

Download the required libraries.

```bash
git submodule update --init --recursive
```

**Expected Output:** The system will register and check out paths for `modules/mavlink`, `modules/gtest`, etc.

---

## 7. First SITL Launch

The first time you run the simulator, it needs to compile the code and generate default parameters.

### 7.1 Wipe and Configure

Use the `-w` flag to wipe parameters and set defaults.

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

**System Activity:**

* The compiler will build the firmware (2–5 minutes).
* Wait until you see the message: `Received [approx 500] parameters`.
* **Action:** Once parameters are received, press **Ctrl+C** to exit.

### 7.2 Verify Path Access

Ensure `sim_vehicle.py` is accessible from the terminal.

```bash
which sim_vehicle.py
```

**Expected Output:** `/home/username/ardupilot/Tools/autotest/sim_vehicle.py`

---

## 8. Test SITL Launch

Now we will launch the simulator properly with the Map and Console.

### 8.1 Launch Simulator

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --map --console
```

* `-v ArduCopter`: Specifies the vehicle type.
* `--map`: Opens the 2D mission planning map.
* `--console`: Opens the MAVProxy command and status window.

**Expected Output:**

* The terminal will show `Starting SITL`.
* Two new windows (Map and Console) will appear.
* You will see the prompt: `STABILIZE>`.

### 8.2 Test Basic Commands

Enter the following commands at the `STABILIZE>` prompt:

1. **Change Mode:**
```bash
mode GUIDED
```


*Result:* `GUIDED>`
2. **Arm Throttle:**
```bash
arm throttle
```


*Result:* `ARMED`
3. **Disarm:**
```bash
disarm
```


*Result:* `DISARMED`

### 8.3 Exit

Press **Ctrl+C** to stop the simulator.

---

## 9. Verification Checklist

Before proceeding, confirm:

* [ ] System updated and essential tools installed.
* [ ] ArduPilot cloned to `~/ardupilot`.
* [ ] Dependencies installed and `.profile` reloaded.
* [ ] Path manually added to `.bashrc`.
* [ ] Submodules updated and checked out to `Copter-4.5.7`.
* [ ] First launch with `-w` flag completed.
* [ ] SITL launches with **Map** and **Console** windows.
* [ ] Vehicle accepts `mode GUIDED` and `arm throttle`.

---

## 10. Common Questions

**Q: Why Copter-4.5.7?**
A: This is the latest stable release tested for this guide. It ensures consistency across all users.

**Q: What is the difference between `-w` and normal launch?**
A: The `-w` flag wipes the virtual EEPROM and resets parameters to default. Use this only for the initial setup.

**Q: How much disk space is required?**
A: Approximately 5GB (2GB for source, 3GB for build artifacts).

---

## 11. Troubleshooting

**Error: "sim_vehicle.py: command not found"**

* **Cause:** The `source ~/.bashrc` command did not work in the current terminal.
* **Solution:** Close the terminal and open a new one, or re-run:

```bash
source ~/.bashrc
```



**Error: "Failed to download submodules"**

* **Cause:** Network firewall blocking git protocol.
* **Solution:** Ensure you ran the `git config` command in Step 6.1.

**Error: "ImportError: No module named pymavlink"**

* **Cause:** Python dependencies missing.
* **Solution:**
```bash
pip3 install --user pymavlink MAVProxy
```



**Error: Map or Console does not open**

* **Cause:** Missing graphical libraries.
* **Solution:**
```bash
sudo apt install python3-wxgtk4.0 python3-matplotlib python3-opencv
```



---

[← Back to Index](../README.md) | [Next: Module 02 - ROS2 Installation](../docs/02-ros2-system.md)

---
<p align="center">
  Made with ❤️ for the drone community<br>
  <a href="https://github.com/simtofly">SimToFly</a> • 
  <a href="https://github.com/sidharthmohannair">@sidharthmohannair</a>
</p>