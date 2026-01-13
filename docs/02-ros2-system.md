# Module 02: ROS2 Installation

## 1. Objectives

In this module, you will install **ROS2 Humble Hawksbill**. ROS2 acts as the "middleware" that manages communication between the drone's flight controller, the simulator, and your custom code.

* **Locale Setup:** Configure system locales to support UTF-8.
* **Repositories:** Add the official ROS2 software sources.
* **System Safety:** Update core system components (Critical step for Ubuntu 22.04).
* **Installation:** Install ROS2 Humble Desktop and development tools.
* **Environment:** Configure the shell to recognize ROS2 commands.
* **Verification:** Run a Talker/Listener demo.

**Estimated Time:** 30 minutes

---

## 2. Prerequisites

* **Completed:** [Module 01.](../docs/01-ardupilot-sitl.md)
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish).
* **Architecture:** x86_64.

---

## 3. Set Locale

ROS 2 requires a UTF-8 locale. If the system locale is not configured correctly, installation may fail or runtime errors will occur.

### 3.1 Check and Generate Locale
Run the following block of commands to ensure `en_US.UTF-8` is active.

```bash
sudo apt update && sudo apt install locales

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

### 3.2 Verify Locale

```bash
locale
```

**Expected Output:** Ensure `LANG` and `LC_ALL` are set to `en_US.UTF-8`.

---

## 4. Setup Sources

We will configure the ROS 2 repositories using the ros2-apt-source package. This package automatically manages the keys and source configuration.

### 4.1 Enable Ubuntu Universe

Ensure the Ubuntu Universe repository is enabled.

```bash
sudo apt install software-properties-common

sudo add-apt-repository universe
```

### 4.2 Download and Install Repository Configuration

We will download the latest release of the `ros2-apt-source` package and install it.

```bash
sudo apt update && sudo apt install curl -y
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

---

## 5. Install ROS 2 Packages

### ⚠️ 5.1 Critical System Update

**Do not skip this step.**
On a fresh Ubuntu 22.04 installation, installing ROS 2 without upgrading system packages first can cause the removal of critical system files (due to `systemd` and `udev` conflicts).

```bash
sudo apt update
sudo apt upgrade
```

*If the upgrade installs many packages, it is recommended to restart your computer before proceeding.*

### 5.2 Install Desktop Version

We install `ros-humble-desktop`, which includes the core libraries plus visualization tools like **RViz**, which are useful for drone simulation.

```bash
sudo apt install ros-humble-desktop
```

**System Activity:**

* This is a large download (~2.5GB).
* It may take 10–20 minutes.

### 5.3 Install Development Tools

Install the compiler tools used to build ROS packages.

```bash
sudo apt install ros-dev-tools
```

---

## 6. Environment Setup

To run ROS 2 commands, you must "source" the setup script. We will add this to your `.bashrc` so it happens automatically every time you open a terminal.

### 6.1 Add to Bashrc

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

### 6.2 Verify Environment Variables

Check if ROS 2 is loaded correctly.

```bash
printenv | grep ROS
```

**Expected Output:**

```text
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
ROS_LOCALHOST_ONLY=0
```

*If you see no output, the sourcing step failed.*

---

## 7. Try Some Examples

We will verify the installation by running the standard C++ and Python demo nodes.

### 7.1 Start the Talker

Open a terminal and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Expected Output:**

```text
[INFO] ...: Publishing: 'Hello World: 1'
[INFO] ...: Publishing: 'Hello World: 2'
```

*Leave this terminal running.*

### 7.2 Start the Listener

Open a **new** terminal (Ctrl+Alt+T) and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

**Expected Output:**

```text
[INFO] ...: I heard: [Hello World: 1]
[INFO] ...: I heard: [Hello World: 2]
```

**Action:** If both terminals show successful communication, press **Ctrl+C** in both windows to stop them.

---

## 8. Verification Checklist

Before proceeding to Module 03, verify the following:

* [ ] Locale is set to UTF-8.
* [ ] ROS 2 GPG key and repository added.
* [ ] **System upgraded** (sudo apt upgrade) to prevent systemd conflicts.
* [ ] `ros-humble-desktop` installed successfully.
* [ ] `ros-dev-tools` installed.
* [ ] `.bashrc` updated and sourced.
* [ ] Talker/Listener demo works across two separate terminals.

---

## 9. Troubleshooting

**Error: "E: Unable to locate package ros-humble-desktop"**

* **Cause:** The ROS 2 repository was not added correctly, or `apt update` was not run.
* **Solution:** Re-run Section 4 carefully.

**Error: "ros2: command not found"**

* **Cause:** The setup script was not sourced.
* **Solution:**
```bash
source /opt/ros/humble/setup.bash
```


*If this works, ensure you added it to `~/.bashrc` correctly.*

**Error: "The following packages will be REMOVED"**

* **Cause:** You did not run `sudo apt upgrade` in Step 5.1.
* **Solution:** **ABORT (Press 'n')**. Run `sudo apt upgrade` first, then try installing ROS 2 again.

---

[← Back: Module 01](../docs/01-ardupilot-sitl.md) | [Next: Module 03 - MAVROS Setup](../docs/03-mavros-bridge.md)

---
<p align="center">
  Made with ❤️ for the drone community<br>
  <a href="https://github.com/simtofly">SimToFly</a> • 
  <a href="https://github.com/sidharthmohannair">@sidharthmohannair</a>
</p>