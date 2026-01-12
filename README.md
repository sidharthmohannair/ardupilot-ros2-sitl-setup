# ArduPilot, ROS2, and MAVROS Integration Guide

![ROS2](https://img.shields.io/badge/ROS2-Humble-343434?style=flat-square&logo=ros&logoColor=white)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-E03C31?style=flat-square&logo=ardupilot&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)
![Status](https://img.shields.io/badge/Status-Active-success?style=flat-square)

This repository provides a structured, reproducible, and verifiable guide for building a UAV research and simulation environment. It covers the installation and integration of ArduPilot SITL (Software-In-The-Loop), ROS2 (Robot Operating System), MAVROS, and Gazebo.

This guide is designed for students, researchers, and developers who require a validated development workflow before moving to advanced topics such as offboard control, perception pipelines, or swarm coordination.

The installation follows a strict sequential roadmap. **Each component must be verified before proceeding to the next.**

## System Compatibility

This repository is validated on the following configuration:

* **Operating System:**
    * **Recommended:** Ubuntu 22.04 LTS (Native Install or Dual Boot).
    * **Supported (Headless):** Windows 10/11 via **WSL2** (Ubuntu 22.04 instance).
* **ROS2 Distribution:** Humble Hawksbill.
* **Architecture:** x86_64.
* **Minimum Hardware:** 4 Cores CPU, 8GB RAM. (16GB RAM + Dedicated GPU required for Gazebo).

**Important Notes on Virtualization:**
* **WSL2 Users:** Phases 1 through 3 (Headless) work perfectly on WSL2. However, Phases 4 and 5 (Gazebo) depend on **WSLg** graphics support, which can vary in stability depending on your GPU drivers.
* **Virtual Machines:** Support is **not provided** for VMWare or VirtualBox due to poor GPU pass-through performance required for the simulation physics engine.

## Repository Structure

```text

├── docs/
│   ├── 01-ardupilot-sitl.md       # Flight dynamics engine setup
│   ├── 02-ros2-system.md          # Middleware installation
│   ├── 03-mavros-bridge.md        # Communication bridge setup
│   ├── 04-gazebo-simulator.md     # Physics engine installation
│   └── 05-gazebo-ardupilot.md     # Full integration and 3D visualization
├── scripts/                       # Helper scripts (if applicable)
├── LICENSE
└── README.md

```

## Installation Roadmap

The setup is divided into independent phases. Follow them in order.

**Flexible Workflow:**
* **Full Simulation:** Complete all phases (1–5) for a complete 3D environment with physics.
* **Headless Mode:** If you only need to test control logic, MAVLink protocols, or swarm behavior without 3D graphics, you may **stop after Phase 3**.

### Phase 1: Flight Controller Simulation (ArduPilot SITL)
Install the ArduPilot firmware and required developer tools.
* **Objective:** Run a virtual UAV using the ArduPilot native console.
* **Outcome:** Confirm that the flight stack and vehicle dynamics are working correctly.
* **[> Go to Module 01: ArduPilot SITL Setup](docs/01-ardupilot-sitl.md)**

### Phase 2: ROS2 Environment
Install and configure the ROS2 middleware.
* **Objective:** Verify ROS2 installation, environment sourcing, and core CLI commands.
* **Outcome:** Ensure ROS2 nodes and topics function independently.
* **[> Go to Module 02: ROS2 Installation](docs/02-ros2-system.md)**

### Phase 3: MAVROS Bridge
Install MAVROS, which bridges MAVLink (used by ArduPilot) with ROS2 DDS topics.
* **Objective:** Establish communication between ArduPilot SITL and ROS2 in a headless setup.
* **Outcome:** Confirm MAVROS topics are visible in ROS2.
* **[> Go to Module 03: MAVROS Setup](docs/03-mavros-bridge.md)**
* *(Note: WSL2 users or Headless users may stop here).*

### Phase 4: Gazebo Simulator
Install the Gazebo simulation environment for physics and visualization.
* **Objective:** Launch Gazebo independently and verify rendering and physics performance.
* **Outcome:** Ensure the simulator runs without ArduPilot or ROS2 dependencies.
* **[> Go to Module 04: Gazebo Installation](docs/04-gazebo-simulator.md)**

### Phase 5: ArduPilot–Gazebo Integration
Integrate ArduPilot with Gazebo using the `ardupilot_gazebo` plugin.
* **Objective:** Run a complete UAV simulation with ArduPilot (control), Gazebo (physics), and ROS2 (telemetry).
* **Outcome:** Perform a full system validation with a 3D visualizer.
* **[> Go to Module 05: Gazebo Bridge Setup](docs/05-gazebo-ardupilot.md)**

## Usage and Validation

This repository serves as a prerequisite for advanced UAV development. Before attempting autonomous flight, offboard control, or swarm simulations, ensure your system meets the following criteria:

1. **Simulation:** Gazebo launches without error and runs at real-time speed.
2. **Connection:** MAVROS connects to the simulated Flight Control Unit (FCU).
3. **Data:** Running `ros2 topic list` displays valid `/mavros/` topics.

## Contributing

Contributions to improve installation scripts, update documentation for newer ROS2 releases, or fix bugs are welcome. Please open an Issue to discuss the change before submitting a Pull Request.

## License

This project is distributed under the MIT License. See the `LICENSE` file for details.

---
<p align="center">
  Made with ❤️ for the drone community<br>
  <a href="https://github.com/simtofly">SimToFly</a> • 
  <a href="https://github.com/sidharthmohannair">@sidharthmohannair</a>
</p>