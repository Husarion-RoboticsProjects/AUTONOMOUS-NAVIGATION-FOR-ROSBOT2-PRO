# ROSbot 2 PRO: Autonomous Navigation & Control (ROS2 Humble/Foxy)

## Overview
This project develops an advanced autonomous navigation system for the **Husarion ROSbot 2 PRO**. We solve the hardware obsolescence challenge by implementing a hybrid architecture:
- **Development Environment:** ROS2 Humble running inside **Docker**.
- **Hardware Host:** ROS2 Foxy (Native support for UP Board / Intel Atom).
- **Control & SLAM:** Integration of `slam_toolbox`, `Nav2`, and custom C++ motion planning.

## Project Structure
- `/src`: ROS2 packages (`rosbot_slam`, `global_motion`, and Python reactive navigation prototype).
- `/docker`: `compose.yaml` and environment setup for seamless deployment.

### 1. Prerequisites
Before deploying, ensure your ROSbot 2 PRO has **Docker** and **Docker Compose** installed. For detailed instructions on setting up the environment and internal hardware communication, refer to the official documentation:
- [Husarion Docker Guide](https://husarion.com/software/docker/robots/)
- [Husarion ROS2 Tutorials](https://husarion.com/tutorials/ros-tutorials/)

### 2. Installation
Clone this repository into your ROSbot's user directory:
```bash
git clone [https://github.com/Husarion-RoboticsProjects/AUTONOMOUS-NAVIGATION-FOR-ROSBOT2-PRO.git]
cd AUTONOMOUS-NAVIGATION-FOR-ROSBOT2-PRO
```
### 3. Compilation Guide

To ensure system stability, follow this dual-compilation workflow:

#### 3.1 Perception Layer (Docker - Humble)
The `rosbot_slam_ws` must be mounted as a shared volume in your Docker container.
```bash
# Inside the Humble container
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```
#### 3.2 Perception Layer (ROSbot Host - Foxy)
cd ~/your_repo_path/custom_rosbot_nav
colcon build --symlink-install
source install/setup.bash

