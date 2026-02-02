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
- [Husarion Docker Guide]([https://husarion.com/manuals/rosbot/docker/](https://husarion.com/software/docker/robots/))
- [Husarion ROS2 Tutorials](https://husarion.com/tutorials/ros-tutorials/)

### 2. Installation
Clone this repository into your ROSbot's user directory:
```bash
git clone [https://github.com/YourOrgName/YourRepo.git](https://github.com/YourOrgName/YourRepo.git)
cd YourRepo
