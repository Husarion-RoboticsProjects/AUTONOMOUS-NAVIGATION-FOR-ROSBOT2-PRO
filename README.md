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
```bash
cd ~/your_repo_path/custom_rosbot_nav
colcon build --symlink-install
source install/setup.bash
```
### Step 1: Mapping (Slam Toolbox)
To create a map of your environment, follow these steps:

Launch the Hardware: Start the base robot nodes and sensors (Lidar/Camera) using Docker:
```bash
docker compose up -d rplidar microros rosbot nav2

docker exec -it ID_navigation2 bash
```
You can identify the Container ID or Name (navigation2)
```bash
docker ps -a
```
```bash
# Inside the container
ros2 launch nav2_bringup navigation_launch.py
ros2 launch rosbot_slam slam.launch.py
```

RViz configuration:

<img width="1386" height="937" alt="image" src="https://github.com/user-attachments/assets/319f91cd-b537-4b31-8b2e-0c74325af3bb" />


To achieve the visualization shown above during the mapping process, configure your RViz2 workspace with the following displays. This ensures correct data validation for SLAM:

* **Fixed Frame:** Set to `map`.
* **Map Display:** * Topic: `/map`
    * Durability Policy: `Transient Local`
* **LaserScan Display:** * Topic: `/scan`
* **RobotModel:** To visualize the ROSbot 2 PRO URDF in real-time.

Once exploration is finished, save the map in the shared volume to ensure it persists on the host:

```bash
ros2 run nav2_map_server map_saver_cli -f /complete-path-to-desired-directory/nombre_mapahome
```
After saving, you can stop the launch files

### Step 2: Path planning and navigation

```bash
#Inside the navigation2 docker
ros2 launch rosbot_slam amcl.launch.py
```

```bash
# In the ROSbot host terminal (Foxy)
ros2 run global_motion global_planner_node
```
RViz configuration:

<img width="886" height="598" alt="image" src="https://github.com/user-attachments/assets/bceb15d9-7dcd-4259-bfed-760d5d1df41e" />


Based on previous configuration for mapping step, you must add:

* **Path:** This represents the obstacle-free route calculated by our custom algorithm. Set to `planned_path`
* **Pose with Covariance:** Estimated robot pose and its associated covariance ellipse, representing the localization system's confidence and spatial uncertainty.. Set to `/amcl_pose`

#### Starting navigation

Set Initial Pose: Click the "2D Pose Estimate" button and indicate the robot's current position and orientation on the map.

Define Destination: Select the "2D Goal Pose" tool and click on the target location. This triggers the path calculation in the global_motion_node and starts the autonomous movement.
