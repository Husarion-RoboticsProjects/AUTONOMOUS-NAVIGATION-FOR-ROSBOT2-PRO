# ROSbot 2 PRO: Autonomous Navigation & Control (ROS2 Humble/Foxy)

## 📌 Overview
This project develops an advanced autonomous navigation system for the **Husarion ROSbot 2 PRO**. We solve the hardware obsolescence challenge by implementing a hybrid architecture:
- **Development Environment:** ROS2 Humble running inside **Docker**.
- **Hardware Host:** ROS2 Foxy (Native support for UP Board / Intel Atom).
- **Control & SLAM:** Integration of `slam_toolbox`, `Nav2`, and custom C++ motion planning.

## 🏗️ Project Structure
- `/src`: ROS2 packages (`rosbot_slam`, `global_motion`, and Python prototypes).
- `/config`: Optimized parameters for RPLidar A2, Astra Camera, and Navigation.
- `/docker`: `compose.yaml` and environment setup for seamless deployment.

## 🚀 Getting Started
1. **Clone the repo:**
   ```bash
   git clone [https://github.com/YourOrgName/YourRepo.git](https://github.com/YourOrgName/YourRepo.git)
