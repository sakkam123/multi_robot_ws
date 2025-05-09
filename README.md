# Multi-Robot SLAM Simulation

This project demonstrates a multi-robot SLAM (Simultaneous Localization and Mapping) simulation using ROS 2 (Humble), with the `slam_toolbox` package. It is designed to work with multiple TurtleBot3 robots in a simulated environment.

## Project Setup

### 1. Clone the Repository

First, clone this repository.

```bash
git clone https://github.com/sakkam123/multi_robot_ws
```
### 2. Install Dependencies
Make sure that ROS 2 Humble is installed on your system. If you haven't already installed it, follow the ROS 2 installation guide.
You will also need to install the necessary dependencies for the project:
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```
### 3. Build the Workspace
Once the repository is cloned and the dependencies are installed, you need to build the workspace:
```bash
cd ~/multi_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
