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
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3* \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup 
```
Environment Setup:
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```
### 3. Build the Workspace
Once the repository is cloned and the dependencies are installed, you need to build the workspace:
```bash
cd ~/multi_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
After the build process finishes, source the workspace setup file to ensure ROS 2 knows about the workspace and its packages:
```bash
source install/setup.bash
```
### 4. Launch the Simulation
Once everything is set up, you can launch the multi-robot simulation with the following command:
```bash
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True
```
This will start the simulation with SLAM functionality enabled for multiple TurtleBot3 robots.
