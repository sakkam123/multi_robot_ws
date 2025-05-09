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
### 5. Navigation Launch

To launch the Navigation2 stack for each robot with its respective map:

```bash
# Launch Nav2 for Robot 1
ros2 launch nav2_bringup bringup_launch.py \
  namespace:=robot1 \
  use_sim_time:=true \
  map:=/home/mariem/multi_robot_ws/src/your_package/maps/robot1_map.yaml

# Launch Nav2 for Robot 2
ros2 launch nav2_bringup bringup_launch.py \
  namespace:=robot2 \
  use_sim_time:=true \
  map:=/home/mariem/multi_robot_ws/src/your_package/maps/robot2_map.yaml
```
### 6. Autonomous Exploration

To launch autonomous exploration using the `explore_lite` package for each robot:

```bash
# Launch exploration for Robot 1
ros2 launch $(ros2 pkg prefix explore_lite)/share/explore_lite/launch/robot1_explore.launch.py use_sim_time:=true

# Launch exploration for Robot 2
ros2 launch $(ros2 pkg prefix explore_lite)/share/explore_lite/launch/robot2_explore.launch.py use_sim_time:=true
```
### 7. Merge Maps
To launch the map merging process, run:
```bash
ros2 launch merge_map merge_map_launch.py
```
### 8. Save Individual Maps
To save the map for robot1, run the following command:
```bash
cd ~/multi_robot_ws/src/multirobot_map_merge/maps
ros2 run nav2_map_server map_saver_cli -f map1 --ros-args -r map:=/robot1/map
```
To save the map for robot2, run:
```bash
ros2 run nav2_map_server map_saver_cli -f map2 --ros-args -r map:=/robot2/map
```
### 9. Save Merged Map
After merging the maps, save the final merged map by running:
```bash
cd ~/multi_robot_ws/src/merge_map/resource
ros2 run nav2_map_server map_saver_cli -f merged_map --ros-args -r map:=/map
```
