import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    # Create a modified params file for robot1
    custom_params = os.path.join(
        os.path.expanduser("~"), "multi_robot_ws", "robot1_explore_params.yaml"
    )
    
    # Use the original params file as reference
    original_config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [
        ("/tf", "tf"), 
        ("/tf_static", "tf_static"),
        ("/map", "map")
    ]
    
    node = Node(
        package="explore_lite",
        name="explore_node",
        namespace="robot2",  
        executable="explore",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_base_frame": "base_link",
                "global_frame": "map",
                "costmap_topic": "map",
                "costmap_updates_topic": "map_updates",
                "visualize": True,
                "planner_frequency": 0.15,
                "progress_timeout": 30.0,
                "potential_scale": 3.0,
                "orientation_scale": 0.0,
                "gain_scale": 1.0,
                "transform_tolerance": 0.3,
                "min_frontier_size": 0.75,
                "planner_server_name": "compute_path_to_pose",
                "navigator_server_name": "navigate_to_pose"
            }
        ],
        output="screen",
        remappings=remappings,
    )
    
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(node)
    
    return ld
