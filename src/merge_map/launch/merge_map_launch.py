import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for map merging system."""
    
    # Path to RViz configuration file
    rviz_file = os.path.join(
        get_package_share_directory('merge_map'),
        'config',
        'merge_map.rviz'
    )

    return LaunchDescription([
        # RViz2 node for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        
        # Map merging node
        Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ("/map1", "/robot1/map"),    # Remap robot1's map topic
                ("/map2", "/robot2/map"),    # Remap robot2's map topic
                ("/merge_map", "/map")      # Publish merged map to standard topic
            ],
        ),
    ])
