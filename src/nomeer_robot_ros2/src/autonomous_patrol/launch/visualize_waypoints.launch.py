import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    autonomous_patrol_dir = get_package_share_directory('autonomous_patrol')
    config_dir = os.path.join(autonomous_patrol_dir, 'config')
    
    # Load configuration file
    config_file = os.path.join(config_dir, 'autonomous_patrol_config.yaml')
    
    # Create visualizer node
    visualizer_node = Node(
        package='autonomous_patrol',
        executable='visualizer_node.py',
        name='waypoint_visualizer',
        parameters=[config_file],
        output='screen',
    )
    
    # Create rviz node for visualization
    rviz_config = os.path.join(autonomous_patrol_dir, 'rviz', 'waypoints.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )

    return LaunchDescription([
        visualizer_node,
        rviz_node,
    ])
