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
    
    # Create the waypoint follower node
    follow_waypoints_node = Node(
        package='autonomous_patrol',
        executable='follow_waypoints_node.py',
        name='follow_waypoints',
        parameters=[config_file],
        output='screen',
    )
    
    # Create visualizer node
    visualizer_node = Node(
        package='autonomous_patrol',
        executable='visualizer_node.py',
        name='waypoint_visualizer',
        parameters=[config_file],
        output='screen',
    )
    
    # Create rviz node for visualization
    # Note: RViz is already launched in robot.launch.py
    # Commenting out to avoid duplicate RViz windows
    
    return LaunchDescription([
        follow_waypoints_node,
        visualizer_node,
    ])
