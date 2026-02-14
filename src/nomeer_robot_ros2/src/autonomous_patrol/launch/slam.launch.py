"""
SLAM Launch File - Smart Auto Mode with Auto-Save
Automatically detects and loads saved maps, or creates a new one if none exists.
When mapping completes, automatically saves the map on exit.

Behavior:
  - If a saved map exists: Launches in LOCALIZATION mode (preserves waypoints)
  - If no map exists: Launches in MAPPING mode (creates new map)
  - On exit in MAPPING mode: Automatically saves map via save_map.py
  
Usage:
  ros2 launch autonomous_patrol slam.launch.py                    # Auto mode (recommended, auto-saves)
  ros2 launch autonomous_patrol slam.launch.py mode:=mapping      # Force mapping (auto-saves on exit)
  ros2 launch autonomous_patrol slam.launch.py mode:=localization # Force localization (no save needed)

After mapping, press Ctrl+C to exit - map will be saved automatically!
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Rutas
    autonomous_patrol_dir = get_package_share_directory('autonomous_patrol')
    config_dir = os.path.join(autonomous_patrol_dir, 'config')
    slam_config = os.path.join(config_dir, 'slam_toolbox_params.yaml')
    
    # Default map path (in the data directory)
    home_dir = os.path.expanduser("~")
    default_map = os.path.join(
        home_dir, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data/warehouse_map_serial"
    )
    
    # Auto-detect mode: if map exists, use localization; otherwise mapping
    def get_auto_mode():
        if os.path.exists(f"{default_map}.posegraph") or os.path.exists(f"{default_map}.data"):
            return 'localization'
        else:
            return 'mapping'
    
    auto_mode = get_auto_mode()

    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode', default_value=auto_mode,
        description=f'SLAM mode: "mapping" (new map) or "localization" (auto-detected: {auto_mode})'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value=default_map,
        description='Path to serialized map file (without extension) for localization mode'
    )

    # Build parameters dict dynamically based on mode
    slam_params = [
        slam_config,
        {
            'use_sim_time': True,
            'mode': LaunchConfiguration('mode'),
        }
    ]
    
    # Only add map_file_name if we're in localization mode
    if auto_mode == 'localization':
        slam_params[1]['map_file_name'] = LaunchConfiguration('map_file')

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=slam_params,
        remappings=[
            ('/scan', '/lidar')
        ],
    )

    # Continuous Map Saver - only in mapping mode
    continuous_saver_node = None
    pose_capturer_node = None
    pose_publisher_node = None
    
    if auto_mode == 'mapping':
        continuous_saver_node = Node(
            package='autonomous_patrol',
            executable='continuous_map_saver.py',
            name='continuous_map_saver',
            output='screen',
        )
        
        # Capture initial pose when starting mapping
        pose_capturer_node = Node(
            package='autonomous_patrol',
            executable='initial_pose_capturer.py',
            name='initial_pose_capturer',
            output='screen',
        )
    
    elif auto_mode == 'localization':
        # Restore initial pose when loading map
        pose_publisher_node = Node(
            package='autonomous_patrol',
            executable='initial_pose_publisher.py',
            name='initial_pose_publisher',
            output='screen',
        )

    # Auto-save map when SLAM exits in mapping mode
    launch_actions = [
        mode_arg,
        map_file_arg,
        slam_toolbox_node,
    ]
    
    # Add continuous saver and pose capturer if in mapping mode
    if continuous_saver_node is not None:
        launch_actions.append(continuous_saver_node)
    
    if pose_capturer_node is not None:
        launch_actions.append(pose_capturer_node)
    
    # Add pose publisher if in localization mode
    if pose_publisher_node is not None:
        launch_actions.append(pose_publisher_node)
    
    # Register shutdown event handler (backup save on exit)
    if auto_mode == 'mapping':
        save_map_process = ExecuteProcess(
            cmd=['ros2', 'run', 'autonomous_patrol', 'save_map.py'],
            output='screen',
        )
        
        launch_actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[save_map_process]
                )
            )
        )

    return LaunchDescription(launch_actions)
