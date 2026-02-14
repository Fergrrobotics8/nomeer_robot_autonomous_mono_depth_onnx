import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    mono_depth_dir = get_package_share_directory('mono_depth_onnx')
    config_dir = os.path.join(mono_depth_dir, 'config')
    config_file = os.path.join(config_dir, 'mono_depth_config.yaml')
    
    # Image Source Node
    image_source_node = Node(
        package='mono_depth_onnx',
        executable='image_source_node.py',
        name='image_source',
        parameters=[config_file],
        output='screen',
    )
    
    # Depth Inference Node
    depth_inference_node = Node(
        package='mono_depth_onnx',
        executable='depth_inference_node.py',
        name='depth_inference',
        parameters=[config_file],
        output='screen',
    )
    
    # Depth Metric Node
    depth_metric_node = Node(
        package='mono_depth_onnx',
        executable='depth_metric_node.py',
        name='depth_metric',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        image_source_node,
        depth_inference_node,
        depth_metric_node,
    ])
