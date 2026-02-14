#!/usr/bin/env python3
"""
Save the current SLAM map for later localization.

Serializes the slam_toolbox pose graph so that future sessions
can use localization mode with a consistent map→odom transform,
ensuring waypoints recorded in one session are valid in the next.

Usage:
    ros2 run autonomous_patrol save_map.py
    ros2 run autonomous_patrol save_map.py --ros-args -p map_name:=my_map

After saving, start SLAM in localization mode:
    ros2 launch autonomous_patrol slam.launch.py mode:=localization
"""

import os
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


class SaveMapNode(Node):
    def __init__(self):
        super().__init__('save_map')
        
        self.declare_parameter('map_name', 'warehouse_map')
        map_name = self.get_parameter('map_name').value
        
        home_dir = os.path.expanduser("~")
        data_dir = os.path.join(
            home_dir, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data"
        )
        self.map_path = os.path.join(data_dir, f"{map_name}_serial")
        
        self.get_logger().info(f'Saving map to: {self.map_path}')
        
        self.client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('slam_toolbox serialize service not available. Is SLAM running?')
            return
        
        request = SerializePoseGraph.Request()
        request.filename = self.map_path
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'✅ Map saved successfully to: {self.map_path}')
            self.get_logger().info(f'To use localization mode:')
            self.get_logger().info(f'  ros2 launch autonomous_patrol slam.launch.py mode:=localization')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to save map: {e}')
        
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
