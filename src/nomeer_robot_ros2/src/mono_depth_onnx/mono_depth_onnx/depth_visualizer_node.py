#!/usr/bin/env python3
"""
Depth Visualizer Node
Displays depth map in a window with metrics overlay.
Used for monitoring and debugging.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge


class DepthVisualizerNode(Node):
    """Visualizes depth maps and metrics in a window."""

    def __init__(self):
        super().__init__('depth_visualizer')
        
        # Declare parameters
        self.declare_parameter('display_window', True)
        self.declare_parameter('window_width', 960)
        self.declare_parameter('window_height', 540)
        
        # Get parameters
        self.display_window = self.get_parameter('display_window').value
        self.window_w = self.get_parameter('window_width').value
        self.window_h = self.get_parameter('window_height').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Current values
        self.last_depth_img = None
        self.last_min_depth = 0.0
        self.last_mean_depth = 0.0
        self.last_obstacle_status = False
        
        # Subscribers
        self.depth_viz_sub = self.create_subscription(
            Image,
            '/camera/depth_colored',
            self.depth_viz_callback,
            10
        )
        
        self.min_depth_sub = self.create_subscription(
            Float32,
            '/depth_metric/min_frontal_depth',
            self.min_depth_callback,
            10
        )
        
        self.mean_depth_sub = self.create_subscription(
            Float32,
            '/depth_metric/mean_depth',
            self.mean_depth_callback,
            10
        )
        
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/depth_metric/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        self.get_logger().info('Depth Visualizer Node initialized')

    def depth_viz_callback(self, msg: Image):
        """Receive colored depth visualization."""
        try:
            self.last_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def min_depth_callback(self, msg: Float32):
        """Receive minimum depth metric."""
        self.last_min_depth = msg.data

    def mean_depth_callback(self, msg: Float32):
        """Receive mean depth metric."""
        self.last_mean_depth = msg.data

    def obstacle_callback(self, msg: Bool):
        """Receive obstacle detection status."""
        self.last_obstacle_status = msg.data

    def destroy_node(self):
        """Clean up."""
        if self.display_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
