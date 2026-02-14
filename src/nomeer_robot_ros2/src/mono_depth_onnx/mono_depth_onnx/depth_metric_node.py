#!/usr/bin/env python3
"""
Depth Metric Node - B4 Implementation
Calculates simple and robust depth metrics from the depth map.

Metrics:
- min_frontal_depth: Minimum depth in central ROI (obstacle detection)
- avg_frontal_depth: Average depth in central ROI
- median_frontal_depth: Median depth in central ROI (robust)
- obstacle_detected: Boolean flag (1 if min_depth < threshold)

Uses percentile-based outlier filtering for robustness.
"""

import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class DepthMetricNode(Node):
    """Calculates metrics from depth maps."""

    def __init__(self):
        super().__init__('depth_metric')
        
        # Declare parameters
        self.declare_parameter('roi_x_start', 0.2)              # 20% from left
        self.declare_parameter('roi_x_end', 0.8)                # 80% from left
        self.declare_parameter('roi_y_start', 0.2)              # 20% from top
        self.declare_parameter('roi_y_end', 0.8)                # 80% from top
        self.declare_parameter('outlier_percentile_low', 5)     # Remove bottom 5%
        self.declare_parameter('outlier_percentile_high', 95)   # Remove top 5%
        self.declare_parameter('obstacle_threshold', 0.5)       # Depth < 0.5 = obstacle
        
        # Get parameters
        self.roi_x_start = self.get_parameter('roi_x_start').value
        self.roi_x_end = self.get_parameter('roi_x_end').value
        self.roi_y_start = self.get_parameter('roi_y_start').value
        self.roi_y_end = self.get_parameter('roi_y_end').value
        self.outlier_percentile_low = self.get_parameter('outlier_percentile_low').value
        self.outlier_percentile_high = self.get_parameter('outlier_percentile_high').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber to depth map
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_estimated',
            self.depth_callback,
            10
        )
        
        # Publishers for metrics
        self.min_depth_pub = self.create_publisher(
            Float32, '/depth_metric/min_frontal_depth', 10)
        self.avg_depth_pub = self.create_publisher(
            Float32, '/depth_metric/avg_frontal_depth', 10)
        self.median_depth_pub = self.create_publisher(
            Float32, '/depth_metric/median_frontal_depth', 10)
        self.obstacle_pub = self.create_publisher(
            Float32, '/depth_metric/obstacle_detected', 10)
        
        # Performance metrics
        self.frame_count = 0
        
        self.get_logger().info(
            f'Depth Metric Node initialized\n'
            f'  ROI (normalized): x=[{self.roi_x_start:.0%}, {self.roi_x_end:.0%}] '
            f'y=[{self.roi_y_start:.0%}, {self.roi_y_end:.0%}]\n'
            f'  Outlier filtering: percentile [{self.outlier_percentile_low}, {self.outlier_percentile_high}]\n'
            f'  Obstacle threshold: {self.obstacle_threshold}'
        )

    def depth_callback(self, msg: Image):
        """Process depth map and calculate metrics."""
        try:
            # Convert ROS Image to OpenCV (16-bit mono)
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            
            # Convert to float (0-1 range)
            # depth_inference_node publishes: 0-65535 = 0-1 normalized
            depth_float = depth_cv.astype(np.float32) / 65535.0
            
            # Extract ROI (Region of Interest - central area)
            roi_depth = self._extract_roi(depth_float)
            
            if roi_depth is None or roi_depth.size == 0:
                return
            
            # Filter outliers using percentile method
            roi_filtered = self._filter_outliers(roi_depth)
            
            if roi_filtered.size == 0:
                self.get_logger().warn('No valid depth values after filtering')
                return
            
            # Calculate metrics
            min_depth = float(np.min(roi_filtered))
            avg_depth = float(np.mean(roi_filtered))
            median_depth = float(np.median(roi_filtered))
            obstacle_detected = float(min_depth < self.obstacle_threshold)
            
            # Publish metrics
            self._publish_metric(self.min_depth_pub, min_depth)
            self._publish_metric(self.avg_depth_pub, avg_depth)
            self._publish_metric(self.median_depth_pub, median_depth)
            self._publish_metric(self.obstacle_pub, obstacle_detected)
            
            # Log metrics periodically
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                obstacle_str = "OBSTACLE" if obstacle_detected else "CLEAR"
                self.get_logger().info(
                    f'Frame {self.frame_count}: '
                    f'min={min_depth:.3f} avg={avg_depth:.3f} median={median_depth:.3f} '
                    f'[{obstacle_str}]'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def _extract_roi(self, depth: np.ndarray) -> Optional[np.ndarray]:
        """
        Extract Region of Interest (central region) from depth map.
        
        Args:
            depth: Depth map (H x W)
            
        Returns:
            Flattened array of ROI depth values
        """
        h, w = depth.shape
        
        # Calculate ROI boundaries (in pixels)
        x_start = int(w * self.roi_x_start)
        x_end = int(w * self.roi_x_end)
        y_start = int(h * self.roi_y_start)
        y_end = int(h * self.roi_y_end)
        
        # Ensure valid boundaries
        x_start = max(0, x_start)
        x_end = min(w, x_end)
        y_start = max(0, y_start)
        y_end = min(h, y_end)
        
        if x_start >= x_end or y_start >= y_end:
            self.get_logger().error(f'Invalid ROI: x=[{x_start}, {x_end}] y=[{y_start}, {y_end}]')
            return None
        
        # Extract ROI and flatten for analysis
        roi = depth[y_start:y_end, x_start:x_end]
        return roi.flatten()

    def _filter_outliers(self, data: np.ndarray) -> np.ndarray:
        """
        Filter outliers using percentile method.
        
        Keeps only values between specified percentiles.
        Robust against extreme values.
        
        Args:
            data: 1D array of depth values
            
        Returns:
            Filtered array
        """
        low_percentile = np.percentile(data, self.outlier_percentile_low)
        high_percentile = np.percentile(data, self.outlier_percentile_high)
        
        # Keep only values between percentiles
        filtered = data[(data >= low_percentile) & (data <= high_percentile)]
        
        return filtered

    def _publish_metric(self, publisher, value: float):
        """Publish a Float32 metric."""
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMetricNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
