#!/usr/bin/env python3
"""
Waypoint Visualizer Node
Publishes waypoints and trajectory for visualization in RViz.
"""

import os
import yaml
import time
from typing import List

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class VisualizerNode(Node):
    """Node for visualizing waypoints and trajectory"""

    def __init__(self):
        super().__init__('waypoint_visualizer')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('data_directory', 'data')
        self.declare_parameter('publish_frequency', 1.0)  # Hz
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        data_dir_param = self.get_parameter('data_directory').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        
        # Build path to source data directory (portable via ROS 2)
        pkg_src_path = get_package_share_directory('autonomous_patrol')
        self.data_dir = os.path.join(pkg_src_path, data_dir_param)
        
        # File path and modification tracking (for hot-reload)
        self.file_path = os.path.join(self.data_dir, self.waypoints_file)
        self.last_mtime = 0.0
        
        # Waypoints data
        self.waypoints_data = None
        self.last_num_waypoints = 0  # Track number of waypoints for cleanup on reload
        self.max_markers_ever_published = 0  # Track max markers for complete cleanup
        
        # Load waypoints
        self.load_waypoints()
        
        # Publishers
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_visualizer/waypoints',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Marker,
            '/waypoint_visualizer/trajectory',
            10
        )
        
        # Publish timer
        self.publish_timer = self.create_timer(
            1.0 / self.publish_freq,
            self.publish_markers
        )
        
        self.get_logger().info(f'Waypoint Visualizer initialized')

    def load_waypoints(self) -> bool:
        """Load waypoints from YAML file"""
        if not os.path.exists(self.file_path):
            self.get_logger().warn(f'Waypoints file not found: {self.file_path}')
            return False
        
        try:
            with open(self.file_path, 'r') as f:
                self.waypoints_data = yaml.safe_load(f)
            
            if self.waypoints_data and 'waypoints' in self.waypoints_data:
                num_waypoints = len(self.waypoints_data['waypoints'])
                self.get_logger().info(f'Loaded {num_waypoints} waypoints for visualization')
                return True
            else:
                self.get_logger().warn('Invalid waypoints file format')
                return False
        
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return False

    def publish_markers(self):
        """Publish waypoint markers and trajectory"""
        # Hot-reload: Check if waypoints file has been modified
        file_changed = False
        try:
            if os.path.exists(self.file_path):
                current_mtime = os.path.getmtime(self.file_path)
                if current_mtime != self.last_mtime:
                    file_changed = True
                    self.get_logger().warn(
                        f'[HOT-RELOAD] Waypoints file changed (old_mtime={self.last_mtime}, new_mtime={current_mtime}) → reloading {self.file_path}'
                    )
                    # If we had waypoints before, clear the old markers first
                    if self.max_markers_ever_published > 0:
                        self.get_logger().info(
                            f'[HOT-RELOAD] Clearing {self.max_markers_ever_published} old markers from RViz'
                        )
                        self._publish_delete_markers(self.max_markers_ever_published)
                        time.sleep(0.05)
                    
                    # Reload waypoints from file
                    self.load_waypoints()
                    # Update last_mtime AFTER successful reload
                    self.last_mtime = current_mtime
        except (OSError, FileNotFoundError) as e:
            self.get_logger().debug(f'Could not check waypoints file: {e}')
            pass  # File temporarily inaccessible, continue with current data
        
        if not self.waypoints_data or 'waypoints' not in self.waypoints_data:
            return
        
        waypoints = self.waypoints_data['waypoints']
        
        if not waypoints:
            return
        
        # Create marker array for individual waypoints
        marker_array = MarkerArray()
        
        # Add waypoint markers
        for i, wp in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(wp['x'])
            marker.pose.position.y = float(wp['y'])
            marker.pose.position.z = float(wp['z'])
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color gradient based on waypoint order
            progress = float(i / len(waypoints)) if len(waypoints) > 1 else 0.0
            marker.color.r = float(progress)
            marker.color.g = float(1.0 - progress)
            marker.color.b = 0.5
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Add text labels for waypoints
        for i, wp in enumerate(waypoints):
            if i % max(1, len(waypoints) // 10) == 0:  # Label every 10% of waypoints
                text_marker = Marker()
                text_marker.header.frame_id = 'odom'
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.id = 1000 + i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = float(wp['x'])
                text_marker.pose.position.y = float(wp['y'])
                text_marker.pose.position.z = float(wp['z']) + 0.2
                text_marker.scale.z = 0.1
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"WP{wp['id']}"
                marker_array.markers.append(text_marker)
        
        # Publish individual waypoint markers
        self.marker_array_pub.publish(marker_array)
        
        # Create trajectory line
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = 'odom'
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.05
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 0.0
        trajectory_marker.color.b = 1.0
        trajectory_marker.color.a = 0.5
        
        # Add all waypoint positions to the line
        for wp in waypoints:
            point = Point()
            point.x = float(wp['x'])
            point.y = float(wp['y'])
            point.z = float(wp['z'])
            trajectory_marker.points.append(point)
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory_marker)
        
        # Update tracking: Keep max to ensure complete cleanup on hot-reload
        current_num_markers = len(waypoints)
        if current_num_markers > self.max_markers_ever_published:
            self.max_markers_ever_published = current_num_markers
        self.last_num_waypoints = current_num_markers

    def _publish_delete_markers(self, num_markers: int):
        """Publish DELETE action for all markers (both waypoints and text labels)"""
        delete_array = MarkerArray()
        
        # Delete individual waypoint markers (IDs 0 to num_markers-1)
        for i in range(num_markers):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.action = Marker.DELETE
            delete_array.markers.append(marker)
        
        # Delete text label markers (IDs 1000 to 1000+num_markers-1)
        for i in range(num_markers):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 1000 + i
            marker.action = Marker.DELETE
            delete_array.markers.append(marker)
        
        # Delete trajectory marker (ID 0 - but use different namespace or just rewrite it)
        self.marker_array_pub.publish(delete_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
