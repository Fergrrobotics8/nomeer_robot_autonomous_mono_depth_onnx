#!/usr/bin/env python3
"""
Waypoint Recording Node
Records waypoints from odometry while robot is teleoperated.
Supports saving to YAML format with configurable sampling.
"""

import os
import yaml
import math
from datetime import datetime
from typing import List, Dict, Tuple
from dataclasses import dataclass, asdict

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import String


@dataclass
class Waypoint:
    """Waypoint data structure"""
    id: int
    timestamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    linear_vel: float = 0.0
    angular_vel: float = 0.0

    def to_dict(self) -> Dict:
        """Convert to dictionary for YAML serialization"""
        return asdict(self)

    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate Euclidean distance to another waypoint"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )


class RecordWaypointsNode(Node):
    """Node for recording waypoints from robot odometry"""

    def __init__(self):
        super().__init__('record_waypoints')
        
        # Declare parameters
        self.declare_parameter('sampling_mode', 'distance')  # 'distance' or 'frequency'
        self.declare_parameter('min_distance_between_waypoints', 0.1)  # meters
        self.declare_parameter('sampling_frequency', 5.0)  # Hz
        self.declare_parameter('output_file', 'waypoints.yaml')
        self.declare_parameter('data_directory', 'data')
        
        # Get parameters
        self.sampling_mode = self.get_parameter('sampling_mode').value
        self.min_distance = self.get_parameter('min_distance_between_waypoints').value
        self.sampling_frequency = self.get_parameter('sampling_frequency').value
        self.output_file = self.get_parameter('output_file').value
        data_dir_param = self.get_parameter('data_directory').value
        
        # Build path to source data directory (portable via ROS 2)
        pkg_src_path = get_package_share_directory('autonomous_patrol')
        self.data_dir = os.path.join(pkg_src_path, data_dir_param)
        
        # Create data directory if it doesn't exist
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir, exist_ok=True)
        
        # State variables
        self.waypoints: List[Waypoint] = []
        self.last_recorded_waypoint: Waypoint = None
        self.is_recording = False
        self.start_time = None
        self.waypoint_counter = 0
        
        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Timer for frequency-based sampling
        if self.sampling_mode == 'frequency':
            self.timer_period = 1.0 / self.sampling_frequency
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.last_odom_msg = None
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/waypoint_recorder/status', 10)
        
        self.get_logger().info(
            f'Waypoint Recorder initialized\n'
            f'  Sampling mode: {self.sampling_mode}\n'
            f'  Min distance: {self.min_distance}m\n'
            f'  Sampling frequency: {self.sampling_frequency}Hz\n'
            f'  Output file: {os.path.join(self.data_dir, self.output_file)}'
        )
        
        # Timer for periodic status updates
        self.create_timer(2.0, self.publish_status)

    def odom_callback(self, msg: Odometry):
        """Process odometry messages"""
        self.last_odom_msg = msg
        
        if self.sampling_mode == 'distance':
            self._record_waypoint_distance_based(msg)

    def timer_callback(self):
        """Frequency-based waypoint recording"""
        if self.last_odom_msg is not None:
            self._record_waypoint_frequency_based(self.last_odom_msg)

    def _record_waypoint_distance_based(self, msg: Odometry):
        """Record waypoint if minimum distance threshold is exceeded"""
        current_wp = self._odom_to_waypoint(msg)
        
        if self.last_recorded_waypoint is None:
            # First waypoint
            self.waypoints.append(current_wp)
            self.last_recorded_waypoint = current_wp
            self.get_logger().info(f'Recorded first waypoint: id={current_wp.id}')
        elif current_wp.distance_to(self.last_recorded_waypoint) >= self.min_distance:
            # Distance threshold exceeded
            self.waypoints.append(current_wp)
            self.last_recorded_waypoint = current_wp
            self.get_logger().info(
                f'Recorded waypoint: id={current_wp.id}, '
                f'pos=({current_wp.x:.2f}, {current_wp.y:.2f}, {current_wp.z:.2f})'
            )

    def _record_waypoint_frequency_based(self, msg: Odometry):
        """Record waypoint at fixed frequency"""
        current_wp = self._odom_to_waypoint(msg)
        self.waypoints.append(current_wp)
        
        if len(self.waypoints) % 10 == 0:  # Log every 10 waypoints
            self.get_logger().info(
                f'Recorded waypoint: id={current_wp.id}, '
                f'pos=({current_wp.x:.2f}, {current_wp.y:.2f}, {current_wp.z:.2f})'
            )

    def _odom_to_waypoint(self, msg: Odometry) -> Waypoint:
        """Convert Odometry message to Waypoint"""
        self.waypoint_counter += 1
        
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin_vel = msg.twist.twist.linear.x
        ang_vel = msg.twist.twist.angular.z
        
        return Waypoint(
            id=self.waypoint_counter,
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            x=pos.x,
            y=pos.y,
            z=pos.z,
            qx=ori.x,
            qy=ori.y,
            qz=ori.z,
            qw=ori.w,
            linear_vel=lin_vel,
            angular_vel=ang_vel
        )

    def publish_status(self):
        """Publish recorder status"""
        msg = String()
        msg.data = f'Recording: {len(self.waypoints)} waypoints recorded'
        self.status_pub.publish(msg)

    def save_waypoints(self):
        """Save recorded waypoints to YAML file"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to save')
            return False
        
        file_path = os.path.join(self.data_dir, self.output_file)
        
        # Create YAML structure
        data = {
            'metadata': {
                'recording_date': datetime.now().isoformat(),
                'total_waypoints': len(self.waypoints),
                'sampling_mode': self.sampling_mode,
                'min_distance': self.min_distance,
                'sampling_frequency': self.sampling_frequency,
            },
            'waypoints': [wp.to_dict() for wp in self.waypoints]
        }
        
        # Write to file
        try:
            with open(file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {file_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
            return False

    def destroy_node(self):
        """Clean up on node shutdown"""
        if self.waypoints:
            self.save_waypoints()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordWaypointsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
