#!/usr/bin/env python3
"""
Follow Waypoints Node
Autonomous navigation through pre-recorded waypoints.
Implements simple PID-like control for waypoint following.
"""

import os
import yaml
import math
import json
from datetime import datetime
from typing import List, Dict, Optional
from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray

# TF2 for map frame absolute positioning (SLAM integration)
import tf2_ros
from tf2_ros import TransformException


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

    @staticmethod
    def from_dict(data: Dict) -> 'Waypoint':
        """Create Waypoint from dictionary"""
        return Waypoint(**data)

    def distance_to_point(self, x: float, y: float, z: float = 0.0) -> float:
        """Calculate distance to a point"""
        return math.sqrt((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)


class FollowWaypointsNode(Node):
    """Node for autonomous waypoint following"""

    def __init__(self):
        super().__init__('follow_waypoints')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('data_directory', 'data')
        self.declare_parameter('results_directory', 'results')
        self.declare_parameter('waypoint_tolerance', 0.2)  # meters
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('use_yaw_control', False)
        self.declare_parameter('use_slam', True)  # Use map frame from SLAM for absolute coords
        self.declare_parameter('reference_frame', 'map')  # Frame for absolute positioning
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        data_dir_param = self.get_parameter('data_directory').value
        results_dir_param = self.get_parameter('results_directory').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.use_yaw_control = self.get_parameter('use_yaw_control').value
        self.use_slam = self.get_parameter('use_slam').value
        self.reference_frame = self.get_parameter('reference_frame').value
        
        # ── TF2 for SLAM absolute positioning ──────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.slam_active = False  # True once map→odom is available
        self.slam_check_count = 0
        
        # Build paths to source data and results directories (portable, no hardcoded usernames)
        # Assumes project is in ~/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/
        home_dir = os.path.expanduser("~")
        pkg_src_path = os.path.join(home_dir, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol")
        self.data_dir = os.path.join(pkg_src_path, data_dir_param)
        self.results_dir = os.path.join(pkg_src_path, results_dir_param)
        
        # Create results directory
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir, exist_ok=True)
        
        # State variables
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_idx = 0
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.is_executing = False
        self.execution_complete = False
        
        # Metrics tracking
        self.waypoints_reached = 0
        self.start_time = None
        self.end_time = None
        self.errors_per_waypoint: List[float] = []
        self.times_per_waypoint: List[float] = []
        self.last_waypoint_time = None
        
        # ── Polar Coordinate Controller (Siegwart & Nourbakhsh) ──────────
        # Proven stable controller for differential-drive waypoint navigation.
        # No PID, no modes. Just geometry.
        #
        # Control law:
        #   v = k_rho * rho * cos(alpha)    (only advance when pointing at goal)
        #   omega = k_alpha * alpha          (turn towards goal)
        #
        # where:
        #   rho   = distance to goal
        #   alpha = angle from robot heading to goal direction
        #
        # cos(alpha) naturally handles ALL edge cases:
        #   - Pointing at goal  (alpha~0):   cos=1  → full speed
        #   - 90° off           (alpha=pi/2): cos=0  → stop, turn
        #   - Passed the goal   (alpha~pi):   cos<0  → stop, turn back
        #
        self.k_rho = 0.3      # Linear gain: v = k_rho * distance * cos(heading_error)
        self.k_alpha = 1.2    # Angular gain: omega = k_alpha * heading_error
        
        # Load waypoints
        if not self.load_waypoints():
            self.get_logger().error('Failed to load waypoints. Node will not execute.')
            return
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_follower/status', 10)
        self.current_wp_pub = self.create_publisher(Int32, '/waypoint_follower/current_waypoint', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_follower/markers', 10)
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop
        )
        
        self.get_logger().info(
            f'Follow Waypoints Node initialized\n'
            f'  Waypoints loaded: {len(self.waypoints)}\n'
            f'  Waypoint tolerance: {self.waypoint_tolerance}m (TARGET: 0.05m)\n'
            f'  Max linear velocity: {self.max_linear_vel}m/s (TARGET: 0.3m/s)\n'
            f'  Max angular velocity: {self.max_angular_vel}rad/s (TARGET: 0.8rad/s)\n'
            f'  Control frequency: {self.control_freq}Hz (TARGET: 20Hz)\n'
            f'  Use yaw control: {self.use_yaw_control}\n'
            f'  Use SLAM: {self.use_slam} (frame: {self.reference_frame})\n'
            f'  Controller: Polar Coordinates (Siegwart)\n'
            f'  Gains: k_rho={self.k_rho}, k_alpha={self.k_alpha}'
        )

    def load_waypoints(self) -> bool:
        """Load waypoints from YAML file"""
        file_path = os.path.join(self.data_dir, self.waypoints_file)
        
        if not os.path.exists(file_path):
            self.get_logger().error(f'Waypoints file not found: {file_path}')
            return False
        
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data:
                self.get_logger().error('Invalid waypoints file format')
                return False
            
            self.waypoints = [Waypoint.from_dict(wp) for wp in data['waypoints']]
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            return True
        
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return False

    def odom_callback(self, msg: Odometry):
        """Process odometry feedback. If SLAM is active, prefer map→base_link TF."""
        if self.use_slam:
            pose_from_slam = self._get_pose_from_tf()
            if pose_from_slam is not None:
                self.current_pose = pose_from_slam
                if not self.slam_active:
                    self.slam_active = True
                    self.get_logger().info(
                        f'[SLAM] map→base_link transform available! '
                        f'Using absolute coordinates in "{self.reference_frame}" frame'
                    )
                return
            else:
                # SLAM not yet ready - periodically log
                self.slam_check_count += 1
                if self.slam_check_count % 100 == 1:
                    self.get_logger().warn(
                        f'[SLAM] map→base_link not available yet (check #{self.slam_check_count}), '
                        f'falling back to odom'
                    )
        
        # Fallback: use raw odometry (odom frame)
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.current_pose['z'] = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = self._quaternion_to_yaw(qx, qy, qz, qw)

    def _get_pose_from_tf(self) -> dict:
        """Get robot pose from TF2 in the reference frame (map).
        Returns pose dict or None if transform not available."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,  # target frame (map)
                'base_link',           # source frame
                rclpy.time.Time(),     # latest available
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            return {
                'x': t.x,
                'y': t.y,
                'z': t.z,
                'yaw': self._quaternion_to_yaw(r.x, r.y, r.z, r.w)
            }
        except TransformException:
            return None

    def control_loop(self):
        """Main control loop"""
        if not self.waypoints or self.execution_complete:
            return
        
        # Start execution timer if first iteration
        if not self.is_executing:
            self.is_executing = True
            self.start_time = time.time()
            self.last_waypoint_time = self.start_time
            self.get_logger().info('Starting autonomous waypoint following')
        
        # Get current target waypoint
        target_wp = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance to target
        distance = math.sqrt(
            (target_wp.x - self.current_pose['x'])**2 +
            (target_wp.y - self.current_pose['y'])**2
        )
        
        # DEBUG: Print poses every 10 iterations to avoid spam
        if int(self.current_pose['x'] * 1000) % 100 == 0:  # Pseudo-random
            self.get_logger().warn(
                f'[DEBUG] Target WP {target_wp.id}: ({target_wp.x:.4f}, {target_wp.y:.4f}) | '
                f'Current Pose: ({self.current_pose["x"]:.4f}, {self.current_pose["y"]:.4f}) | '
                f'Calculated Distance: {distance:.4f}m'
            )
        
        # Check if waypoint reached
        if distance <= self.waypoint_tolerance:
            self._handle_waypoint_reached(target_wp, distance)
            
            # Move to next waypoint
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                self.current_waypoint_idx += 1
                self.last_waypoint_time = time.time()
            else:
                # All waypoints completed
                self._execution_complete()
                return
        
        # Calculate control commands
        cmd_vel = self._calculate_velocity_command(target_wp, distance)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log velocity command for verification
        #self.get_logger().info(
        #    f'PUBLISHING /cmd_vel → Linear: {cmd_vel.linear.x:.4f} m/s | '
        #    f'Angular: {cmd_vel.angular.z:.4f} rad/s | Distance: {distance:.4f}m'
        #)
        
        # Publish current waypoint index
        msg = Int32()
        msg.data = self.current_waypoint_idx
        self.current_wp_pub.publish(msg)
        
        # Publish visualization markers
        self._publish_markers(target_wp, distance)

    def _handle_waypoint_reached(self, target_wp: Waypoint, error: float):
        """Handle successful waypoint arrival"""
        self.waypoints_reached += 1
        elapsed = time.time() - self.last_waypoint_time
        
        self.errors_per_waypoint.append(error)
        self.times_per_waypoint.append(elapsed)
        
        self.get_logger().warn(
            f'WAYPOINT REACHED: ID={target_wp.id} at ({target_wp.x:.4f}, {target_wp.y:.4f}) | '
            f'Robot at ({self.current_pose["x"]:.4f}, {self.current_pose["y"]:.4f}) | '
            f'Error: {error:.4f}m (tolerance: {self.waypoint_tolerance}m) | '
            f'Time: {elapsed:.2f}s | Total: {self.waypoints_reached}/{len(self.waypoints)}'
        )
        
        self.get_logger().info(
            f'Waypoint {target_wp.id} reached!\n'
            f'  Error: {error:.4f}m\n'
            f'  Time: {elapsed:.2f}s\n'
            f'  Total: {self.waypoints_reached}/{len(self.waypoints)}'
        )
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Waypoint {target_wp.id} reached. Progress: {self.waypoints_reached}/{len(self.waypoints)}'
        self.status_pub.publish(status_msg)

    def _execution_complete(self):
        """Handle execution completion"""
        self.execution_complete = True
        self.end_time = time.time()
        
        # Stop robot
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info('All waypoints completed!')
        self._generate_metrics()

    def _calculate_velocity_command(self, target_wp: Waypoint, distance: float) -> Twist:
        """Polar coordinate controller for differential-drive waypoint navigation.
        
        Based on Siegwart & Nourbakhsh, 'Introduction to Autonomous Mobile Robots'.
        Transforms Cartesian error into polar coordinates (rho, alpha) and applies
        a proportional control law that is provably asymptotically stable.
        
        v     = k_rho * rho * cos(alpha)   — advance only when facing the goal
        omega = k_alpha * alpha             — steer towards the goal
        
        The cos(alpha) coupling ensures:
          - Full speed when aligned with goal
          - Automatic deceleration when misaligned
          - Full stop + pure rotation when perpendicular or past the goal
          - Natural deceleration on approach (rho → 0 ⟹ v → 0)
        """
        cmd = Twist()
        
        # ── Polar error computation ─────────────────────────────────────
        # rho: distance to goal (always >= 0)
        rho = distance
        
        # alpha: angle from robot heading to the direction of the goal
        dx = target_wp.x - self.current_pose['x']
        dy = target_wp.y - self.current_pose['y']
        angle_to_goal = math.atan2(dy, dx)
        alpha = self._normalize_angle(angle_to_goal - self.current_pose['yaw'])
        
        # ── Control law ─────────────────────────────────────────────────
        # Angular: proportional to heading error
        omega = self.k_alpha * alpha
        
        # Linear: proportional to distance, modulated by cos(heading error)
        # cos(alpha) > 0  → robot faces goal → move forward
        # cos(alpha) <= 0 → robot faces away  → stop, let omega turn it
        v = self.k_rho * rho * math.cos(alpha)
        
        # Only move forward (no reversing for a differential drive)
        v = max(0.0, v)
        
        # ── Saturation (respect actuator limits) ────────────────────────
        v = min(v, self.max_linear_vel)
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        
        # ── Apply ───────────────────────────────────────────────────────
        cmd.linear.x = v
        cmd.angular.z = omega
        
        return cmd

    def _quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """Convert quaternion to yaw angle"""
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _publish_markers(self, target_wp: Waypoint, error: float):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Use map frame if SLAM active, otherwise odom
        marker_frame = self.reference_frame if self.slam_active else 'odom'
        
        # Target waypoint marker (larger sphere)
        target_marker = Marker()
        target_marker.header.frame_id = marker_frame
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target_wp.x
        target_marker.pose.position.y = target_wp.y
        target_marker.pose.position.z = 0.0
        target_marker.scale.x = 0.3
        target_marker.scale.y = 0.3
        target_marker.scale.z = 0.3
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 0.7
        marker_array.markers.append(target_marker)
        
        # Current position marker
        current_marker = Marker()
        current_marker.header.frame_id = marker_frame
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.id = 1
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        current_marker.pose.position.x = self.current_pose['x']
        current_marker.pose.position.y = self.current_pose['y']
        current_marker.pose.position.z = 0.0
        current_marker.scale.x = 0.2
        current_marker.scale.y = 0.2
        current_marker.scale.z = 0.2
        current_marker.color.r = 0.0
        current_marker.color.g = 1.0
        current_marker.color.b = 0.0
        current_marker.color.a = 0.7
        marker_array.markers.append(current_marker)
        
        # Line connecting current to target
        line_marker = Marker()
        line_marker.header.frame_id = marker_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.id = 2
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        p1 = Point()
        p1.x = self.current_pose['x']
        p1.y = self.current_pose['y']
        p1.z = 0.0
        
        p2 = Point()
        p2.x = target_wp.x
        p2.y = target_wp.y
        p2.z = 0.0
        
        line_marker.points = [p1, p2]
        line_marker.scale.x = 0.05
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.5
        marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)

    def _generate_metrics(self):
        """Generate and save execution metrics"""
        total_time = self.end_time - self.start_time
        mean_error = sum(self.errors_per_waypoint) / len(self.errors_per_waypoint) if self.errors_per_waypoint else 0.0
        max_error = max(self.errors_per_waypoint) if self.errors_per_waypoint else 0.0
        
        metrics = {
            'execution_summary': {
                'total_execution_time': total_time,
                'waypoints_completed': self.waypoints_reached,
                'total_waypoints': len(self.waypoints),
                'success': self.waypoints_reached == len(self.waypoints),
                'execution_date': datetime.now().isoformat(),
                'slam_active': self.slam_active,
                'reference_frame': self.reference_frame if self.slam_active else 'odom',
            },
            'error_metrics': {
                'mean_error_to_waypoint': mean_error,
                'max_error_to_waypoint': max_error,
                'min_error_to_waypoint': min(self.errors_per_waypoint) if self.errors_per_waypoint else 0.0,
            },
            'timing_metrics': {
                'mean_time_per_waypoint': sum(self.times_per_waypoint) / len(self.times_per_waypoint) if self.times_per_waypoint else 0.0,
                'max_time_per_waypoint': max(self.times_per_waypoint) if self.times_per_waypoint else 0.0,
                'min_time_per_waypoint': min(self.times_per_waypoint) if self.times_per_waypoint else 0.0,
            },
            'per_waypoint_data': [
                {
                    'waypoint_id': i + 1,
                    'error': self.errors_per_waypoint[i] if i < len(self.errors_per_waypoint) else None,
                    'time': self.times_per_waypoint[i] if i < len(self.times_per_waypoint) else None,
                }
                for i in range(len(self.waypoints))
            ]
        }
        
        # Save to JSON
        metrics_file = os.path.join(self.results_dir, 'metrics.json')
        try:
            with open(metrics_file, 'w') as f:
                json.dump(metrics, f, indent=2)
            self.get_logger().info(f'Metrics saved to {metrics_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {e}')
        
        # Print summary to console
        self.get_logger().info(
            f'=== EXECUTION SUMMARY ===\n'
            f'Total time: {total_time:.2f}s\n'
            f'Waypoints: {self.waypoints_reached}/{len(self.waypoints)}\n'
            f'Mean error: {mean_error:.4f}m\n'
            f'Max error: {max_error:.4f}m\n'
            f'Status: {"SUCCESS" if self.waypoints_reached == len(self.waypoints) else "INCOMPLETE"}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypointsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutdown
        if node.cmd_vel_pub:
            cmd = Twist()
            node.cmd_vel_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
