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
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        data_dir_param = self.get_parameter('data_directory').value
        results_dir_param = self.get_parameter('results_directory').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.use_yaw_control = self.get_parameter('use_yaw_control').value
        
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
        
        # PID controller state for linear velocity
        self.linear_last_error = 0.0
        self.linear_integral_error = 0.0
        self.last_control_time = time.time()
        
        # PID controller state for angular velocity
        self.angular_last_error = 0.0
        self.angular_integral_error = 0.0
        
        # PID tuning parameters - optimized for precision waypoint following
        # LINEAL: controla convergencia a la distancia exacta
        self.linear_kp = 1.8    # Proportional: respuesta rápida al error
        self.linear_ki = 0.3    # Integral: elimina errores sostenidos (ej: stuck en 0.19m)
        self.linear_kd = 0.6    # Derivative: amortigua y previene overshooting
        self.linear_i_max = 0.4 # Anti-windup limit
        
        # ANGULAR: controla orientación hacia el waypoint
        self.angular_kp = 2.0   # Más responsivo
        self.angular_ki = 0.15  # Persiste para alineación perfecta
        self.angular_kd = 0.5   # Suaviza rotación
        self.angular_i_max = 0.6 # Anti-windup limit
        
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
            f'  PID Gains - Linear: Kp={self.linear_kp}, Ki={self.linear_ki}, Kd={self.linear_kd}\n'
            f'  PID Gains - Angular: Kp={self.angular_kp}, Ki={self.angular_ki}, Kd={self.angular_kd}'
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
        """Process odometry feedback"""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.current_pose['z'] = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = self._quaternion_to_yaw(qx, qy, qz, qw)

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
        """Calculate velocity command using PID control (proportional + integral + derivative)"""
        cmd = Twist()
        
        # Calculate time step for PID
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        if dt <= 0:
            dt = 0.01  # Default 10ms step
        
        # ===== LINEAR VELOCITY CONTROL (Distance-based PID) =====
        # Error: distance to target
        linear_error = distance
        
        # Proportional term
        linear_p = self.linear_kp * linear_error
        
        # Integral term (accumulate error over time)
        self.linear_integral_error += linear_error * dt
        # Anti-windup: limit integral to prevent saturation
        self.linear_integral_error = max(-self.linear_i_max, 
                                         min(self.linear_i_max, 
                                             self.linear_integral_error))
        linear_i = self.linear_ki * self.linear_integral_error
        
        # Derivative term (rate of change of error)
        linear_d = 0.0
        if dt > 0:
            linear_d = self.linear_kd * (linear_error - self.linear_last_error) / dt
        
        self.linear_last_error = linear_error
        
        # Calculate desired heading to target
        dx = target_wp.x - self.current_pose['x']
        dy = target_wp.y - self.current_pose['y']
        desired_yaw = math.atan2(dy, dx)
        
        # Current yaw error
        yaw_error = self._normalize_angle(desired_yaw - self.current_pose['yaw'])
        yaw_error_mag = abs(yaw_error)
        
        # ===== ANGULAR VELOCITY CONTROL (Yaw error-based PID) =====
        angular_error = yaw_error
        
        # Proportional term
        angular_p = self.angular_kp * angular_error
        
        # Integral term
        self.angular_integral_error += angular_error * dt
        self.angular_integral_error = max(-self.angular_i_max,
                                          min(self.angular_i_max,
                                              self.angular_integral_error))
        angular_i = self.angular_ki * self.angular_integral_error
        
        # Derivative term
        angular_d = 0.0
        if dt > 0:
            angular_d = self.angular_kd * (angular_error - self.angular_last_error) / dt
        
        self.angular_last_error = angular_error
        
        # Combine PID terms
        cmd.angular.z = angular_p + angular_i + angular_d
        cmd.angular.z = min(self.max_angular_vel, max(-self.max_angular_vel, cmd.angular.z))
        
        # Linear velocity control: be aggressive from far away, precise when close
        # Only move forward when reasonably aligned (< 60 degrees for some motion, < 30 for full speed)
        if yaw_error_mag < 1.047:  # ~60 degrees: allow forward motion with rotation
            # Use PID to calculate velocity, with adaptive scaling based on distance
            pid_linear = linear_p + linear_i + linear_d
            
            if distance > 0.2:
                # Far away: go at reasonable speed
                cmd.linear.x = min(self.max_linear_vel, max(0.01, pid_linear))
            elif distance > 0.1:
                # Medium distance: controlled approach
                cmd.linear.x = min(0.2, max(0.01, pid_linear * 0.8))
            else:
                # Very close: ultra-precise control
                cmd.linear.x = min(0.1, max(0.01, pid_linear * 0.5))
        else:
            # Not aligned: rotate in place
            cmd.linear.x = 0.0
        
        # Always clamp to valid range
        cmd.linear.x = max(0.0, cmd.linear.x)
        
        # Info logging: publish command values (no es necesario)
        #self.get_logger().info(
        #    f'[PID] Distance: {distance:.4f}m | Linear: {cmd.linear.x:.3f} m/s | '
        #    f'Yaw: {math.degrees(yaw_error):.1f}° | Angular: {cmd.angular.z:.3f} rad/s'
        #)
        
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
        
        # Target waypoint marker (larger sphere)
        target_marker = Marker()
        target_marker.header.frame_id = 'odom'
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
        current_marker.header.frame_id = 'odom'
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
        line_marker.header.frame_id = 'odom'
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
