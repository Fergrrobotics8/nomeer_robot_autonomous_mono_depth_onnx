#!/usr/bin/env python3
"""
Autonomous Depth Safety Node
Integrates depth metrics with autonomous navigation.
Implements safety rules based on depth estimation.

Optional but recommended for safe autonomous operation.
"""

import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import Odometry


class AutonomousDepthSafetyNode(Node):
    """Integrates depth safety with autonomous navigation."""

    def __init__(self):
        super().__init__('autonomous_depth_safety')
        
        # Declare parameters
        self.declare_parameter('min_safe_depth', 0.2)
        self.declare_parameter('emergency_stop_depth', 0.1)
        self.declare_parameter('reduce_speed_depth', 0.3)
        self.declare_parameter('speed_reduction_factor', 0.5)
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('log_safety_events', True)
        self.declare_parameter('results_directory', 'results')
        
        # Get parameters
        self.min_safe_depth = self.get_parameter('min_safe_depth').value
        self.emergency_stop_depth = self.get_parameter('emergency_stop_depth').value
        self.reduce_speed_depth = self.get_parameter('reduce_speed_depth').value
        self.speed_factor = self.get_parameter('speed_reduction_factor').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.log_events = self.get_parameter('log_safety_events').value
        self.results_dir = self.get_parameter('results_directory').value
        
        # Create results directory
        os.makedirs(self.results_dir, exist_ok=True)
        
        # State variables
        self.current_cmd_vel = Twist()
        self.min_depth = 1.0
        self.obstacle_detected = False
        self.emergency_stop_active = False
        self.safety_events = []
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',  # Original cmd_vel from follower
            self.cmd_vel_callback,
            10
        )
        
        self.min_depth_sub = self.create_subscription(
            Float32,
            '/depth_metric/min_frontal_depth',
            self.min_depth_callback,
            10
        )
        
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/depth_metric/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_safe_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety/status', 10)
        
        self.get_logger().info(
            f'Autonomous Depth Safety Node initialized\n'
            f'  Min safe depth: {self.min_safe_depth}\n'
            f'  Emergency stop depth: {self.emergency_stop_depth}\n'
            f'  Reduce speed depth: {self.reduce_speed_depth}\n'
            f'  Safety enabled: {self.enable_safety}'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity command from autonomous follower."""
        self.current_cmd_vel = msg

    def min_depth_callback(self, msg: Float32):
        """Receive minimum depth."""
        self.min_depth = msg.data

    def obstacle_callback(self, msg: Bool):
        """Receive obstacle detection status."""
        self.obstacle_detected = msg.data
        
        # Process command with safety rules
        self._process_safety_command()

    def _process_safety_command(self):
        """Apply safety rules and publish modified command."""
        if not self.enable_safety:
            self.cmd_vel_safe_pub.publish(self.current_cmd_vel)
            return
        
        safe_cmd = Twist()
        safe_cmd.linear = self.current_cmd_vel.linear
        safe_cmd.angular = self.current_cmd_vel.angular
        
        status = "✓ NOMINAL"
        
        # Emergency stop rule
        if self.min_depth < self.emergency_stop_depth or self.obstacle_detected:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = 0.0
            
            status = "🚨 EMERGENCY STOP"
            self._log_safety_event("EMERGENCY_STOP", self.min_depth)
            self.emergency_stop_active = True
        
        # Speed reduction rule
        elif self.min_depth < self.reduce_speed_depth:
            safe_cmd.linear.x *= self.speed_factor
            safe_cmd.linear.y *= self.speed_factor
            safe_cmd.linear.z *= self.speed_factor
            
            status = "⚠️  REDUCED SPEED"
            self._log_safety_event("SPEED_REDUCTION", self.min_depth)
            self.emergency_stop_active = False
        
        else:
            status = "✓ NOMINAL"
            self.emergency_stop_active = False
        
        # Publish safe command
        self.cmd_vel_safe_pub.publish(safe_cmd)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"{status} | Depth: {self.min_depth:.4f}m"
        self.safety_status_pub.publish(status_msg)

    def _log_safety_event(self, event_type: str, depth: float):
        """Log safety event."""
        if not self.log_events:
            return
        
        event = {
            'timestamp': datetime.now().isoformat(),
            'event_type': event_type,
            'min_depth': depth,
            'cmd_vel': {
                'linear_x': self.current_cmd_vel.linear.x,
                'angular_z': self.current_cmd_vel.angular.z,
            }
        }
        
        self.safety_events.append(event)
        
        self.get_logger().warn(f'{event_type}: depth={depth:.4f}m')
        
        # Save periodically
        if len(self.safety_events) % 10 == 0:
            self._save_safety_log()

    def _save_safety_log(self):
        """Save safety events log."""
        try:
            log_file = os.path.join(self.results_dir, 'safety_events.json')
            with open(log_file, 'w') as f:
                json.dump(self.safety_events, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save safety log: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDepthSafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
