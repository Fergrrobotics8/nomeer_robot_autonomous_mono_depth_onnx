#!/usr/bin/env python3
"""
Odometry Verification Node
Compares the published odometry with Gazebo ground truth to verify accuracy.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetEntityState
import numpy as np


class OdomVerifier(Node):
    """Verify that odometry matches Gazebo ground truth"""

    def __init__(self):
        super().__init__('odom_verifier')
        
        self.declare_parameter('entity_name', 'nomeer_robot')
        self.declare_parameter('check_interval', 5.0)  # seconds
        
        entity_name = self.get_parameter('entity_name').value
        check_interval = self.get_parameter('check_interval').value
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Client for Gazebo entity state
        self.state_client = self.create_client(GetEntityState, '/get_entity_state')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_entity_state...')
        
        # Latest odometry message
        self.latest_odom = None
        
        # Create timer for periodic comparison
        self.check_timer = self.create_timer(check_interval, self.verify_odometry)
        
        self.get_logger().info(f'Odometry Verifier started (checking every {check_interval}s)')

    def odom_callback(self, msg: Odometry):
        """Store latest odometry"""
        self.latest_odom = msg

    def verify_odometry(self):
        """Compare odometry with ground truth"""
        
        if self.latest_odom is None:
            self.get_logger().warn('No odometry received yet')
            return
        
        entity_name = self.get_parameter('entity_name').value
        request = GetEntityState.Request()
        request.name = entity_name
        
        try:
            future = self.state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
            
            if future.done() and future.result():
                response = future.result()
                
                # Extract positions
                odom_x = self.latest_odom.pose.pose.position.x
                odom_y = self.latest_odom.pose.pose.position.y
                odom_z = self.latest_odom.pose.pose.position.z
                
                real_x = response.state.pose.position.x
                real_y = response.state.pose.position.y
                real_z = response.state.pose.position.z
                
                # Calculate differences
                dx = abs(odom_x - real_x)
                dy = abs(odom_y - real_y)
                dz = abs(odom_z - real_z)
                total_error = np.sqrt(dx**2 + dy**2 + dz**2)
                
                # Extract orientations (quaternions)
                odom_q = [
                    self.latest_odom.pose.pose.orientation.x,
                    self.latest_odom.pose.pose.orientation.y,
                    self.latest_odom.pose.pose.orientation.z,
                    self.latest_odom.pose.pose.orientation.w
                ]
                
                real_q = [
                    response.state.pose.orientation.x,
                    response.state.pose.orientation.y,
                    response.state.pose.orientation.z,
                    response.state.pose.orientation.w
                ]
                
                # Calculate quaternion difference (dot product)
                q_diff = abs(np.dot(odom_q, real_q))
                angle_error = np.arccos(min(1.0, q_diff)) * 2  # Convert to angle
                
                # Print verification results
                status = "✅ GOOD" if total_error < 0.1 else "⚠️ ACCEPTABLE" if total_error < 0.3 else "❌ BAD"
                
                self.get_logger().info(
                    f'\n{status} Odometry Verification:\n'
                    f'  Position Error: {total_error:.4f}m (Δx={dx:.4f}, Δy={dy:.4f}, Δz={dz:.4f})\n'
                    f'  Orientation Error: {angle_error:.4f} rad\n'
                    f'  Odom: ({odom_x:.4f}, {odom_y:.4f}, {odom_z:.4f})\n'
                    f'  Real: ({real_x:.4f}, {real_y:.4f}, {real_z:.4f})'
                )
            
        except Exception as e:
            self.get_logger().error(f'Verification failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
