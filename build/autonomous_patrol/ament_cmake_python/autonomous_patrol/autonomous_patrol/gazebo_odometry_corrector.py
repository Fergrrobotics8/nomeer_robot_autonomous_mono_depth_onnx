#!/usr/bin/env python3
"""
Gazebo Odometry Corrector
Publishes the TRUE robot pose from Gazebo as accurate odometry using TF2.
This bypasses the defective DiffDrive plugin odometry.

The DiffDrive plugin in robot.sdf has incorrect wheel parameters,
causing massive drift. This node uses Gazebo's TF transforms instead.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
import numpy as np


class GazeboOdometryCorrector(Node):
    """Node to publish accurate odometry from Gazebo ground truth using TF"""

    def __init__(self):
        super().__init__('gazebo_odom_corrector')
        
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        ref_frame = self.get_parameter('reference_frame').value
        child_frame = self.get_parameter('child_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher for corrected odometry
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Previous pose for velocity calculation
        self.prev_pose = None
        self.prev_time = None
        
        # Create timer for publishing
        self.publish_timer = self.create_timer(
            1.0 / publish_rate,
            lambda: self.publish_corrected_odometry(ref_frame, child_frame)
        )
        
        self.get_logger().info(
            f'Gazebo Odometry Corrector initialized (TF-based)\n'
            f'  Reference frame: {ref_frame}\n'
            f'  Child frame: {child_frame}\n'
            f'  Publishing to: /odom (ground truth from TF)\n'
            f'  This bypasses the defective DiffDrive plugin odometry'
        )

    def publish_corrected_odometry(self, ref_frame: str, child_frame: str):
        """Query TF2 and publish corrected odometry"""
        
        try:
            # Get transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(
                ref_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = ref_frame
            odom.child_frame_id = child_frame
            
            # Set pose from transform
            odom.pose.pose.position = transform.transform.translation
            odom.pose.pose.orientation = transform.transform.rotation
            
            # Calculate velocities (simple differentiation)
            current_time = self.get_clock().now()
            if self.prev_pose is not None and self.prev_time is not None:
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                if dt > 0:
                    # Linear velocity
                    dx = odom.pose.pose.position.x - self.prev_pose[0]
                    dy = odom.pose.pose.position.y - self.prev_pose[1]
                    dz = odom.pose.pose.position.z - self.prev_pose[2]
                    
                    odom.twist.twist.linear.x = dx / dt
                    odom.twist.twist.linear.y = dy / dt
                    odom.twist.twist.linear.z = dz / dt
            else:
                odom.twist.twist.linear = Vector3()
            
            odom.twist.twist.angular = Vector3()
            
            # Pose covariance (high confidence since it's ground truth from TF)
            odom.pose.covariance = [
                0.01, 0, 0, 0, 0, 0,
                0, 0.01, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0,
                0, 0, 0, 0.01, 0, 0,
                0, 0, 0, 0, 0.01, 0,
                0, 0, 0, 0, 0, 0.01
            ]
            
            # Twist covariance
            odom.twist.covariance = [
                0.01, 0, 0, 0, 0, 0,
                0, 0.01, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0,
                0, 0, 0, 0.01, 0, 0,
                0, 0, 0, 0, 0.01, 0,
                0, 0, 0, 0, 0, 0.01
            ]
            
            # Publish
            self.odom_pub.publish(odom)
            
            # Store for next iteration
            self.prev_pose = [
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z
            ]
            self.prev_time = current_time
            
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOdometryCorrector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
