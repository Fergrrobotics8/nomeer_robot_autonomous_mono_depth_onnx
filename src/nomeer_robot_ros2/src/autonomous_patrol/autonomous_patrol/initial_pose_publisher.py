#!/usr/bin/env python3
"""
Initial Pose Restorer for SLAM Localization
Publishes the saved initial pose when SLAM starts in localization mode.
This ensures the robot knows where it is in the loaded map.

Usage: Started automatically by slam.launch.py in localization mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import math

# Import pose manager
import sys
sys.path.insert(0, os.path.dirname(__file__))
from pose_manager import load_initial_pose

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Publisher para /initialpose (topic que lee slam_toolbox)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Esperar a que SLAM esté listo
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pose_published = False
        
        self.get_logger().info("[INIT-POSE] Esperando a que SLAM inicie...")
    
    def timer_callback(self):
        """Publica la pose inicial una vez cuando SLAM esté listo"""
        if self.pose_published:
            return
        
        # Cargar pose guardada
        x, y, theta = load_initial_pose()
        if x is None:
            self.get_logger().warn("[INIT-POSE] No hay pose guardada, usando (0, 0, 0)")
            x, y, theta = 0.0, 0.0, 0.0
        
        # Crear mensaje PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Posición
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        
        # Orientación (convertir theta a quaternion)
        half_theta = float(theta) / 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_theta)
        msg.pose.pose.orientation.w = math.cos(half_theta)
        
        # Covarianza (confianza alta en la posición inicial)
        # Matriz 6x6 pero como lista plana de 36 valores
        msg.pose.covariance = (
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,       # x variance
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,       # y variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # z variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # rx variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # ry variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0687      # rz variance
        )
        
        # Publicar
        self.publisher.publish(msg)
        self.get_logger().info(f"[INIT-POSE] ✅ Pose publicada: x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad")
        
        self.pose_published = True
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[INIT-POSE] Cerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
