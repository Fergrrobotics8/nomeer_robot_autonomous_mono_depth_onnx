#!/usr/bin/env python3
"""
Initial Pose Capturer for SLAM Mapping
When SLAM starts in mapping mode, this captures and saves the robot's initial pose.
On next run, this pose will be used to initialize localization correctly.

Usage: Started automatically by slam.launch.py in mapping mode
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import math
import os
import sys

# Import pose manager
sys.path.insert(0, os.path.dirname(__file__))
from pose_manager import save_initial_pose, clear_initial_pose

class InitialPoseCapturer(Node):
    def __init__(self):
        super().__init__('initial_pose_capturer')
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Try to capture pose after a delay
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.pose_captured = False
        self.attempt = 0
        self.max_attempts = 5
        
        # Clear previous pose on new mapping session
        clear_initial_pose()
        
        self.get_logger().info("[CAPTURE-POSE] Esperando a capturar pose inicial...")
    
    def timer_callback(self):
        """Captura la pose inicial del robot"""
        if self.pose_captured:
            return
        
        self.attempt += 1
        
        try:
            # Obtener transform base_link → map (o base_link → odom si map no existe)
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                frame = 'map'
            except Exception as map_err:
                # Si map no existe, usar odom
                try:
                    transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                    frame = 'odom'
                except Exception as odom_err:
                    if self.attempt >= self.max_attempts:
                        self.get_logger().error(f"[CAPTURE-POSE] No se encontraron frames después de {self.max_attempts} intentos")
                        self.destroy_timer(self.timer)
                    else:
                        self.get_logger().warn(f"[CAPTURE-POSE] Intento {self.attempt}/{self.max_attempts}: Esperando frames...")
                    return
            
            # Extraer posición
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extraer orientación (quaternion → yaw)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convertir quaternion a yaw (theta)
            theta = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            
            # Guardar
            save_initial_pose(x, y, theta)
            self.get_logger().info(f"[CAPTURE-POSE] ✅ Pose guardada desde {frame}: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
            
            self.pose_captured = True
            self.destroy_timer(self.timer)
            
        except Exception as e:
            self.get_logger().error(f"[CAPTURE-POSE] Excepción: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseCapturer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[CAPTURE-POSE] Cerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
