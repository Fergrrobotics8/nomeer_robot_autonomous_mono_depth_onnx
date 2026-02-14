#!/usr/bin/env python3
"""
Visualize the depth map from the depth inference node in real-time.
Subscribe to /camera/depth_colored and display it using OpenCV.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth_colored',
            self.depth_callback,
            10
        )
        self.get_logger().info('🔍 Depth Viewer iniciado. Esperando mapas de profundidad...')
        
    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add timestamp to display
            timestamp = msg.header.stamp.sec
            frame_text = f"Mapa de Profundidad | Timestamp: {timestamp}"
            cv2.putText(depth_cv, frame_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display
            cv2.imshow('Mapa de Profundidad - MiDaS ONNX', depth_cv)
            
            # Press 'q' to quit, 's' to save
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('❌ Cerrando visor...')
                rclpy.shutdown()
            elif key == ord('s'):
                filename = f'depth_map_{timestamp}.png'
                cv2.imwrite(filename, depth_cv)
                self.get_logger().info(f'✅ Mapa guardado: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')

def main():
    rclpy.init()
    viewer = DepthViewer()
    
    print("╔════════════════════════════════════════════════════╗")
    print("║     Visor de Mapa de Profundidad - MiDaS ONNX      ║")
    print("╠════════════════════════════════════════════════════╣")
    print("║  Controles:                                        ║")
    print("║    q -> Salir                                      ║")
    print("║    s -> Guardar imagen actual                      ║")
    print("╚════════════════════════════════════════════════════╝")
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\n⚠️  Interrumpido por usuario")
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
