#!/usr/bin/env python3
"""
Continuous Map Saver - saves SLAM map periodically while running
Ejecutado automáticamente por slam.launch.py en paralelo con SLAM
"""

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph
import os
import time

class ContinuousMapSaver(Node):
    def __init__(self):
        super().__init__('continuous_map_saver')
        
        # Map path
        home = os.path.expanduser("~")
        self.map_path = os.path.join(
            home, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data/warehouse_map_serial"
        )
        
        # Save interval (segundos)
        self.save_interval = 5.0
        self.last_save_time = 0
        
        # Cliente del servicio
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
        # Timer que intenta guardar periódicamente
        self.timer = self.create_timer(self.save_interval, self.timer_callback)
        
        self.get_logger().info(f"[MAP-SAVER] Iniciado. Guardando cada {self.save_interval}s a {self.map_path}")
    
    def timer_callback(self):
        """Guarda el mapa periódicamente"""
        try:
            if not self.serialize_client.service_is_ready():
                # El servicio aún no está listo (SLAM aún iniciando)
                return
            
            # Crear request
            request = SerializePoseGraph.Request()
            request.filename = self.map_path
            
            # Llamar al servicio de forma sincrónica con timeout
            future = self.serialize_client.call_async(request)
            
            # Esperar resultado con timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"[MAP-SAVER] ✅ Mapa guardado ({os.path.getsize(self.map_path + '.posegraph')} bytes)")
                    self.last_save_time = time.time()
                else:
                    self.get_logger().warn("[MAP-SAVER] ⚠️ Servicio retornó error")
            else:
                self.get_logger().warn("[MAP-SAVER] ⚠️ Timeout al guardar")
        
        except Exception as e:
            self.get_logger().warn(f"[MAP-SAVER] Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousMapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[MAP-SAVER] Cerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
