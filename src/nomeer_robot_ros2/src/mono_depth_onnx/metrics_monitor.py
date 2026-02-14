#!/usr/bin/env python3
"""
Real-time depth metrics monitor for B4 validation.
Subscribe to /depth_metric/* topics and display metrics continuously.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import threading
import time
from datetime import datetime

class MetricsMonitor(Node):
    def __init__(self):
        super().__init__('metrics_monitor')
        
        # Initialize metric values
        self.min_depth = None
        self.avg_depth = None
        self.median_depth = None
        self.obstacle_detected = None
        self.last_update = None
        self.frame_count = 0
        
        # Subscribe to all metric topics
        self.create_subscription(Float32, '/depth_metric/min_frontal_depth', 
                                self.min_callback, 10)
        self.create_subscription(Float32, '/depth_metric/avg_frontal_depth', 
                                self.avg_callback, 10)
        self.create_subscription(Float32, '/depth_metric/median_frontal_depth', 
                                self.median_callback, 10)
        self.create_subscription(Bool, '/depth_metric/obstacle_detected', 
                                self.obstacle_callback, 10)
        
        self.get_logger().info('📊 Metrics Monitor iniciado')
        
        # Start display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
    
    def min_callback(self, msg):
        self.min_depth = msg.data
        self.last_update = datetime.now()
        self.frame_count += 1
    
    def avg_callback(self, msg):
        self.avg_depth = msg.data
    
    def median_callback(self, msg):
        self.median_depth = msg.data
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
    
    def display_loop(self):
        """Display metrics every 1 second"""
        while rclpy.ok():
            time.sleep(1)
            self.display_metrics()
    
    def display_metrics(self):
        if all(v is not None for v in [self.min_depth, self.avg_depth, self.median_depth, self.obstacle_detected]):
            print("\033[2J\033[H", end="")  # Clear screen
            print("╔════════════════════════════════════════════════════════════════╗")
            print("║           B4 - Métricas de Profundidad (ONNX)                   ║")
            print("╠════════════════════════════════════════════════════════════════╣")
            print(f"║  Profundidad Mínima (Frontal)       : {self.min_depth:.4f}            ║")
            print(f"║  Profundidad Promedio (Frontal)     : {self.avg_depth:.4f}            ║")
            print(f"║  Profundidad Mediana (Frontal)      : {self.median_depth:.4f}            ║")
            
            obstacle_icon = "🚨 OBSTÁCULO DETECTADO" if self.obstacle_detected else "✅ DESPEJADO"
            print(f"║  Estado Frontal                     : {obstacle_icon:<40} ║")
            print("╠════════════════════════════════════════════════════════════════╣")
            print(f"║  Frames Procesados                  : {self.frame_count:<45} ║")
            if self.last_update:
                print(f"║  Última Actualización               : {self.last_update.strftime('%H:%M:%S'):<45} ║")
            print("╚════════════════════════════════════════════════════════════════╝")
            print("\n💡 Presiona Ctrl+C para salir\n")

def main():
    rclpy.init()
    monitor = MetricsMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n⚠️  Monitor detenido por usuario")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
