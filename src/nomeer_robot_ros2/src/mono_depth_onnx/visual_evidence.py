#!/usr/bin/env python3
"""
B5 - Visual Evidence Recording
Real-time visualization of depth map + metrics evolution
Saves data and graphs for documentation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from datetime import datetime
import os

class VisualEvidenceRecorder(Node):
    def __init__(self):
        super().__init__('visual_evidence_recorder')
        
        # Setup directories
        self.output_dir = os.path.expanduser('~/ros2_ws/results/visual_evidence')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Data storage
        self.max_history = 500
        self.timestamps = deque(maxlen=self.max_history)
        self.min_depths = deque(maxlen=self.max_history)
        self.avg_depths = deque(maxlen=self.max_history)
        self.median_depths = deque(maxlen=self.max_history)
        self.obstacles = deque(maxlen=self.max_history)
        
        # Current frame
        self.current_depth_colored = None
        self.current_depth_map = None
        self.frame_count = 0
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(Image, '/camera/depth_colored', 
                                self.depth_colored_callback, 10)
        self.create_subscription(Image, '/camera/depth_estimated',
                                self.depth_estimated_callback, 10)
        self.create_subscription(Float32, '/depth_metric/min_frontal_depth',
                                self.min_callback, 10)
        self.create_subscription(Float32, '/depth_metric/avg_frontal_depth',
                                self.avg_callback, 10)
        self.create_subscription(Float32, '/depth_metric/median_frontal_depth',
                                self.median_callback, 10)
        self.create_subscription(Bool, '/depth_metric/obstacle_detected',
                                self.obstacle_callback, 10)
        
        self.get_logger().info(f'📊 Visual Evidence Recorder - Output: {self.output_dir}')
    
    def depth_colored_callback(self, msg):
        try:
            self.current_depth_colored = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
            
            # Save depth map snapshot every 100 frames
            if self.frame_count % 100 == 0:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                snapshot_path = os.path.join(self.output_dir, f'depth_map_{self.frame_count:04d}.png')
                cv2.imwrite(snapshot_path, self.current_depth_colored)
                self.get_logger().info(f'📸 Frame {self.frame_count}: Guardado {snapshot_path}')
                
        except Exception as e:
            self.get_logger().error(f'Error in depth_colored: {e}')
    
    def depth_estimated_callback(self, msg):
        try:
            self.current_depth_map = self.bridge.imgmsg_to_cv2(msg, 'mono16')
        except Exception as e:
            self.get_logger().error(f'Error in depth_estimated: {e}')
    
    def min_callback(self, msg):
        self.min_depths.append(float(msg.data))
        self.timestamps.append(datetime.now())
    
    def avg_callback(self, msg):
        self.avg_depths.append(float(msg.data))
    
    def median_callback(self, msg):
        self.median_depths.append(float(msg.data))
    
    def obstacle_callback(self, msg):
        self.obstacles.append(1.0 if msg.data else 0.0)
    
    def save_final_report(self):
        """Save comprehensive report with matplotlib"""
        if not self.min_depths:
            self.get_logger().warning('❌ No data collected')
            return
        
        try:
            import matplotlib
            matplotlib.use('Agg')  # Use non-interactive backend
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().error(f'⚠️  Matplotlib error: {e}. Guardando solo CSV.')
            self.save_csv_only()
            return
        
        # Create comprehensive report figure
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('B5 - EVIDENCIA VISUAL: Evolución de Métricas de Profundidad', 
                    fontsize=16, fontweight='bold')
        
        # Plot 1: All metrics together
        ax1 = axes[0, 0]
        ax1.plot(range(len(self.min_depths)), list(self.min_depths), label='Mín', marker='o', markersize=2)
        ax1.plot(range(len(self.avg_depths)), list(self.avg_depths), label='Prom', marker='s', markersize=2)
        ax1.plot(range(len(self.median_depths)), list(self.median_depths), label='Med', marker='^', markersize=2)
        ax1.set_title('Evolución Combinada de Profundidades', fontweight='bold')
        ax1.set_ylabel('Profundidad Normalizada (0-1)')
        ax1.set_xlabel('Muestras')
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([0, 1])
        
        # Plot 2: Min depth with threshold
        ax2 = axes[0, 1]
        ax2.plot(range(len(self.min_depths)), list(self.min_depths), 'r-', linewidth=2, label='Min Depth')
        ax2.axhline(y=0.5, color='red', linestyle='--', label='Threshold (0.5)', linewidth=2)
        ax2.fill_between(range(len(self.min_depths)), 0, 0.5, alpha=0.2, color='red', label='Zona de Obstáculo')
        ax2.set_title('Detección de Obstáculos', fontweight='bold')
        ax2.set_ylabel('Profundidad')
        ax2.set_xlabel('Muestras')
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Obstacle timeline
        ax3 = axes[1, 0]
        colors_timeline = ['red' if x > 0 else 'green' for x in self.obstacles]
        ax3.bar(range(len(self.obstacles)), [1]*len(self.obstacles), color=colors_timeline, alpha=0.7, width=1.0)
        ax3.set_title('Timeline de Detección de Obstáculos', fontweight='bold')
        ax3.set_ylabel('Estado')
        ax3.set_xlabel('Muestras')
        ax3.set_yticks([])
        ax3.set_ylim([0, 1.2])
        
        # Plot 4: Statistics
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        stats_text = f"""ESTADÍSTICAS DE PROFUNDIDAD

Profundidad Mínima (Frontal):
  Min: {min(self.min_depths):.4f}
  Max: {max(self.min_depths):.4f}
  Prom: {np.mean(list(self.min_depths)):.4f}
  Std Dev: {np.std(list(self.min_depths)):.4f}

Profundidad Mediana (Frontal):
  Min: {min(self.median_depths):.4f}
  Max: {max(self.median_depths):.4f}
  Prom: {np.mean(list(self.median_depths)):.4f}
  Std Dev: {np.std(list(self.median_depths)):.4f}

Detección de Obstáculos:
  Total Muestras: {len(self.obstacles)}
  Obstáculos Detectados: {int(sum(self.obstacles))}
  % Despejado: {100*(1 - np.mean(list(self.obstacles))):.1f}%
  % Con Obstáculo: {100*np.mean(list(self.obstacles)):.1f}%
        """
        
        ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
                fontsize=10, verticalalignment='top', family='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        plt.tight_layout()
        
        report_path = os.path.join(self.output_dir, 'EVIDENCIA_VISUAL_B5.png')
        plt.savefig(report_path, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'✅ REPORTE VISUAL GUARDADO: {report_path}')
        plt.close()
        
        # Save data as CSV
        self.save_csv_only()
    
    def save_csv_only(self):
        """Save metrics data as CSV"""
        csv_path = os.path.join(self.output_dir, 'metrics_data.csv')
        with open(csv_path, 'w') as f:
            f.write('frame,timestamp,min_depth,avg_depth,median_depth,obstacle\n')
            for i in range(len(self.min_depths)):
                ts = self.timestamps[i].isoformat() if i < len(self.timestamps) else 'N/A'
                f.write(f'{i},{ts},{self.min_depths[i]:.6f},{self.avg_depths[i]:.6f},'
                       f'{self.median_depths[i]:.6f},{int(self.obstacles[i])}\n')
        self.get_logger().info(f'📊 DATOS EXPORTADOS (CSV): {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    recorder = VisualEvidenceRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print('\n⏹️  GENERANDO REPORTE FINAL B5...')
        recorder.save_final_report()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

