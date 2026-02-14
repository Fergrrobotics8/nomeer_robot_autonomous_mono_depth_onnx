#!/usr/bin/env python3
"""
SLAM Auto-Save Wrapper
Automatically saves the map when exiting from mapping mode.

Launches SLAM and handles graceful shutdown with auto-save.
"""

import os
import sys
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node


class SLAMAutoSaveManager:
    def __init__(self):
        self.slam_process = None
        self.mode = None
        
    def get_mode(self):
        """Detect auto mode (mapping if no map, localization if map exists)"""
        home_dir = os.path.expanduser("~")
        default_map = os.path.join(
            home_dir, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data/warehouse_map_serial"
        )
        if os.path.exists(f"{default_map}.posegraph") or os.path.exists(f"{default_map}.data"):
            return 'localization'
        else:
            return 'mapping'
    
    def launch_slam(self):
        """Launch SLAM with auto-detected mode"""
        self.mode = self.get_mode()
        print(f"\n{'='*60}")
        print(f"🚀 Launching SLAM in {self.mode.upper()} mode")
        print(f"{'='*60}\n")
        
        cmd = [
            'ros2', 'launch', 'autonomous_patrol', 'slam.launch.py',
            f'mode:={self.mode}'
        ]
        
        try:
            self.slam_process = subprocess.Popen(cmd)
            self.slam_process.wait()
        except KeyboardInterrupt:
            self.on_exit()
    
    def on_exit(self):
        """Handle graceful shutdown and auto-save"""
        print(f"\n\n{'='*60}")
        print("🛑 SLAM shutting down...")
        
        if self.slam_process:
            self.slam_process.terminate()
            try:
                self.slam_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.slam_process.kill()
        
        # Auto-save if was in mapping mode
        if self.mode == 'mapping':
            print("💾 Auto-saving map...")
            time.sleep(2)  # Give SLAM time to shutdown cleanly
            
            try:
                # Initialize ROS if needed
                if not rclpy.ok():
                    rclpy.init()
                
                # Try to save the map
                save_cmd = ['ros2', 'run', 'autonomous_patrol', 'save_map.py']
                result = subprocess.run(save_cmd, timeout=10, capture_output=True, text=True)
                
                if result.returncode == 0:
                    print("✅ Map saved automatically!")
                else:
                    print(f"⚠️  Failed to auto-save map: {result.stderr}")
                    print("   Run manually: ros2 run autonomous_patrol save_map.py")
            except Exception as e:
                print(f"⚠️  Could not auto-save: {e}")
                print("   Run manually: ros2 run autonomous_patrol save_map.py")
        
        print(f"{'='*60}\n")
        sys.exit(0)


def main():
    manager = SLAMAutoSaveManager()
    
    # Setup signal handlers
    def signal_handler(sig, frame):
        manager.on_exit()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Launch SLAM
    manager.launch_slam()


if __name__ == '__main__':
    main()
