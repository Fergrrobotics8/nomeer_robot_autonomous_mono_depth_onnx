#!/usr/bin/env python3
"""
Save/Load initial robot pose with the map.
When SLAM starts mapping, this saves the robot's initial pose.
When loading a saved map for localization, it restores that pose.
"""

import json
import os

HOME = os.path.expanduser("~")
POSE_FILE = os.path.join(
    HOME, "ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data/warehouse_map_initial_pose.json"
)

def save_initial_pose(x, y, theta):
    """Save the robot's initial pose"""
    pose_data = {
        "x": float(x),
        "y": float(y),
        "theta": float(theta)
    }
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    with open(POSE_FILE, 'w') as f:
        json.dump(pose_data, f, indent=2)
    print(f"[POSE] Guardada pose inicial: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")

def load_initial_pose():
    """Load the robot's initial pose"""
    if not os.path.exists(POSE_FILE):
        return None, None, None
    
    with open(POSE_FILE, 'r') as f:
        pose_data = json.load(f)
    
    x = pose_data.get("x", 0.0)
    y = pose_data.get("y", 0.0)
    theta = pose_data.get("theta", 0.0)
    print(f"[POSE] Cargada pose inicial: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
    return x, y, theta

def clear_initial_pose():
    """Clear saved pose (for new mapping session)"""
    if os.path.exists(POSE_FILE):
        os.remove(POSE_FILE)
        print("[POSE] Pose anterior borrada")
