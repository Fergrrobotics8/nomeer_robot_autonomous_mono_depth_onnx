#!/usr/bin/env python3
"""
Quick start guide script for autonomous_patrol package
This script generates example waypoints for testing
"""

import os
import yaml
import math

# Create data directory if it doesn't exist
os.makedirs('data', exist_ok=True)

# Generate example waypoints in a square pattern
waypoints = []
metadata = {
    'recording_date': '2026-02-12',
    'total_waypoints': 8,
    'sampling_mode': 'distance',
    'min_distance': 0.1,
    'sampling_frequency': 5.0,
    'note': 'Example square path for testing'
}

# Define square waypoints
square_points = [
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
]

wp_id = 1
for i, (angle_idx, (x, y)) in enumerate([
    (0, (0.0, 0.0)),
    (1, (1.0, 0.0)),
    (2, (1.0, 1.0)),
    (3, (0.0, 1.0)),
    (4, (0.0, 0.0)),
]):
    waypoints.append({
        'id': wp_id,
        'timestamp': 100 + i * 10,
        'x': float(x),
        'y': float(y),
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0,
        'linear_vel': 0.3,
        'angular_vel': 0.0
    })
    wp_id += 1

# Save to file
data = {
    'metadata': metadata,
    'waypoints': waypoints
}

with open('data/example_waypoints.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

print("✓ Example waypoints generated: data/example_waypoints.yaml")
print(f"✓ Total waypoints: {len(waypoints)}")
print("\nTo use these waypoints, update config/autonomous_patrol_config.yaml:")
print("  follower:")
print("    waypoints_file: 'example_waypoints.yaml'")
