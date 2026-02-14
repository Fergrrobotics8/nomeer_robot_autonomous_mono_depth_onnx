# AUTONOMOUS PATROL - Technical Specifications

## Project Overview

Implementation of autonomous waypoint-based navigation system for ROS 2 compatible robots. Includes recording, playback, visualization, and comprehensive metrics generation.

## Implementation Status - Part A (Autonomía por Waypoints)

### ✅ A1: Recording Waypoints - COMPLETE

**File**: [autonomous_patrol/record_waypoints_node.py](autonomous_patrol/record_waypoints_node.py)

**Features:**
- Subscribes to `/odom` topic for odometry data
- Two sampling modes:
  - `distance`: Records waypoint when traveled distance exceeds threshold
  - `frequency`: Records at fixed frequency (Hz)
- Saves waypoints to YAML format with metadata
- Real-time status publishing

**Parameters** (via config file):
```yaml
recorder:
  sampling_mode: "distance"              # "distance" or "frequency"
  min_distance_between_waypoints: 0.1    # meters
  sampling_frequency: 5.0                # Hz
  output_file: "waypoints.yaml"
  data_directory: "data"
```

**Data Structure** (Waypoint):
- Local position (x, y, z)
- Orientation quaternion (qx, qy, qz, qw)
- Velocity (linear, angular)
- Timestamp
- Unique ID

**Output Format**: YAML file with metadata and waypoint array

---

### ✅ A2: Autonomous Following - COMPLETE

**File**: [autonomous_patrol/follow_waypoints_node.py](autonomous_patrol/follow_waypoints_node.py)

**Features:**
- Loads waypoints from YAML file
- Executes sequential waypoint following
- Publishes velocity commands on `/cmd_vel`
- Odometry feedback for control loop
- Simple proportional velocity control
- Real-time metrics tracking
- Automatic metrics generation on completion

**Parameters**:
```yaml
follower:
  waypoints_file: "waypoints.yaml"
  waypoint_tolerance: 0.2               # meters - arrival threshold
  max_linear_velocity: 0.5              # m/s
  max_angular_velocity: 1.0             # rad/s
  control_frequency: 10.0               # Hz
  use_yaw_control: false                # true for orientation control
```

**Control Algorithm**: 
- Linear velocity proportional to distance
- Optional angular velocity control (configurable)
- Waypoint change when distance < tolerance

**Status Publishing**:
- `/waitpoint_follower/status`: Execution status messages
- `/waypoint_follower/current_waypoint`: Current waypoint index (Int32)
- `/waypoint_follower/markers`: Visualization markers

---

### ✅ A3: Visualization in RViz - COMPLETE

**File**: [autonomous_patrol/visualizer_node.py](autonomous_patrol/visualizer_node.py)

**Features:**
- Publishes all waypoints as sphere markers (color-coded by progress)
- Draws complete trajectory as line
- Labels every 10% of waypoints for clarity
- Publishes in real-time
- Configurable publish frequency

**Published Topics**:
- `/waypoint_visualizer/waypoints` (MarkerArray): Individual waypoints
- `/waypoint_visualizer/trajectory` (Marker): Complete path

**RViz Configuration**: [rviz/waypoints.rviz](rviz/waypoints.rviz)
- Pre-configured with all necessary displays
- Fixed frame: `odom`

---

### ✅ A4: Metrics Generation - COMPLETE

**Metrics Tracked**:

1. **Execution Summary**
   - Total execution time (seconds)
   - Waypoints completed / total
   - Success status (boolean)
   - Execution timestamp

2. **Error Metrics**
   - Mean error to waypoint (meters)
   - Maximum error to waypoint (meters)
   - Minimum error to waypoint (meters)

3. **Timing Metrics**
   - Mean time per waypoint transition (seconds)
   - Maximum transition time
   - Minimum transition time

4. **Per-Waypoint Data**
   - Individual error for each waypoint
   - Individual transition time

**Output**: [results/metrics.json](results/metrics.json) in JSON format

---

## Architecture Overview

```
autonomous_patrol/
├── autonomous_patrol/                  # Python package
│   ├── record_waypoints_node.py       # A1: Recording node
│   ├── follow_waypoints_node.py       # A2: Following node
│   └── visualizer_node.py             # A3: Visualization node
├── config/
│   ├── autonomous_patrol_config.yaml  # Main configuration
│   └── test_config.yaml               # Test configuration
├── launch/
│   ├── record_waypoints.launch.py     # Launch recording
│   ├── follow_waypoints.launch.py     # Launch following + viz
│   └── visualize_waypoints.launch.py  # Launch visualization only
├── data/                              # Waypoint storage
│   └── example_waypoints.yaml         # Example data
├── results/                           # Metrics output
│   └── metrics.json                   # Execution results
├── rviz/
│   └── waypoints.rviz                 # RViz config
└── README.md                          # Full documentation
```

---

## ROS 2 Topics

### Input Topics
- `/odom` (Odometry): Robot odometry [REQUIRED]

### Output Topics
- `/cmd_vel` (Twist): Velocity commands to robot
- `/waypoint_recorder/status` (String): Recorder status
- `/waypoint_follower/status` (String): Follower status
- `/waypoint_follower/current_waypoint` (Int32): Current waypoint index
- `/waypoint_follower/markers` (MarkerArray): Real-time control visualization
- `/waypoint_visualizer/waypoints` (MarkerArray): All waypoints
- `/waypoint_visualizer/trajectory` (Marker): Trajectory path

---

## Workflow

### Recording Phase
1. Start robot simulator (Gazebo)
2. Launch `record_waypoints.launch.py`
3. Teleoperate robot through desired path
4. Press Ctrl+C to stop and save waypoints
5. Waypoints saved to `data/waypoints.yaml`

### Following Phase
1. Launch `follow_waypoints.launch.py`
2. Robot executes waypoint following autonomously
3. Visualization shows progress in RViz
4. Metrics automatically generated in `results/metrics.json`

---

## Configuration Best Practices

### For High Precision
```yaml
min_distance_between_waypoints: 0.05    # More waypoints
waypoint_tolerance: 0.1                 # Strict arrival
max_linear_velocity: 0.3                # Conservative speed
```

### For Speed
```yaml
min_distance_between_waypoints: 0.2     # Fewer waypoints
waypoint_tolerance: 0.3                 # Relaxed arrival
max_linear_velocity: 0.8                # Higher speed
```

### For Real-time Performance
```yaml
sampling_frequency: 20.0                # Higher control rate
control_frequency: 20.0                 # Match frequencies
```

---

## File Formats

### Waypoints YAML Structure
```yaml
metadata:
  recording_date: ISO timestamp
  total_waypoints: integer
  sampling_mode: "distance" | "frequency"
  min_distance: float
  sampling_frequency: float

waypoints:
  - id: integer
    timestamp: float (seconds)
    x: float
    y: float
    z: float
    qx: float (quaternion x)
    qy: float (quaternion y)
    qz: float (quaternion z)
    qw: float (quaternion w)
    linear_vel: float
    angular_vel: float
```

### Metrics JSON Structure
```json
{
  "execution_summary": {
    "total_execution_time": float,
    "waypoints_completed": integer,
    "total_waypoints": integer,
    "success": boolean,
    "execution_date": string
  },
  "error_metrics": {
    "mean_error_to_waypoint": float,
    "max_error_to_waypoint": float,
    "min_error_to_waypoint": float
  },
  "timing_metrics": {
    "mean_time_per_waypoint": float,
    "max_time_per_waypoint": float,
    "min_time_per_waypoint": float
  },
  "per_waypoint_data": [...]
}
```

---

## Dependencies

### ROS 2 Packages
- `rclpy`: ROS 2 Python client
- `geometry_msgs`: Twist, Point messages
- `nav_msgs`: Odometry message
- `visualization_msgs`: Marker, MarkerArray
- `std_msgs`: String, Int32 messages
- `tf2`: Coordinate transformation
- `rviz2`: Visualization tool

### Python Packages
- `PyYAML`: YAML file handling
- Standard library: math, json, time, os, dataclasses

---

## Compilation & Installation

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_patrol
source install/setup.bash
```

Status: ✅ **Compiles successfully**

---

## Testing

### Example Waypoints
Generated script: [generate_example_waypoints.py](generate_example_waypoints.py)

Creates simple square path for testing:
- 5 waypoints forming a square
- Easy to verify visually in RViz

Run with:
```bash
python3 generate_example_waypoints.py
```

---

## Known Limitations

1. **Control**: Simple proportional control - no PID or advanced path planning
2. **Orientation**: Optional yaw control only - no full SE(2) tracking
3. **Obstacles**: No obstacle avoidance implemented
4. **Frame Assumption**: Assumes "odom" frame as global reference
5. **Performance**: Linear velocity proportional to distance may cause oscillation near waypoints

---

## Future Enhancements

1. PID controller for better tracking
2. Obstacle avoidance (costmap-based)
3. Dynamic reconfiguration of parameters
4. ROS 2 Action interface for client-server pattern
5. Spline interpolation between waypoints
6. Multi-robot coordination
7. Adaptive velocity control

---

## Version Information

- **Package Version**: 0.0.1
- **ROS 2 Distribution**: Humble
- **Python Version**: 3.10+
- **Date**: 2026-02-12

---

## Author

Abdullah Nomeer (abdullahnomeer@gmail.com)

## License

Apache License 2.0
