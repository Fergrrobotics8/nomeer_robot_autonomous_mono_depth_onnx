# Nomeer Robot - Complete Setup & Usage Guide

## Quick Start

```bash
cd ~/ros2_ws

# Source ROS 2 (do this once per terminal session)
source /opt/ros/humble/setup.bash

# Initial setup (one time)
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
pip3 install 'numpy<2' opencv-python onnxruntime PyYAML scipy torch timm onnx onnxscript

# Build packages
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash

# Download AI model
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
cd ~/ros2_ws
```

---

## Part A: Waypoint Recording & Following

### Record a Trajectory

**Terminal 1** - Start simulator:
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch robot_description robot.launch.py
```

**Terminal 2** - Start recorder:
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

**Terminal 3** - Teleop robot:
```bash
cd ~/ros2_ws && source install/setup.bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

**Recording**:
- Use keyboard (w/x for forward/backward, a/d for turn)
- Press Ctrl+C in Terminal 2 to save waypoints
- Waypoints saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/data/waypoints.yaml`

### Play Back Trajectory

**Terminal 1** - Start simulator:
```bash
ros2 launch robot_description robot.launch.py
```

**Terminal 2** - Start follower:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Terminal 3** (Optional) - Visualize in RViz:
```bash
ros2 launch autonomous_patrol visualize_waypoints.launch.py
```

**Results**: Metrics saved to `src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json`

---

## Part B: Monocular Depth with AI (ONNX)

### Quick Start - Full Pipeline

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

This runs complete pipeline with test images:
- Loads RGB images
- Runs MiDaS depth inference
- Calculates depth metrics
- Displays colored depth visualization

### With Webcam (Real-time)

**Terminal 1** - Edit config:
```bash
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```

Change to:
```yaml
image_source:
  source_type: "webcam"
  source_path: "0"
```

**Terminal 2** - Launch:
```bash
colcon build --packages-select mono_depth_onnx
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### With Video File

```bash
# Place video in:
cp your_video.mp4 src/nomeer_robot_ros2/src/mono_depth_onnx/data/video.mp4

# Edit config:
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```

Change to:
```yaml
image_source:
  source_type: "video"
  source_path: "data/video.mp4"
```

Then:
```bash
colcon build --packages-select mono_depth_onnx
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### Inference Only (Subscribe to /rgb_image)

```bash
ros2 run mono_depth_onnx depth_inference_node.py
```

This subscribes to `/rgb_image` and publishes:
- `/camera/depth_estimated` - Raw depth map
- `/camera/depth_colored` - Colored visualization

---

## Integration: Autonomy + Vision + Safety

Run all three together for autonomous navigation with depth-based obstacle detection:

**Terminal 1** - Autonomy:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Terminal 2** - Vision/Depth:
```bash
ros2 run mono_depth_onnx depth_inference_node.py
```

**Terminal 3** - Safety (Emergency stop if obstacle < 0.1m):
```bash
ros2 run mono_depth_onnx autonomous_depth_safety_node.py
```

---

## Important Files

```bash
# Recorded waypoints
src/nomeer_robot_ros2/src/autonomous_patrol/data/waypoints.yaml

# Execution metrics
src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json

# Part A config
src/nomeer_robot_ros2/src/autonomous_patrol/config/autonomous_patrol_config.yaml

# Part B config
src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml

# AI model
src/nomeer_robot_ros2/src/mono_depth_onnx/models/midas_v21_small.onnx
```

---

## Available ROS 2 Topics

### Part A (Autonomy)
- `/cmd_vel` - Robot velocity commands
- `/odom` - Robot odometry
- `/waypoint_follower/current_waypoint` - Current waypoint index
- `/waypoint_follower/status` - Status string

### Part B (Vision/Depth)
- `/rgb_image` - Input RGB image (for external sources)
- `/camera/depth_estimated` - Depth map (mono16, 0-65535)
- `/camera/depth_colored` - Colored depth visualization
- `/depth_metric/min_frontal_depth` - Minimum frontal depth (0-1)
- `/depth_metric/mean_depth` - Mean depth (0-1)
- `/depth_metric/obstacle_detected` - Boolean: obstacle present?

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Package not found" | `colcon build && source install/setup.bash` |
| "numpy: _ARRAY_API error" | `pip3 install 'numpy<2'` |
| "ONNX model not found" | `cd src/nomeer_robot_ros2/src/mono_depth_onnx && python3 scripts/download_midas_model.py` |
| "Gazebo won't open" | `sudo apt install -y ros-humble-gazebo-ros` |
| "Can't see topics" | Ensure `source install/setup.bash` in all terminals |
| "Permission denied" | `chmod +x src/nomeer_robot_ros2/src/*/scripts/*.py` |

---

## Verify Installation

```bash
cd ~/ros2_ws
bash verify_installation.sh
```

Expected output: ✅ 38/38 CHECKS PASSED

---

## Build Only Specific Package

```bash
# Autonomy only
colcon build --packages-select autonomous_patrol

# Vision/Depth only
colcon build --packages-select mono_depth_onnx

# Robot description only
colcon build --packages-select robot_description
```

---

## Clean Build

```bash
cd ~/ros2_ws
colcon clean all
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash
```

---

**System**: ROS 2 Humble  
**Date**: February 14, 2026  
**Status**: ✅ Production Ready
