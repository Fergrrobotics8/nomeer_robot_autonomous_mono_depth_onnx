# Fergrbot (based on Nomeer Robot) - Autonomous Waypoint Navigation

A ROS 2 Humble-based autonomous robotic system featuring:
- **Waypoint-based autonomous navigation** with trajectory recording and playback
- **Real-time monocular depth estimation** using ONNX-accelerated AI (MiDaS)
- **Obstacle detection** with emergency stop capabilities

![Autonomous Robot Demo](assets/depth_mono_onnx.gif)

## System Setup from Scratch (Ubuntu)

**Prerequisites: Ubuntu 22.04.5 or similar**

> **Tip:** See [Tested Configurations](#tested-configurations) below for verified setups (VirtualBox specs, GPU notes, etc.) before starting installation.

### 1. System Locale Configuration

Ensure proper locale for ROS 2:

```bash
sudo apt update && sudo apt upgrade -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Verify locale setup:
```bash
locale  # Should show en_US.UTF-8 for LANG and LC_CTYPE at minimum
```

### 2. Add Official ROS 2 Repository

Install prerequisites:
```bash
sudo apt install software-properties-common curl -y
```

Add ROS 2 signing key:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add ROS 2 repository:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### 3. Install ROS 2 Humble Desktop

```bash
sudo apt install ros-humble-desktop -y
```

This includes ROS 2, RViz, and Gazebo Harmonic (Ignition Gazebo 6.x) with modern simulation support.

**Note on Gazebo**: This project uses **Gazebo Harmonic** (Ignition Gazebo 6), which is the current default in ROS 2 Humble. Do NOT install `ros-humble-gazebo-ros` (it's for the legacy Gazebo 11) or attempt other Gazebo versions. The correct packages (`ros-gz-sim`, `ros-gz-bridge`, `ros-gz-interfaces`) will be installed by `rosdep`.

### 4. Install Development Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete python3-pip -y
```

### 5. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

---


## Quick Start (After System Setup)

### Step 1: Create workspace and source ROS 2

```bash
mkdir -p ~/ros2_ws
source /opt/ros/humble/setup.bash
```

### Step 2: Clone the robot repository

Clone the repository into the `ros2_ws` directory:

```bash
cd ~
git clone https://github.com/Fergrrobotics8/robot_autonomous_patrol_mono_depth_onnx.git ros2_ws
cd ~/ros2_ws
```

This command downloads the complete project structure (including `src/` and all packages) into `~/ros2_ws/`.

### Step 3: Install system and ROS dependencies

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Install Python dependencies

```bash
sudo pip3 install 'numpy<2' opencv-python onnxruntime PyYAML scipy torch timm onnx onnxscript
```

### Step 5: Build all packages

```bash
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash
```

### Step 6: Download AI model

```bash
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
cd ~/ros2_ws
```

### Pro Tip: Automatic Setup

Add this line to `~/.bashrc` to avoid sourcing every time you open a new terminal:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Important:** After modifying YAML configs or parameters, you must:
1. Run `colcon build` to rebuild packages
2. Source the setup in your current terminal: `source install/setup.bash` (or close and reopen terminal)

### Every Session

In each new terminal (unless you added the auto-setup line above):
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Part A: Autonomous Waypoint Navigation

### Recording a Trajectory

**IMPORTANT: Follow the order carefully!**

1. Terminal 1 - Start robot simulator:
```bash
ros2 launch robot_description robot.launch.py
```
   - Gazebo and RViz will open automatically
   - **IMPORTANT:** Press the PLAY button in Gazebo to start the simulation

### IF YOU WANT TO RECORD WAYPOINTS

2. Terminal 2 - Teleoperate robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Terminal 3 - Start waypoint recorder:
```bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

4. Navigate the robot using keyboard (w/x forward/backward, a/d turn)
5. Press Ctrl+C in Terminal 3 to save waypoints

Waypoints saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/data/waypoints.yaml`



## Part B: Monocular Depth Vision with AI

**Advanced Configuration**

### Depth Inference and depth metric 

For testing depth vision independently:

1. Terminal 1 - Start depth inference:
```bash
ros2 launch mono_depth_onnx inference.launch.py
```

You can view the depth coloured in the rviz visualizer and see if there is an obstacle in the terminal (CLEAR=NO OBSTACLE)

The viewer displays:
- Colored depth visualization (red=close, blue=far)
- Depth metrics in real-time

### Depth Configuration

Edit configuration file for advanced settings:
```bash
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```

Key parameters:
- `roi_x_start/roi_x_end`: Region of interest (horizontal, 0-1)
- `roi_y_start/roi_y_end`: Region of interest (vertical, 0-1)
- `obstacle_threshold`: Detection threshold (0=far, 1=close)
- `outlier_percentile_low/high`: Outlier filtering (5/95 = remove extreme 5%)

### Published Topics

The depth pipeline publishes:
- `/camera/depth_estimated` - Raw depth map (mono16, 0-65535)
- `/camera/depth_colored` - Colored visualization (BGR8)
- `/depth_metric/median_frontal_depth` - Median depth in ROI (0-1)
- `/depth_metric/min_frontal_depth` - Minimum depth in ROI (0-1)
- `/depth_metric/avg_frontal_depth` - Average depth in ROI (0-1)
- `/depth_metric/obstacle_detected` - Boolean flag (1=obstacle, 0=clear)

## Playing Back Trajectory

**No need to close anything** Simply:

1. In Terminal 4 (or a new terminal), start the waypoint follower:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

2. The robot will automatically follow the saved trajectory. Watch the visualization in RViz that is already open

**Metrics** are automatically saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json`



## Obstacle Detection Configuration

The autonomous navigation system includes real-time obstacle detection using monocular depth estimation:
- During waypoint following, if frontal depth median falls below the configured threshold, the robot **stops immediately**
- Obstacle event is logged in metrics.json
- Configure sensitivity in:

```yaml
# ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
depth_metric:
  obstacle_threshold: 0.7  # Increase for less sensitivity, decrease for more
  roi_x_start: 0.15
  roi_x_end: 0.85
  roi_y_start: 0.15
  roi_y_end: 0.65
```

## Important Files

```
src/nomeer_robot_ros2/
├── src/
│   ├── autonomous_patrol/
│   │   ├── data/waypoints.yaml              # Recorded trajectories
│   │   ├── results/metrics.json             # Execution metrics
│   │   └── config/autonomous_patrol_config.yaml
│   ├── mono_depth_onnx/
│   │   ├── models/midas_v21_small.onnx      # AI model
│   │   └── config/mono_depth_config.yaml
│   └── robot_description/
├── README.md
└── src/nomeer_robot_ros2/README.md          # Original project README
```

## Available ROS 2 Topics

### Autonomy (Part A)
- `/cmd_vel` - Robot velocity commands
- `/odom` - Robot odometry
- `/waypoint_follower/current_waypoint` - Current waypoint index
- `/waypoint_follower/status` - Status string

### Vision/Depth (Part B)
- `/rgb_image` - Input RGB image (for external sources)
- `/camera/depth_estimated` - Depth map (mono16, 0-65535)
- `/camera/depth_colored` - Colored depth visualization
- `/depth_metric/min_frontal_depth` - Minimum frontal depth (0-1)
- `/depth_metric/mean_depth` - Mean depth (0-1)
- `/depth_metric/obstacle_detected` - Boolean: obstacle present

## Build Commands

Build all packages:
```bash
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
```

Build individual package:
```bash
colcon build --packages-select autonomous_patrol
colcon build --packages-select mono_depth_onnx
colcon build --packages-select robot_description
```

Clean build:
```bash
colcon clean all
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash
```

## Verification

```bash
bash verify_installation.sh
```

Expected output: 38/38 CHECKS PASSED

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Package not found" | `colcon build && source install/setup.bash` |
| "numpy: _ARRAY_API error" | `pip3 install 'numpy<2'` |
| "ONNX model not found" | `cd src/nomeer_robot_ros2/src/mono_depth_onnx && python3 scripts/download_midas_model.py` |
| "Gazebo won't open" | Ensure `rosdep install --from-paths src --ignore-src -r -y` completed successfully |
| "Can't see topics" | Ensure `source install/setup.bash` in all terminals |
| "Permission denied" | `chmod +x src/nomeer_robot_ros2/src/*/scripts/*.py` |
| "Gazebo/ROS version mismatch" | Use Gazebo Harmonic with `ros_gz_sim` and `ros_gz_bridge`, NOT `ros-humble-gazebo-ros` |

## Project Structure

```
ros2_ws/
├── README.md              # This file
├── SETUP_GUIDE.md         # Detailed setup documentation
├── QUICK_START.md         # Quick reference in Spanish
├── src/
│   └── nomeer_robot_ros2/
│       ├── README.md      # Original project readme
│       └── src/
│           ├── autonomous_patrol/    # Part A: Waypoint navigation
│           ├── mono_depth_onnx/      # Part B: Depth vision
│           └── robot_description/    # Robot model
├── build/        # Build artifacts
├── install/      # Installed packages
└── log/          # Build logs
```

## System Requirements

**Pre-installation:**
- Ubuntu 22.04 LTS or similar
- sudo access
- Internet connection
- Locale: en_US.UTF-8 (required for ROS)

**Post-installation:**
- ROS 2 Humble
- Python 3.10+
- ONNX Runtime (installed via pip)
- OpenCV Python

Optional:
- NVIDIA CUDA for GPU acceleration

## Tested Configurations

### Virtual Machine (VirtualBox)

This project has been successfully tested on:

**VirtualBox 7.0.26 with Ubuntu 22.04.5 amd64**
- Download: [ubuntu-22.04.5-desktop-amd64.iso](https://releases.ubuntu.com/jammy/ubuntu-22.04.5-desktop-amd64.iso)
- ROS 2 Humble
- **Recommended VM Specs:**
  - RAM: 10 GB
  - Video Memory (VRAM): 128 MB
  - CPUs: 6 cores (or as many as your host allows)
  - Disk: 100 GB

**Important GPU Note:** When using VirtualBox:
- **Disable 3D hardware acceleration** if using AMD GPUs - Gazebo will display a blank window with acceleration enabled on AMD systems
- NVIDIA GPUs may work with acceleration, but it's safer to disable it if you encounter display issues

### Native Linux

The system works well on native Ubuntu 22.04 installations without the VM constraints.

## Documentation

For more detailed information, see:
- `src/nomeer_robot_ros2/README.md` - Original project documentation

## License

See original project documentation.

---

**Status**: Development - Autonomous Waypoint Navigation and Depth-based Obstacle Detection  
**Date**: February 16, 2026  
**ROS Distribution**: Humble (with Gazebo Harmonic)