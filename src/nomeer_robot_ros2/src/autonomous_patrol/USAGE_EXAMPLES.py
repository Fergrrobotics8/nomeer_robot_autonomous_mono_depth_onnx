#!/usr/bin/env python3
"""
Example usage scenarios for autonomous_patrol package
"""

print("""
╔════════════════════════════════════════════════════════════════╗
║         AUTONOMOUS PATROL - Usage Examples                    ║
║              Part A: Waypoint Navigation                       ║
╚════════════════════════════════════════════════════════════════╝

SCENARIO 1: Quick Test with Example Waypoints
─────────────────────────────────────────────────

1. Generate example waypoints:
   $ cd ~/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol
   $ python3 generate_example_waypoints.py

2. Update configuration to use example waypoints:
   # Edit config/autonomous_patrol_config.yaml
   follower:
     waypoints_file: 'example_waypoints.yaml'  ← Change this

3. Start simulator:
   $ ros2 launch robot_description gazebo.launch.py

4. Follow waypoints:
   $ ros2 launch autonomous_patrol follow_waypoints.launch.py

5. View results:
   $ cat autonomous_patrol/results/metrics.json | python3 -m json.tool

═══════════════════════════════════════════════════════════════════


SCENARIO 2: Full Workflow - Record and Replay
──────────────────────────────────────────────

Terminal 1: Simulator
$ ros2 launch robot_description gazebo.launch.py

Terminal 2: Record Phase
$ ros2 launch autonomous_patrol record_waypoints.launch.py

Terminal 3: Teleoperation
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
    --ros-args -r /cmd_vel:=/cmd_vel

  [Move robot around the space]
  [In Terminal 2, press Ctrl+C to save]

Terminal 2: Playback Phase  
$ ros2 launch autonomous_patrol follow_waypoints.launch.py

  [Watch robot replay the path in RViz]
  [After completion, check metrics.json]

═══════════════════════════════════════════════════════════════════


SCENARIO 3: Visualization Only
───────────────────────────────

View recorded waypoints without execution:

Terminal 1: Simulator
$ ros2 launch robot_description gazebo.launch.py

Terminal 2: Visualization
$ ros2 launch autonomous_patrol visualize_waypoints.launch.py

  [RViz opens showing the waypoint path]
  [No robot movement]

═══════════════════════════════════════════════════════════════════


SCENARIO 4: Tuning Parameters for Better Precision
──────────────────────────────────────────────────

Edit config/autonomous_patrol_config.yaml for:

High Precision:
  recorder:
    min_distance_between_waypoints: 0.05
  follower:
    waypoint_tolerance: 0.1
    max_linear_velocity: 0.3
    control_frequency: 20.0

Default (Balanced):
  recorder:
    min_distance_between_waypoints: 0.1
  follower:
    waypoint_tolerance: 0.2
    max_linear_velocity: 0.5
    control_frequency: 10.0

High Speed:
  recorder:
    min_distance_between_waypoints: 0.2
  follower:
    waypoint_tolerance: 0.3
    max_linear_velocity: 0.8
    control_frequency: 20.0

═══════════════════════════════════════════════════════════════════


SCENARIO 5: Runtime Parameter Changes
──────────────────────────────────────

Change parameters while nodes are running:

$ ros2 param list /follow_waypoints
$ ros2 param get /follow_waypoints follower.waypoint_tolerance
$ ros2 param set /follow_waypoints follower.waypoint_tolerance 0.15

═══════════════════════════════════════════════════════════════════


SCENARIO 6: Monitoring Execution
─────────────────────────────────

Watch status in real-time:

Terminal 1:
$ ros2 topic echo /waypoint_recorder/status

Terminal 2:
$ ros2 topic echo /waypoint_follower/status

Terminal 3:
$ ros2 topic echo /waypoint_follower/current_waypoint

═══════════════════════════════════════════════════════════════════


SCENARIO 7: File Management
────────────────────────────

View recorded data:
$ cat autonomous_patrol/data/waypoints.yaml

View metrics:
$ python3 -m json.tool autonomous_patrol/results/metrics.json

List all waypoint files:
$ ls -lah autonomous_patrol/data/*.yaml

Copy waypoints to backup:
$ cp autonomous_patrol/data/waypoints.yaml \\
   autonomous_patrol/data/waypoints_backup_$(date +%s).yaml

═══════════════════════════════════════════════════════════════════


SCENARIO 8: Debugging Common Issues
────────────────────────────────────

Issue: Robot not moving after launch
Fix:
1. Check /odom is publishing:
   $ ros2 topic hz /odom

2. Check /cmd_vel is being sent:
   $ ros2 topic echo /cmd_vel

3. Verify Gazebo bridge is active:
   $ ros2 topic list | grep -i cmd

Issue: Large errors in metrics
Fix:
1. Reduce speed: waypoint_tolerance: 0.3
2. Increase control frequency: control_frequency: 20.0
3. Check odometry accuracy

Issue: Waypoints not saving
Fix:
1. Verify directory exists:
   $ mkdir -p autonomous_patrol/data
   $ chmod 755 autonomous_patrol/data

2. Check file permissions
3. Look at recorder logs for errors

═══════════════════════════════════════════════════════════════════


SCENARIO 9: Integration with Other ROS 2 Tools
───────────────────────────────────────────────

Using with Nav2:
- Can provide initial waypoints for Nav2
- Complement with obstacle avoidance

Using with TF2:
- Works with transform frames
- Assumed frame: 'odom'

Using with ROS 2 Bags:
- Record waypoints with rosbag2
- Replay for analysis

═══════════════════════════════════════════════════════════════════


SCENARIO 10: Custom Configuration Files
────────────────────────────────────────

Create custom configuration:

$ cp config/autonomous_patrol_config.yaml config/my_config.yaml
$ # Edit my_config.yaml for your robot

Launch with custom config:

$ ros2 run autonomous_patrol record_waypoints_node.py \\
    --ros-args -p config_file:=config/my_config.yaml

Or modify launch file:

# launch/custom.launch.py
def generate_launch_description():
    config_file = os.path.join(..., 'config', 'my_config.yaml')
    ...

═══════════════════════════════════════════════════════════════════


USEFUL COMMANDS REFERENCE
─────────────────────────

# Build
$ cd ~/ros2_ws && colcon build --packages-select autonomous_patrol

# Source (always after build)
$ source install/setup.bash

# List nodes
$ ros2 node list

# List topics
$ ros2 topic list

# Get node info
$ ros2 node info /record_waypoints

# Get topic info
$ ros2 topic info /waypoint_follower/status

# Check messages
$ ros2 interface show geometry_msgs/msg/Twist

# Record bag
$ ros2 bag record -o waypoint_execution /odom /cmd_vel

# Playback bag
$ ros2 bag play waypoint_execution_0/

═══════════════════════════════════════════════════════════════════


TROUBLESHOOTING QUICK REFERENCE
────────────────────────────────

Error                          Solution
────────────────────────────────────────────────────────────────
Build fails                    └─ Check Python syntax, ROS 2 installed
Node won't run                 └─ Source setup.bash, check PATH
Odom not publishing            └─ Gazebo running? Check simulator
No visualization               └─ RViz frame="odom", topics active
Metrics not generated          └─ Check results/ directory exists
Config not loaded              └─ Verify path, YAML syntax
Robot won't stop               └─ Check /cmd_vel listener

═══════════════════════════════════════════════════════════════════

For more information, see:
- README.md: Full documentation
- QUICK_START.md: Get started guide
- TECHNICAL_SPECS.md: Technical details
- RESUMEN_ES.md: Resumen en español

═══════════════════════════════════════════════════════════════════
""")
