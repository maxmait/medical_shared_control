# Medical Shared Control for Surgical Robotics

This project implements a safety-critical shared-control module in modern C++ 
for simulated surgical robotics, using a Franka Emika Panda robot in ROS2/Gazebo.

Structure:
- core/: standalone C++ safety & control library (ROS-independent)
- panda_ws/: ROS2 workspace for simulation and integration

Goal:
Design and evaluate distance-based virtual safety constraints and shared control
for precise human-guided manipulation near delicate tissue.

LiDAR distance estimation:
A single-beam LiDAR mounted at the tool tip measures the line-of-sight distance
to tissue surfaces. The range reading is used as a real-time proxy for tool-to-
tissue separation and feeds the safety constraint logic that limits motion as
the tool approaches the tissue.

## Quick Start Commands

### Demo Launch (Gazebo + RViz camera + teleop)

```bash
# Terminal 1: Gazebo world with the human model
ros2 launch panda_description gazebo.launch.py world_file:=medical_procedure.sdf

# Terminal 2: Enable velocity control
ros2 launch panda_controller enable_velocity_control.launch.py

# Terminal 3: RViz camera feed
ros2 launch panda_description rviz_camera.launch.py

# Terminal 4: Teleop with haptic controller
ros2 launch panda_controller teleop.launch.py
```

This launches the simulation, the camera feed, and teleop so you can control
robot velocity with a haptic controller.

![Robot control demo](readme_assets/robot_demo_2.gif)

### Method 1: Step-by-step Launch (Recommended for Development)

```bash
# Terminal 1: Build and setup
cd panda_ws
colcon build
source install/setup.bash

# Terminal 2: Launch Gazebo simulation
ros2 launch panda_description gazebo.launch.py

# Terminal 3: Wait for Gazebo to fully load, then spawn controllers
ros2 launch panda_controller spawn_controllers.launch.py

# Terminal 4: Enable velocity control
ros2 launch panda_controller enable_velocity_control.launch.py

# Terminal 5: Test velocity control
ros2 run panda_controller velocity_test.py
```

### Method 2: Manual Controller Switching (For Debugging)

```bash
# After running gazebo and spawn_controllers:

# Check current controller status
ros2 control list_controllers

# Check hardware interfaces (should show position interfaces as claimed)
ros2 control list_hardware_interfaces

# Manual switch to velocity control
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['arm_velocity_controller'], deactivate_controllers: ['arm_controller'], strictness: 2}"

# Verify switch worked (should show velocity interfaces as claimed)
ros2 control list_hardware_interfaces

# Start velocity controller node
ros2 run panda_controller panda_velocity_controller

# Test with manual commands
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --rate 10

# Stop robot
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once
```

### Debugging Commands

```bash
# Check if controllers are loaded properly
ros2 control list_controllers

# Check hardware interface status
ros2 control list_hardware_interfaces

# Monitor joint velocities being sent to robot
ros2 topic echo /arm_velocity_controller/commands

# Monitor joint states
ros2 topic echo /joint_states

# Check velocity commands being received
ros2 topic echo /panda_velocity_cmd

# List all available controllers
ros2 control list_controller_types

# Check controller manager status
ros2 service list | grep controller_manager
```

### Troubleshooting

**If controller switch fails (`ok=False`):**
1. Check controllers are loaded: `ros2 control list_controllers`
2. Make sure `arm_velocity_controller` shows as `inactive`
3. Make sure `arm_controller` shows as `active`
4. Try manual step-by-step switching

**If robot doesn't move despite non-zero joint velocities:**
1. Check hardware interfaces are claimed: `ros2 control list_hardware_interfaces`
2. Verify velocity interfaces show `[claimed]`
3. Check velocity commands: `ros2 topic echo /arm_velocity_controller/commands`

**If Gazebo crashes or hangs:**
1. Kill all processes: `pkill -f gazebo && pkill -f ros2`
2. Restart step by step
3. Wait longer between launch steps

### Development Workflow

```bash
# Full restart after code changes:
pkill -f gazebo && pkill -f ros2  # Kill all processes
cd panda_ws
colcon build                      # Rebuild
source install/setup.bash        # Source new build

# Then follow Method 1 above
```

### Quick Test Commands

```bash
# Send different velocity commands for testing:

# Move forward in X
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}}' --rate 10

# Move up in Z  
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.1}}' --rate 10

# Rotate around Z axis
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{angular: {x: 0.0, y: 0.0, z: 0.2}}' --rate 10

# Stop all motion
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{}' --once

# Run comprehensive test sequence
ros2 run panda_controller velocity_test.py
```

---


Test vibration: sudo fftest /dev/input/event20
## Project Status
- ✅ Panda robot simulation in Gazebo
- ✅ ROS2 control integration
- ✅ Cartesian velocity control with KDL
- ✅ Real-time inverse kinematics
- Safety constraints (in development)
- Shared autonomy (in development)