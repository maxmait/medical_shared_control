# Medical Shared Control for Surgical Robotics

This project implements a safety-critical shared-control module in modern C++ 
for simulated surgical robotics, using a Franka Emika Panda robot in ROS2/Gazebo.

Structure:
- panda_ws/src/safety_controller/: ROS-independent C++ RT safety core
- panda_ws/src/panda_controller/: ROS2 bridge + teleop + controllers
- panda_ws/: ROS2 workspace for simulation and integration

Goal:
Design and evaluate distance-based virtual safety constraints and shared control
for precise human-guided manipulation near delicate tissue.

LiDAR distance estimation:
A single-beam LiDAR mounted at the tool tip measures the line-of-sight distance
to tissue surfaces. The range reading is used as a real-time proxy for tool-to-
tissue separation and feeds the safety constraint logic that limits motion as
the tool approaches the tissue.

Real-time safety control (RT core):
- The safety controller runs in a dedicated C++ thread at 1 kHz.
- The RT loop avoids dynamic memory allocation and never waits on ROS.
- ROS nodes only exchange data via a lock-free latest-value buffer.
- The RT core outputs a safe velocity command based on LiDAR distance.

## Architecture
![Simplified Data Flow Chart](readme_assets/medical_RT_communication.png)

Safety bridge topic mapping:
- Inputs: `/probe/scan`, `/panda_velocity_cmd_user`, `/joint_states`
- Output: `/panda_velocity_cmd`, `/haptic_feedback` 

Safety scaling behavior:
- The RT core scales motion toward the LiDAR beam direction more aggressively.
- Lateral motion is mildly slowed near obstacles for stability.
- Motion away from the obstacle is allowed with minimal slowdown.

Direction-aware slowdown:
The LiDAR beam defines a unit direction in the base frame. The controller
projects the commanded linear velocity onto this direction. Only the component
that moves toward the obstacle (positive projection) receives the strong
distance-based slowdown. Components that move away or laterally are kept at
full or mildly reduced speed. This prevents unnecessary slowdown when the
tool is retreating or sliding parallel to tissue.

## Docker Usage

Requirements:
- Docker Engine
- Docker Compose (v2)
- X11 display (for Gazebo/RViz)

Build the image:

```bash
docker build -t medical_shared .
```

Run with volumes via compose:

```bash
xhost +local:docker
docker compose up
```

Open a new terminal in the running container:

```bash
docker exec -it medical_robot_ros bash
```

Remember to source in each container terminal:

```bash
cd /workspaces/medical_robot_shared_control/panda_ws
source install/setup.bash
```

## Quick Start Commands

### Demo Launch (Gazebo + RViz camera + teleop)

```bash
# Terminal 1: Gazebo world with the human model
ros2 launch panda_description gazebo.launch.py world_file:=medical_procedure.sdf

# Terminal 2: Enable velocity control
ros2 launch panda_controller enable_velocity_control.launch.py

# Terminal 3: RViz camera feed
ros2 launch panda_description rviz_camera.launch.py

# Terminal 4: Teleop with haptic controller + safety bridge
ros2 launch panda_controller teleop.launch.py
```

This launches the simulation, the camera feed, and teleop so you can control
robot velocity with a haptic controller.

Safety control is enabled by default. Teleop publishes to a user command topic,
and the safety bridge generates the final velocity command.

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

# Terminal 5: Start safety bridge (RT core)
ros2 run panda_controller shared_control_bridge

# Terminal 6: Teleop (safe by default)
ros2 launch panda_controller teleop.launch.py
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

# Start safety bridge (RT core)
ros2 run panda_controller shared_control_bridge

# Test with manual commands (bypass safety)
ros2 topic pub /panda_velocity_cmd geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --rate 10

# Test with manual commands (through safety)
ros2 topic pub /panda_velocity_cmd_user geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --rate 10

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

# Check user velocity commands (before safety)
ros2 topic echo /panda_velocity_cmd_user

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

**If LiDAR direction feels inverted:**
Use the `lidar_axis` parameter on the bridge to match the sensor axis:

```bash
ros2 run panda_controller shared_control_bridge --ros-args -p lidar_axis:=x
```

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
# Send different velocity commands for testing (through safety):

# Move forward in X
ros2 topic pub /panda_velocity_cmd_user geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}}' --rate 10

# Move up in Z  
ros2 topic pub /panda_velocity_cmd_user geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.1}}' --rate 10

# Rotate around Z axis
ros2 topic pub /panda_velocity_cmd_user geometry_msgs/msg/Twist '{angular: {x: 0.0, y: 0.0, z: 0.2}}' --rate 10

# Stop all motion
ros2 topic pub /panda_velocity_cmd_user geometry_msgs/msg/Twist '{}' --once

# Run comprehensive test sequence
ros2 run panda_controller velocity_test.py
```

### Safety Control Notes

- RT core runs at 1 kHz in a dedicated C++ thread.
- The RT loop is allocation-free and only accesses shared state.
- ROS nodes never block the RT loop; they only update shared inputs.
- Safety is enabled by default in teleop.
- Disable safety for direct control:

```bash
ros2 launch panda_controller teleop.launch.py disable_safety:=true
```

### Teleop Controller Notes

- B button sends a safe joint pose by switching to the position controller,
  publishing a single `JointTrajectory`, then switching back to velocity control.
- X button toggles between position and velocity controllers.
```

---
## Project Status
- ✅ Panda robot simulation in Gazebo
- ✅ ROS2 control integration
- ✅ Cartesian velocity control with KDL
- ✅ Real-time inverse kinematics
- ✅ Safety constraints (in development)
- Shared autonomy (in development)