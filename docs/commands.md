# Commands and Debugging

This file collects extended launch sequences, debugging helpers, and safety notes
so the README stays focused on the core overview.

## Manual Controller Switching (Debugging)

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

## Debugging Commands

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

## Troubleshooting

**If controller switch fails (ok=False):**
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

## Development Workflow

```bash
# Full restart after code changes:
pkill -f gazebo && pkill -f ros2  # Kill all processes
cd panda_ws
colcon build                      # Rebuild
source install/setup.bash        # Source new build

# Then follow Method 1 in the README
```

## Quick Test Commands

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

## Safety Control Notes

- RT core runs at 1 kHz in a dedicated C++ thread.
- The RT loop is allocation-free and only accesses shared state.
- ROS nodes never block the RT loop; they only update shared inputs.
- Safety is enabled by default in teleop.
- Disable safety for direct control:

```bash
ros2 launch panda_controller teleop.launch.py disable_safety:=true
```

## Teleop Controller Notes

- B button sends a safe joint pose by switching to the position controller,
  publishing a single `JointTrajectory`, then switching back to velocity control.
- X button toggles between position and velocity controllers.
