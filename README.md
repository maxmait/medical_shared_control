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

### Demo Launch (one command)

```bash
ros2 launch panda_controller bringup.launch.py
```

That single command starts everything the demo needs — Gazebo with the medical
world, the RViz camera feed, the controllers and velocity control, the teleop +
safety bridge + haptics, and the live status window. It replaces the old
four-terminal sequence.

Useful arguments:

```bash
# A different world, no RViz:
ros2 launch panda_controller bringup.launch.py world_file:=procedure.sdf start_rviz:=false

# Bypass the safety layer (direct teleop):
ros2 launch panda_controller bringup.launch.py disable_safety:=true

# Hide the status window / haptics:
ros2 launch panda_controller bringup.launch.py enable_dashboard:=false enable_haptic:=false
```

Safety control is enabled by default. Teleop publishes to a user command topic,
and the safety bridge generates the final velocity command.

### Configuration

All node parameters — the safety-law thresholds, IK limits, and teleop scales —
live in one file: [panda_ws/src/panda_controller/config/sim.yaml](panda_ws/src/panda_controller/config/sim.yaml).
Edit values there (no rebuild needed for a plain launch) or point the launch at
another file with `params_file:=/path/to/your.yaml`.

### Status window

The bringup starts a small live dashboard (`status_dashboard`) showing the
joystick connection, active coordinate frame (global/local) and mode, which
buttons/sticks/triggers are active (with each stick's axis meaning), the LiDAR
tool-to-tissue distance, and — most usefully — the **user-commanded velocity
next to the safety-scaled velocity**, so you can watch the safety layer slow the
tool as it nears tissue. It also has a control guide and a **Reconnect joystick**
button. It is display-only for the robot; run it alone with
`ros2 run panda_controller status_dashboard.py`.

### Joystick reconnect

A `joy_manager` node supervises the joystick driver: it owns `joy_node` and
restarts it fresh — which re-grabs a replugged controller — either automatically
after the `/joy` stream goes silent, or on demand via the **Reconnect joystick**
button (the `/joystick/reconnect` service). This fixes the stock `joy_node`
holding a dead handle after the controller is unplugged and plugged back in.

The individual launch files (`gazebo.launch.py`, `enable_velocity_control.launch.py`,
`rviz_camera.launch.py`, `teleop.launch.py`) still work standalone — see
[docs/commands.md](docs/commands.md).

![Robot control demo](readme_assets/robot_demo_2.gif)

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
- A dead-man watchdog zeroes the output if the LiDAR or user-command stream
  goes stale (default 0.2 s), so a dropped input never replays the last motion.
- All safety thresholds are ROS parameters (`safety.stop_distance`,
  `safety.slow_distance`, `safety.max_linear_vel`, `safety.input_timeout_s`,
  ...) on the `shared_control_bridge`, and the output is saturated to absolute
  velocity limits.
- The pure safety law is unit-tested with gtest (`colcon test
  --packages-select safety_controller`).

Inverse kinematics:
- `panda_velocity_controller` resolves the full 7-DoF arm from a Cartesian
  twist using a damped-least-squares KDL solver (`ChainIkSolverVel_wdls`), so
  joint velocities stay bounded near singularities. Output joint velocities are
  additionally clamped to `max_joint_vel`.
- Teleop is 6-DoF: hold the orientation button to command roll/pitch/yaw with
  the sticks instead of translation.

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


## More Commands and Debugging

Extended command lists, troubleshooting notes, and teleop details are in
[docs/commands.md](docs/commands.md).

---
## Testing

The real-time safety core has a gtest unit suite (velocity scaling, stop
hysteresis, direction-aware slowdown, output clamping, watchdog zeroing, and
the lock-free buffer). GitHub Actions builds the workspace and runs the tests
on every push (see `.github/workflows/ci.yml`).

```bash
cd panda_ws
colcon build --packages-select safety_controller panda_description panda_controller
colcon test --packages-select safety_controller && colcon test-result --verbose
```

## Project Status
- Panda robot simulation in Gazebo (done)
- ROS2 control integration (done)
- Cartesian velocity control with KDL (done)
- Full 7-DoF real-time inverse kinematics (damped least-squares) (done)
- 6-DoF teleoperation (translation + orientation) (done)
- Distance-based safety constraints with dead-man watchdog (done)
- Controller reconnect with bounded retry (done)
- Unit tests + CI for the safety core (done)
- Eyeball curvature-following (future work — see `to_do.txt`)