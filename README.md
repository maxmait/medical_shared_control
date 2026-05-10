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


## More Commands and Debugging

Extended command lists, troubleshooting notes, and teleop details are in
[docs/commands.md](docs/commands.md).

---
## Project Status
- ✅ Panda robot simulation in Gazebo
- ✅ ROS2 control integration
- ✅ Cartesian velocity control with KDL
- ✅ Real-time inverse kinematics
- ✅ Safety constraints
- Shared autonomy (in development)