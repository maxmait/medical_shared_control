"""Single-command bringup for the shared-control simulation.

Launches, in order, everything the demo needs:
  1. Gazebo + robot + sensor bridges (panda_description/gazebo.launch.py)
  2. RViz camera feed              (panda_description/rviz_camera.launch.py)
  3. Controllers + velocity control (panda_controller/enable_velocity_control.launch.py)
  4. Teleop + safety bridge + haptics (panda_controller/teleop.launch.py)

All node parameters come from a single YAML file (config/sim.yaml by default).

Example:
  ros2 launch panda_controller bringup.launch.py
  ros2 launch panda_controller bringup.launch.py world_file:=procedure.sdf start_rviz:=false
  ros2 launch panda_controller bringup.launch.py disable_safety:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    panda_controller = FindPackageShare('panda_controller')
    panda_description = FindPackageShare('panda_description')

    # --- Launch arguments ---
    world_file = LaunchConfiguration('world_file')
    start_rviz = LaunchConfiguration('start_rviz')
    enable_haptic = LaunchConfiguration('enable_haptic')
    disable_safety = LaunchConfiguration('disable_safety')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    startup_delay = LaunchConfiguration('startup_delay')
    enable_dashboard = LaunchConfiguration('enable_dashboard')

    args = [
        DeclareLaunchArgument(
            'world_file', default_value='procedure.sdf',
            description='World file under panda_description/worlds'),
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='Start the RViz camera view'),
        DeclareLaunchArgument(
            'enable_haptic', default_value='true',
            description='Start the haptic feedback node'),
        DeclareLaunchArgument(
            'disable_safety', default_value='false',
            description='Bypass the safety bridge (direct teleop)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([panda_controller, 'config', 'sim.yaml']),
            description='YAML file with node parameters (safety law, IK, teleop)'),
        DeclareLaunchArgument(
            'startup_delay', default_value='6.0',
            description='Seconds to let Gazebo initialise before starting the '
                        'controllers and teleop stack'),
        DeclareLaunchArgument(
            'enable_dashboard', default_value='true',
            description='Start the live status/telemetry window'),
    ]

    def include(pkg, launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', launch_file])),
            launch_arguments=(launch_arguments or {}).items(),
        )

    # 1. Gazebo world, robot, and sensor bridges (starts immediately).
    gazebo = include(panda_description, 'gazebo.launch.py', {'world_file': world_file})

    # 2. RViz camera feed (independent of the control stack).
    rviz = include(panda_description, 'rviz_camera.launch.py',
                   {'start_rviz': start_rviz, 'use_sim_time': use_sim_time})

    # Live status/telemetry window (display-only).
    dashboard = Node(
        package='panda_controller',
        executable='status_dashboard.py',
        name='status_dashboard',
        output='screen',
        condition=IfCondition(enable_dashboard),
    )

    # 3 + 4. Controllers/velocity control and teleop, delayed so Gazebo's
    # controller_manager is up first (matches the proven manual launch order).
    control_stack = TimerAction(
        period=startup_delay,
        actions=[
            include(panda_controller, 'enable_velocity_control.launch.py',
                    {'use_sim_time': use_sim_time, 'params_file': params_file}),
            include(panda_controller, 'teleop.launch.py',
                    {'enable_haptic': enable_haptic,
                     'disable_safety': disable_safety,
                     'params_file': params_file}),
        ],
    )

    return LaunchDescription(args + [gazebo, rviz, dashboard, control_stack])
