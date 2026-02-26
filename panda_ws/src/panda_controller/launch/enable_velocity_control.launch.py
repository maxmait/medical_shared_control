from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Include controller spawning (with delay)
    controllers_launch = TimerAction(
        period=1.0,  # Wait for Gazebo to fully load
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('panda_controller'),
                        'launch',
                        'spawn_controllers.launch.py'
                    ])
                ]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )
    
    # Switch to velocity control (with more delay)
    velocity_switch = TimerAction(
        period=12.0,  # Wait for controllers to load
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/controller_manager/switch_controller',
                    'controller_manager_msgs/srv/SwitchController',
                    "{activate_controllers: ['arm_velocity_controller'], deactivate_controllers: ['arm_controller'], strictness: 2}"
                ],
                output='screen',
                name='switch_to_velocity_control'
            )
        ]
    )

    # Send initial joint pose before switching to velocity control
    initial_pose = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/arm_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory',
                    "joint_names: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']\n"
                    "points:\n"
                    "- positions: [0.049, -0.13, -0.19, -1.69, -0.021, 1.55, 0.29]\n"
                    "  time_from_start: {sec: 2}"
                ],
                output='screen',
                name='set_initial_joint_pose'
            )
        ]
    )
    
    # Start velocity controller node (with even more delay)
    velocity_controller = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='panda_controller',
                executable='panda_velocity_controller',
                name='panda_velocity_controller',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        controllers_launch,
        initial_pose,
        velocity_switch,
        velocity_controller
    ])