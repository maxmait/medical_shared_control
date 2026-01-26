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
        velocity_switch,
        velocity_controller
    ])