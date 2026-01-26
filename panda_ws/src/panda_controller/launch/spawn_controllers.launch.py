import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Add some delay to ensure Gazebo is fully loaded
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Optional: spawn velocity controller (but don't activate it)
    velocity_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_velocity_controller", 
                    "--controller-manager", "/controller_manager",
                    "--inactive"  # Load but don't activate
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        velocity_controller_spawner,
    ])