from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def find_haptic_controller():
    """Find controller that supports haptic feedback by checking /proc/bus/input/devices"""
    try:
        with open('/proc/bus/input/devices', 'r') as f:
            content = f.read()
        
        devices = content.split('\n\n')
        
        for device_block in devices:
            if not device_block.strip():
                continue
                
            lines = device_block.strip().split('\n')
            device_info = {}
            
            for line in lines:
                if line.startswith('N: '):
                    device_info['name'] = line.split('Name="')[1].rstrip('"') if 'Name="' in line else ''
                elif line.startswith('H: '):
                    handlers = line.split('Handlers=')[1] if 'Handlers=' in line else ''
                    device_info['handlers'] = handlers
                elif line.startswith('B: FF='):
                    ff_capabilities = line.split('B: FF=')[1] if 'B: FF=' in line else ''
                    device_info['ff'] = ff_capabilities
            
            if ('ff' in device_info and
                device_info['ff'] != '0' and
                'handlers' in device_info and
                'event' in device_info['handlers']):
                
                handlers = device_info['handlers'].split()
                for handler in handlers:
                    if handler.startswith('event'):
                        device_path = f'/dev/input/{handler}'
                        if os.path.exists(device_path):
                            print(f"Auto-detected haptic controller: {device_info.get('name', 'Unknown')} at {device_path}")
                            return device_path
    except Exception as e:
        print(f"Haptic controller auto-detection failed: {e}")
    
    return '/dev/input/event20'

def find_joystick():
    """Find joystick device by checking /proc/bus/input/devices"""
    try:
        with open('/proc/bus/input/devices', 'r') as f:
            content = f.read()
        
        devices = content.split('\n\n')
        
        for device_block in devices:
            if not device_block.strip():
                continue
            
            lines = device_block.strip().split('\n')
            device_info = {}
            
            for line in lines:
                if line.startswith('N: '):
                    device_info['name'] = line.split('Name="')[1].rstrip('"') if 'Name="' in line else ''
                elif line.startswith('H: '):
                    handlers = line.split('Handlers=')[1] if 'Handlers=' in line else ''
                    device_info['handlers'] = handlers
            
            # Look for js device (joystick)
            if ('handlers' in device_info and 'js' in device_info['handlers']):
                handlers = device_info['handlers'].split()
                for handler in handlers:
                    if handler.startswith('js'):
                        device_path = f'/dev/input/{handler}'
                        if os.path.exists(device_path):
                            print(f"Auto-detected joystick: {device_info.get('name', 'Unknown')} at {device_path}")
                            return device_path
    except Exception as e:
        print(f"Joystick auto-detection failed: {e}")
    
    return '/dev/input/js0'

def generate_launch_description():

    # --- Launch Arguments ---
    enable_haptic_arg = DeclareLaunchArgument(
        'enable_haptic',
        default_value='true',
        description='Enable haptic feedback node'
    )

    max_vibration_intensity_arg = DeclareLaunchArgument(
        'max_vibration_intensity',
        default_value='25000',
        description='Max vibration intensity (0-65535)'
    )

    enable_collision_feedback_arg = DeclareLaunchArgument(
        'enable_collision_feedback',
        default_value='true',
        description='Enable vibration on collision detection'
    )

    enable_force_feedback_arg = DeclareLaunchArgument(
        'enable_force_feedback',
        default_value='false',
        description='Enable vibration based on force sensor data'
    )

    enable_status_feedback_arg = DeclareLaunchArgument(
        'enable_status_feedback',
        default_value='true',
        description='Enable vibration on status changes (e.g. mode switch)'
    )

    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale',
        default_value='0.15',
        description='Linear velocity scale for teleoperation'
    )

    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale',
        default_value='0.3',
        description='Angular velocity scale for teleoperation'
    )

    disable_safety_arg = DeclareLaunchArgument(
        'disable_safety',
        default_value='false',
        description='Disable safety bridge and remapping'
    )

    # --- Auto-detect devices ---
    joystick_device = find_joystick()
    haptic_device = find_haptic_controller()

    # --- Nodes ---

    # Joy node - reads raw joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {'device': joystick_device}
        ],
        output='screen'
    )

    # Teleoperation input node - converts joystick to velocity commands
    tele_controller_safety = Node(
        package='panda_controller',
        executable='tele_controller_input',
        name='tele_controller_input',
        parameters=[
            {'linear_scale': LaunchConfiguration('linear_scale')},
            {'angular_scale': LaunchConfiguration('angular_scale')}
        ],
        remappings=[
            ('/panda_velocity_cmd', '/panda_velocity_cmd_user')
        ],
        condition=UnlessCondition(LaunchConfiguration('disable_safety')),
        output='screen'
    )

    tele_controller_direct = Node(
        package='panda_controller',
        executable='tele_controller_input',
        name='tele_controller_input',
        parameters=[
            {'linear_scale': LaunchConfiguration('linear_scale')},
            {'angular_scale': LaunchConfiguration('angular_scale')}
        ],
        condition=IfCondition(LaunchConfiguration('disable_safety')),
        output='screen'
    )

    shared_control_bridge = Node(
        package='panda_controller',
        executable='shared_control_bridge',
        name='shared_control_bridge',
        condition=UnlessCondition(LaunchConfiguration('disable_safety')),
        output='screen'
    )

    # Haptic feedback node - optional, controlled by enable_haptic argument
    haptic_feedback = Node(
        package='panda_controller',
        executable='haptic_feedback_node.py',
        name='haptic_feedback',
        parameters=[
            {'controller_device': haptic_device},
            {'max_vibration_intensity': LaunchConfiguration('max_vibration_intensity')},
            {'enable_collision_feedback': LaunchConfiguration('enable_collision_feedback')},
            {'enable_force_feedback': LaunchConfiguration('enable_force_feedback')},
            {'enable_status_feedback': LaunchConfiguration('enable_status_feedback')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_haptic')),
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        enable_haptic_arg,
        max_vibration_intensity_arg,
        enable_collision_feedback_arg,
        enable_force_feedback_arg,
        enable_status_feedback_arg,
        linear_scale_arg,
        angular_scale_arg,
        disable_safety_arg,
        # Nodes
        joy_node,
        tele_controller_safety,
        tele_controller_direct,
        shared_control_bridge,
        haptic_feedback
    ])
#Argument example:
#ros2 launch panda_controller teleop.launch.py enable_haptic:=true enable_force_feedback:=true disable_safety:=true linear_scale:=0.1