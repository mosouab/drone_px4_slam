#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch Arguments (override from command line) ---
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port connecting Pi 4 to Pixhawk (UART: /dev/ttyAMA0, USB: /dev/ttyUSB0)'
    )
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='921600',
        description='Baud rate for serial connection (921600 for UART, 57600 for USB)'
    )
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic from the downward-facing camera driver'
    )
    tgt_altitude_arg = DeclareLaunchArgument(
        'target_altitude',
        default_value='2.0',
        description='Hover/search altitude in metres before descending onto marker'
    )

    # --- MAVROS Node ---
    # Bridges ROS 2 <-> PX4 over the serial link to the Pixhawk.
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': [LaunchConfiguration('serial_port'), '@', LaunchConfiguration('serial_baud')],
            'gcs_url': '',           # No ground station forwarding by default
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',  # MAVLink 2
        }],
    )

    # --- Vision Node ---
    # Detects ArUco marker in downward camera feed and publishes normalised
    # pixel error on /landing/target.
    vision_node = Node(
        package='drone_control',
        executable='vision',
        name='vision_aruco_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
        }],
    )

    # --- Control / Landing Node ---
    # Reads /landing/target and /mavros topics; commands velocity setpoints
    # via MAVROS to align over the marker and land.
    # Delayed by 3 s to give MAVROS time to connect to the FCU first.
    control_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drone_control',
                executable='control',
                name='control_land_node',
                output='screen',
                parameters=[{
                    'target_altitude': LaunchConfiguration('target_altitude'),
                }],
            )
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        camera_topic_arg,
        tgt_altitude_arg,
        mavros_node,
        vision_node,
        control_node,
    ])
