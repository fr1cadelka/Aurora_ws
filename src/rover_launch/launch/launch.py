import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pc_ip_arg = DeclareLaunchArgument(
        'pc_ip',
        default_value='127.0.0.1',
        description='IP address of the PC running the UDP client'
    )

    # УЗЕЛ 1: video_camera_pkg
    video_camera_node = ExecuteProcess(
        cmd=['ros2', 'run', 'video_camera_pkg', 'video_camera_pkg_node'],
        output='screen'
    )

    # УЗЕЛ 2: udp_pkg_server_node
    udp_server_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'udp_pkg', 'udp_pkg_server_node',
            '--ros-args', '-p', 'pc_ip:=127.0.0.1'
        ],
        output='screen'
    )

    # УЗЕЛ 3: TCP_pkg - ТОЧНОЕ ИМЯ!
    tcp_client_node = ExecuteProcess(
        cmd=['ros2', 'run', 'TCP_pkg', 'tcp_client_node'],
        output='screen'
    )

    return LaunchDescription([
        pc_ip_arg,
        video_camera_node,
        udp_server_node,
        tcp_client_node
    ])