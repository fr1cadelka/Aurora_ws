#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Устанавливаем ROS_DOMAIN_ID = 44
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='44'
    )

    # Параметры сети
    pc_ip = LaunchConfiguration('pc_ip', default='127.0.0.1')
    udp_port = LaunchConfiguration('udp_port', default='12346')
    tcp_port = LaunchConfiguration('tcp_port', default='6000')

    # Запуск dual_vesc.launch.py из vesc_ackermann
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_ackermann'),
                'launch',
                'dual_vesc.launch.py'
            )
        )
    )

    rover_nodes = [
        # Видеокамера
        Node(
            package='video_camera_pkg',
            executable='video_camera_pkg_node',
            name='video_camera_node',
            output='screen',
        ),

        # UDP-узел (закомментирован, пока не нужен)
        # Node(
        #     package='udp_pkg',
        #     executable='udp_pkg_server_node',
        #     name='udp_server_node',
        #     parameters=[{'server_ip': pc_ip, 'server_port': udp_port}],
        #     output='screen',
        # ),

        # Управление VESC (control_vesk) – вывод ВИДЕН в терминале
        Node(
            package='control_telega_pkg',
            executable='control_vesk_pkg_node',
            name='vesc_control_node',
            output='screen',                     # ← вывод в терминал
            arguments=['--ros-args', '--log-level', 'INFO'],  # уровень INFO (видно команды)
        ),

        # Одометрия – ПОЛНОСТЬЮ ПОДАВЛЕНА (никакого вывода)
        Node(
            package='odometry_pkg',
            executable='odometry_pkg_node',
            name='odometry_node',
            output='log',                        # ← перенаправляем в лог-файл, не в терминал
            arguments=['--ros-args', '--log-level', 'ERROR'],  # логируем только ошибки
        ),

        # TCP-клиент
        Node(
            package='tcp_pkg',
            executable='tcp_client_node',
            name='tcp_client_node',
            parameters=[{'server_ip': pc_ip, 'server_port': tcp_port}],
            output='screen',
        ),

        # Line follower
        Node(
            package='control_all_system_pkg',
            executable='line_follower_node',
            name='line_follower_node',
            # output не указываем – вывод по умолчанию в лог (не в терминал)
        ),
    ]

    return LaunchDescription([
        set_domain_id,
        vesc_launch,
        *rover_nodes,
    ])
