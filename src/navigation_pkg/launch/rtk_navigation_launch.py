#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ИСПРАВЛЕНО: используем navigation_pkg вместо aurora_navigation_pkg
    nav_pkg = 'navigation_pkg'

    # Пути к конфигурационным файлам
    pkg_share = FindPackageShare(package=nav_pkg).find(nav_pkg)
    config_dir = os.path.join(pkg_share, 'config')

    # Пути к файлам
    ekf_params = os.path.join(config_dir, 'ekf_gps_params.yaml')
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')
    mapviz_config = os.path.join(config_dir, 'mapviz_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'aurora_navigation.rviz')

    # Создаем папку rviz если её нет
    if not os.path.exists(os.path.dirname(rviz_config)):
        os.makedirs(os.path.dirname(rviz_config))

    return LaunchDescription([
        # 1. Запуск драйвера GPS (замените на ваш реальный узел)
        Node(
            package='ublox_dgnss_node',
            executable='ublox_dgnss_node',
            name='ublox_dgnss',
            output='screen',
            parameters=[{
                'CFG_USBOUTPROT_NMEA': True,
                'DEVICE_FAMILY': 'F9P'
            }]
        ),

        # 2. Запуск robot_localization для преобразования GPS в одометрию
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # 3. Запуск трансформатора координат
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'delay': 0.0,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': False,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'wait_for_datum': True,
                'datum': [55.7558, 37.6173, 0.0]  # Замените на координаты вашей базы
            }]
        ),

        # 4. Запуск Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_params,
                'autostart': 'True'
            }.items()
        ),

        # 5. Запуск Mapviz для визуализации карты
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        # 6. Запуск нашей C++ ноды
        Node(
            package='navigation_pkg',
            executable='navigation_pkg_node',
            name='navigation_pkg_node',
            output='screen'
        ),

        # 7. Запуск RViz2 для навигации (опционально)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
        ),
    ])
