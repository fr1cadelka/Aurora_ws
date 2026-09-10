#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    set_domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='44')

    rover_ip = LaunchConfiguration('rover_ip', default='192.168.1.100')
    tcp_port = LaunchConfiguration('tcp_port', default='6000')
    udp_port = LaunchConfiguration('udp_port', default='12346')

    return LaunchDescription([
        set_domain_id,

        Node(
            package='tcp_pkg',                   # имя пакета (из install/tcp_pkg)
            executable='tcp_server_node',        # имя из add_executable
            name='tcp_server_node',
            parameters=[{'server_port': tcp_port}],
            output='screen',
        ),

        # Node(
        #     package='udp_pkg',                   # имя пакета (из install/udp_pkg)
        #     executable='udp_client_node',        # проверьте в CMakeLists.txt udp_pkg
        #     name='udp_client_node',
        #     parameters=[{'server_ip': rover_ip, 'server_port': udp_port}],
        #     output='screen',
        # ),
    ])
