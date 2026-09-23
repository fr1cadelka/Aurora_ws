from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    anchors_x = [0.0, 4.67, 4.67, 0.0]
    anchors_y = [0.0, 0.0, 10.10, 10.10]
    anchors_z = [1.70, 1.56, 1.82, 1.82]
    field_width  = 4.67
    field_height = 10.10

    # === UWB драйвер ===
    nlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nlink_parser2'),
                'launch', 'linktrack.launch.py')
        ),
        launch_arguments=[('port_name', '/dev/ttyACM0')],
    )

    field_visualizer = Node(
        package='uwb_localization_pkg',
        executable='field_visualizer_pkg_node',
        name='field_visualizer_pkg_node',
        output='screen',
        parameters=[{
            'anchors_x': anchors_x, 'anchors_y': anchors_y, 'anchors_z': anchors_z,
            'field_width': field_width, 'field_height': field_height,
            'lane_width': 1.0,
        }]
    )

    tag_localizer = Node(
        package='uwb_localization_pkg',
        executable='tag_localizer_pkg_node',
        name='tag_localizer_pkg_node',
        output='screen',
        parameters=[{
            'anchors_x': anchors_x, 'anchors_y': anchors_y, 'anchors_z': anchors_z,
            'frame_id_map': 'map', 'frame_id_odom': 'odom',
            'frame_id_robot': 'base_link', 'publish_tf': True,
            'ema_alpha': 0.3,
            'kalman_q_pos': 0.05, 'kalman_q_vel': 0.5, 'kalman_r_pos': 1.0,
            'heading_min_dist': 0.15, 'heading_alpha': 0.3,
            'yaw_comp_alpha': 0.05, 'yaw_min_speed': 0.2, 'yaw_update_min_dt': 0.5,
            'uwb_timeout_s': 1.0,
            # === Покрытие поля ===
            'lane_width': 1.0,
            'coverage_margin': 0.5,     # 0.5 м от границы
            'coverage_exit_m': 2.0,     # выезд за пределы поля
            'field_x_min': 0.0,
            'field_x_max': 4.67,
            'field_y_min': 0.0,
            'field_y_max': 10.10,
        }]
    )

    return LaunchDescription([nlink_launch, field_visualizer, tag_localizer])
