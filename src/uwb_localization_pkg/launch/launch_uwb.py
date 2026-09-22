from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # === ОБЩИЕ ПАРАМЕТРЫ МАЯКОВ (меняйте здесь!) ===
    anchors_x = [0.0, 4.67, 4.67, 0.0]
    anchors_y = [0.0, 0.0, 10.1, 10.10]
    anchors_z = [1.70, 1.56, 1.82, 1.82]
    field_width  = 4.67
    field_height = 10.10

    field_visualizer = Node(
        package='uwb_localization_pkg',
        executable='field_visualizer_pkg_node',
        name='field_visualizer_pkg_node',
        output='screen',
        parameters=[{
            'anchors_x': anchors_x,
            'anchors_y': anchors_y,
            'anchors_z': anchors_z,
            'field_width': field_width,
            'field_height': field_height,
            'lane_width': 1.0,
        }]
    )

    tag_localizer = Node(
        package='uwb_localization_pkg',
        executable='tag_localizer_pkg_node',
        name='tag_localizer_pkg_node',
        output='screen',
        parameters=[{
            'anchors_x': anchors_x,
            'anchors_y': anchors_y,
            'anchors_z': anchors_z,
            'frame_id_map': 'map',
            'frame_id_robot': 'base_link',
            'ema_alpha': 0.3,
            'kalman_q_pos': 0.05,
            'kalman_q_vel': 0.5,
            'kalman_r_pos': 1.0,
            'deadband_m': 0.05,
            'heading_min_dist': 0.10,
            'heading_alpha': 0.3,
            'lane_width': 1.0,
            'field_margin': 1.0,
            'field_x_min': 0.0,
            'field_x_max': 3.0,
            'field_y_min': 0.0,
            'field_y_max': 8.0,
        }]
    )

    return LaunchDescription([field_visualizer, tag_localizer])
