import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='motor_control',
            executable='dc_motor',
            name='motor_sys',
            parameters=[config]
        ),
        Node(
            package='motor_control',
            executable='set_point',
            name='set_point_node',
            parameters=[config]
        ),
        Node(
            package='motor_control',
            executable='ctrl',
            name='controller_node',
            parameters=[config]
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=['/set_point/data', '/motor_speed_y/data']
        )
    ])