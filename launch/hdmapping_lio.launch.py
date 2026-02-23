import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hdmapping_lio')
    default_params = os.path.join(pkg_dir, 'config', 'default_params.yaml')
    default_rviz = os.path.join(pkg_dir, 'config', 'hdmapping_lio.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('lidar_topic', default_value='/points'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        Node(
            package='hdmapping_lio',
            executable='hdmapping_lio_node',
            name='hdmapping_lio_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'lidar_topic': LaunchConfiguration('lidar_topic'),
                },
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
