from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('pinky_mission')) / 'rviz' / 'pinky_mission.rviz')
    params = str(Path(get_package_share_directory('pinky_mission')) / 'config' / 'mission_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='pinky_mission', executable='mission_manager', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'allow_motion': LaunchConfiguration('allow_motion')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
