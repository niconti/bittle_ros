import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

PKGNAME = 'bittle_gym'


def generate_launch_description():
    
    gz_bridge_config_file_arg = DeclareLaunchArgument('gz_bridge_config_file', default_value=[
        TextSubstitution(text=os.path.join(get_package_share_directory(PKGNAME), 'config', 'ros_gz_bridge.yaml'))
    ])

    gz_bridge_node = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    parameters=[
                        {"config_file": LaunchConfiguration('gz_bridge_config_file')},
                    ],
                    output='screen', emulate_tty=True)        

    return LaunchDescription([
        gz_bridge_config_file_arg,
        gz_bridge_node
    ])