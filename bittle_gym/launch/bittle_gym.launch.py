import os
# ROS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKGNAME = 'bittle_gym'


def generate_launch_description():
    
    # ros_gz_bridge_config_file_arg = DeclareLaunchArgument('gz_bridge_config_file', default_value=[
    #     TextSubstitution(text=os.path.join(FindPackageShare(PKGNAME), 'config', 'ros_gz_bridge.yaml'))
    # ])

    # ros_gz_bridge_node = Node(package='ros_gz_bridge', executable='parameter_bridge',
    #     parameters=[
    #         {"config_file": LaunchConfiguration('gz_bridge_config_file')},
    #     ],
    #     output='screen')

    bittle_gym_node = Node(package='bittle_gym', executable='bittle_gym',
        parameters=[
        ],
        output='screen')        


    return LaunchDescription([
        # gz_bridge_config_file_arg,
        # gz_bridge_node
        bittle_gym_node
    ])