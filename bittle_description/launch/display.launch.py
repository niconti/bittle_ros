from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKGNAME = 'bittle_description'


def generate_launch_description():

    urdf_launch_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': TextSubstitution(text=PKGNAME),
            # 'urdf_package_path': PathJoinSubstitution(['urdf', 'bittle.urdf']),
            'urdf_package_path': PathJoinSubstitution(['xacro', 'bittle.urdf.xacro']),
            'rviz_config': PathJoinSubstitution([FindPackageShare(PKGNAME), 'rviz', 'urdf.rviz'])
        }.items()
    )

    return LaunchDescription([
        urdf_launch_include
    ])