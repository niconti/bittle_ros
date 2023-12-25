import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file_name = 'empty.world'


    ros_gz_sim_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': 'empty.sdf'
        }.items()
    )

    # Parse robot description from xacro
    robot_description_file = os.path.join(get_package_share_directory('bittle_description'), 'xacro', 'bittle.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # # Add models to IGN_GAZEBO_RESOURCE_PATH
    # pkg_share_path = os.path.join(get_package_share_directory('bittle_description'))
    # if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
    #     os.environ['IGN_GAZEBO_RESOURCE_PATH'] += pkg_share_path
    # else:
    #     os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  pkg_share_path


    ros_gz_sim_spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[ 
                     '-file', PathJoinSubstitution([FindPackageShare('bittle_description'), 'xacro', 'bittle.urdf']) 
                 ], output='screen')

    # ros_gz_sim_spawn = Node(package='ros_gz_sim', executable='create',
    #             arguments=[ 
    #                 '-name', 'bittle',
    #                 '-topic', 'robot_description', 
    #             ], output='screen')
    
    return LaunchDescription([
        ros_gz_sim_include,
        # robot_state_publisher,
        ros_gz_sim_spawn
    ])


