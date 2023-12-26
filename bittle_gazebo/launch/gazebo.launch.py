import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

PKGNAME = 'bittle_gazebo'
ROBOT_NAME = 'bittle'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file_name = 'empty.world'

    # Gazebo Sim
    ros_gz_sim_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': 'empty.sdf'
        }.items()
    )

    # # Robot state publisher
    # robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[robot_description],
    #     output='screen',
    # )

    # ros_gz_sim_spawn = Node(package='ros_gz_sim', executable='create',
    #             arguments=[ 
    #                 '-name', 'bittle',
    #                 '-topic', 'robot_description', 
    #             ], output='screen')

    # Gazebo - ROS Bridge
    ros_gz_bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Imu (IGN -> ROS2)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Joint states (IGN -> ROS2)
            f'/world/empty/model/{ROBOT_NAME}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # # JointPositionController (IGN <- ROS2)
            # f'/model/{ROBOT_NAME}/joint/left_front_shoulder_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/left_front_knee_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/left_back_shoulder_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/left_back_knee_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/right_front_shoulder_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/right_front_knee_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/right_back_shoulder_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # f'/model/{ROBOT_NAME}/joint/right_back_knee_joint/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # JointPositionController (IGN <- ROS2)
            f'/joint/left_front_shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/left_front_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/left_back_shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/left_back_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/right_front_shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/right_front_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/right_back_shoulder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/joint/right_back_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # ControlWorld Service (IGN <- ROS2)
            f'/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
        ],
        remappings=[
            (f'/world/empty/model/{ROBOT_NAME}/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=[ 
            '-name', TextSubstitution(text=ROBOT_NAME),
            '-z', '0.650',
            # '-file', PathJoinSubstitution([FindPackageShare('bittle_description'), 'xacro', 'bittle.urdf']),
            '-file', PathJoinSubstitution([FindPackageShare('bittle_gazebo'), 'models', ROBOT_NAME, 'model.sdf']),
        ], 
        output='screen')

    return LaunchDescription([
        ros_gz_sim_include,
        ros_gz_bridge,
        # robot_state_publisher,
        spawn
    ])


