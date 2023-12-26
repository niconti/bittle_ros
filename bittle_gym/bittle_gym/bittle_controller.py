import numpy as np
from scipy.spatial.transform import Rotation
# ROS
import rclpy
import rclpy.qos
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry

NUM_LEGS = 4
ROBOT_NAME = 'bittle'


class BittleController(Node):

    def __init__(self, joint_names):
        super().__init__('bittle_controller', namespace=ROBOT_NAME)

        # Observations
        self.joint_states_observation = np.zeros(shape=(2, 8))
        self.JOINT_STATES_TOPIC_NAME = '/joint_states'
        self.joint_states_sub = self.create_subscription(JointState, self.JOINT_STATES_TOPIC_NAME, 
                                                         callback=self.joint_states_cb, 
                                                         qos_profile=rclpy.qos.ReliabilityPolicy.RELIABLE)        
        
        self.imu_observation = np.zeros(shape=(2, 3))
        self.IMU_TOPIC_NAME = '/imu'
        self.imu_sub = self.create_subscription(Imu, self.IMU_TOPIC_NAME, 
                                                callback=self.imu_cb,
                                                qos_profile=rclpy.qos.ReliabilityPolicy.RELIABLE)

        # Actions
        self.joint_names = joint_names
        self.joint_cmd_pos_pubs = []
        for i in range(len(self.joint_names)):        
            self.JOINT_CMD_POS_TOPIC_NAME = f"/joint/{joint_names[i]}/cmd_pos"
            pub = self.create_publisher(Float64, self.JOINT_CMD_POS_TOPIC_NAME,
                                        qos_profile=rclpy.qos.ReliabilityPolicy.RELIABLE)
            self.joint_cmd_pos_pubs.append(pub)
            self.get_logger().info(f"Publisher: {self.JOINT_CMD_POS_TOPIC_NAME}")


    def joint_states_cb(self, msg):
        """
        """

        # pisition
        if self.joint_states_observation.shape[0] \
        != len(msg.position):
            self.get_logger().error(f"obervation.shape: {self.joint_states_observation.shape[0]} != msg.position: {len(msg.position)}")
            return

        for i in range(len(self.joint_states_observation[0])):
            self.joint_states_observation[0][i] = msg.position[i]

        # velocity
        if self.joint_states_observation.shape[0] \
        != len(msg.velocity):
            self.get_logger().error(f"obervation.shape: {self.joint_states_observation.shape[0]} != msg.velocity: {len(msg.velocity)}")
            return

        for i in range(len(self.joint_states_observation[1])):
            self.joint_states_observation[1][i] = msg.velocity[i]


    def imu_cb(self, msg):
        """
        """
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        R = Rotation.from_quat([qx, qy, qz, qw])
        euler = R.as_euler(seq='xyz')

        rx = euler[0]
        ry = euler[1]
        rz = euler[2]

        self.imu_observation[0][0] = rx
        self.imu_observation[0][1] = ry
        self.imu_observation[0][2] = rz

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z


    def pub_joint_cmd_pos(self, action):
        """
        """

        if action.size \
        != len(self.joint_cmd_pos_pubs):
            self.get_logger().error(f"action.size: {action.size} != joint_cmd_pos_pubs: {len(self.joint_cmd_pos_pubs)}")
            return

        for i in range(action.shape[0]):
            for j in range(action.shape[1]):
                msg = Float64()
                msg.data = action[i][j]
                self.joint_cmd_pos_pubs[action.shape[1]*i+j].publish(msg)