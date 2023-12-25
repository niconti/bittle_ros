import numpy as np
from scipy.spatial.transform import Rotation
# ROS
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class BittleController(Node):

    def __init__(self):
        super().__init__()

        self.observations = np.zeros(shape=(2, 6))

        self.ODOMETRY_TOPIC_NAME = '/model/vehicle_blue/odometry'
        self.odom_sub = self.create_subscription(Odometry, self.ODOMETRY_TOPIC_NAME, callback=self.odometry_cb)


    def odometry_cb(self, msg):

        # pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        R = Rotation.from_quat([qx, qy, qz, qw])
        euler = R.as_euler(seq='xyz')

        self.observations[0][0] = x
        self.observations[0][1] = y
        self.observations[0][2] = z
        self.observations[0][3] = euler[0]
        self.observations[0][4] = euler[1]
        self.observations[0][5] = euler[2]

        # twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        self.observations[1][0] = vx
        self.observations[1][1] = vy
        self.observations[1][2] = vz
        self.observations[1][3] = wx
        self.observations[1][4] = wy
        self.observations[1][5] = wz
