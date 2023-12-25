#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld


class GazeboConnector(Node):

    def __init__(self, world_name='empty'):
        """
        """
        self.WORLD_NAME = world_name
        self.SERVICE_TIMEOUT = 3.0
        self.SERVICE_NAME = f"/world/{self.WORLD_NAME}/control"
        self.control_world_cli = self.create_client(ControlWorld, self.SERVICE_NAME)

    def pause(self):
        if not self.control_world_cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().error(f"{self.SERVICE_NAME} service timeout after {self.SERVICE_TIMEOUT}s.")
            return False
        try:
            req = ControlWorld.Request()
            req.world_control.pause = True
            res = self.control_world_cli.call()
        except TypeError as ex:
            self.get_logger().error(f"{self.SERVICE_NAME} service call failed")
            return False
        return True
        
    def unpause(self):
        if not self.control_world_cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().error(f"{self.SERVICE_NAME} service timeout after {self.SERVICE_TIMEOUT}s.")
            return False
        try:
            req = ControlWorld.Request()
            req.world_control.pause = False
            res = self.control_world_cli.call()
        except TypeError as ex:
            self.get_logger().error(f"{self.SERVICE_NAME} service call failed")
            return False
        return True
    
    def step(self):
        if not self.control_world_cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().error(f"{self.SERVICE_NAME} service timeout after {self.SERVICE_TIMEOUT}s.")
            return False
        try:
            req = ControlWorld.Request()
            req.world_control.step = True
            res = self.control_world_cli.call()
        except TypeError as ex:
            self.get_logger().error(f"{self.SERVICE_NAME} service call failed")
            return False
        return True

    def reset(self, seed=None):
        if not self.control_world_cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().error(f"{self.SERVICE_NAME} service timeout after {self.SERVICE_TIMEOUT}s.")
            return False
        try:
            req = ControlWorld.Request()
            req.world_control.reset.all = True
            res = self.control_world_cli.call()
        except TypeError as ex:
            self.get_logger().error(f"{self.SERVICE_NAME} service call failed")
            return False
        return True