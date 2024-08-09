#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This example demonstrates simulating function calls for any robot,
# such as controlling velocity and other service commands.
# By modifying the content of this file,
# A calling interface can be created for the function calls of any robot.
# The Python script creates a ROS 2 Node
# that controls the movement of the TurtleSim
# by creating a publisher for cmd_vel messages and a client for the reset service.
# It also includes a ChatGPT function call server
# that can call various functions to control the TurtleSim
# and return the result of the function call as a string.
#
# Author: Herman Ye @Auromix

# ROS related
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatGPT
from geometry_msgs.msg import Pose

# LLM related
import json
from llm_config.user_config import UserConfig
import time

# Global Initialization
config = UserConfig()


class Chatgpt_Data_Publisher(Node):
    def __init__(self):
        super().__init__("chatgpt_data_publisher")
        # Client for reset
        # self.reset_client = self.create_client(Empty, "/reset")
        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Pose, "/ChatGpt_arm_pose_data", 10)

        # while not self.reset_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Service /reset not available, waiting again...")
        # Server for function call
        self.function_call_server = self.create_service(
            ChatGPT, "/ChatGPT_function_call_service", self.function_call_callback
        )
        # Node initialization log
        self.get_logger().info("Chatgpt_Data_publisher has been initilised")

    def function_call_callback(self, request, response):
        req = json.loads(request.request_text)
        function_name = req["name"]
        function_args = json.loads(req["arguments"])
        func_obj = getattr(self, function_name)
        try:
            function_execution_result = func_obj(**function_args)
        except Exception as error:
            self.get_logger().info(f"Failed to call function: {error}")
            response.response_text = str(error)
        else:
            response.response_text = str(function_execution_result)
        return response

    def agv_arm_pose(self, **kwargs):
        """
        Publishes Target position flavour data to the topic
        """
        target_pose = Pose()
        target_pose.orientation.w = float(kwargs.get('w', 0.0))
        target_pose.position.x = float(kwargs.get('x', 0.0))
        target_pose.position.y = float(kwargs.get('y', 0.0))
        target_pose.position.z = float(kwargs.get('z', 0.0))
        self.publisher_.publish(target_pose)
        self.get_logger().info(f"Publishing target position flavour message successfully: {target_pose}")
        # time.sleep(10)
        return target_pose
        # chocolate = kwargs.get("chocolate", 0.0)
        # vanilla = kwargs.get("vanilla", 0.0)
        # blueberry = kwargs.get("blueberry", 0.0)

        # ice_cream_msg = String()
        # ice_cream_msg.data = f"chocolate: {chocolate}, vanilla: {vanilla}, blueberry: {blueberry}"

        # self.publisher_.publish(ice_cream_msg)
        # self.get_logger().info(f"Publishing ice cream flavour message successfully: {ice_cream_msg.data}")
        # return ice_cream_msg.data


def main():
    rclpy.init()
    chatgpt_data_publisher = Chatgpt_Data_Publisher()
    rclpy.spin(chatgpt_data_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
