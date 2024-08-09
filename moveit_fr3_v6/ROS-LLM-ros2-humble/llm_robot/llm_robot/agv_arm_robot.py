#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Description:
# This code defines a ROS node called MoveItNode
# The node interacts with the MoveIt service to implement motion planning
# The node sets a target pose for the robot arm and plans the motion
# The node executes the planned motion if successful
# The node also includes a ChatGPT function call server to handle function calls from ChatGPTNode
#
# Author: Herman Ye @Auromix

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from llm_interfaces.srv import ChatGPT
import json

class MoveItNode(Node):
    def __init__(self):
        super().__init__('moveit_node')
        self.logger = self.get_logger()
        self.move_group = MoveGroupCommander('agv_arm')
        self.function_call_server = self.create_service(ChatGPT, '/ChatGPT_function_call_service', self.function_call_callback)
        self.logger.info('MoveItNode has been initialized')

    def function_call_callback(self, request, response):
        req = json.loads(request.request_text)
        function_name = req['name']
        function_args = json.loads(req['arguments'])
        func_obj = getattr(self, function_name)
        try:
            function_execution_result = func_obj(**function_args)
        except Exception as error:
            self.logger.info(f'Failed to call function: {error}')
            response.response_text = str(error)
        else:
            response.response_text = str(function_execution_result)
        return response

    def agv_arm_pose(self, **kwargs):
        target_pose = Pose()
        target_pose.orientation.w = float(kwargs.get('w', 0.0))
        target_pose.position.x = float(kwargs.get('x', 0.0))
        target_pose.position.y = float(kwargs.get('y', 0.0))
        target_pose.position.z = kwargs.get('z', 0.0)
        self.move_group.set_pose_target(target_pose)
        success, plan, _, _ = self.move_group.plan()
        if success:
            self.logger.info('Planning successful! Executing the plan...')
            self.move_group.execute(plan, wait=True)
            return "Motion executed successfully"
        else:
            self.logger.error('Planning failed!')
            return "Planning failed"

def main(args=None):
    rclpy.init(args=args)
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    print("yes")
    node = MoveItNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
