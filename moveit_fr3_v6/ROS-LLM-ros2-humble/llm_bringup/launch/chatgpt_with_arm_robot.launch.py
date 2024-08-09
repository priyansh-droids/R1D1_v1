# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# # flake8: noqa
# #
# # Copyright 2023 Herman Ye @Auromix
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Description:
# # This launch file is a part of ROS-LLM project developed to control and interact with the turtlesim robot or your own robot.
# # The launch file contains a LaunchDescription object which defines the ROS2 nodes to be executed.
# # 
# # Node test Method:
# # ros2 launch llm_bringup chatgpt_with_arm_robot.launch.py
# # ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1


# #by me
# # ros2 launch llm_bringup chatgpt_with_arm_robot.launch.py   -> run this
# # ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1  -> run this if llm input is below otherwies change 'listening' to what u want send to chatgpt

# # Author: Herman Ye @Auromix

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# import time 


# def generate_launch_description():
#     #agv_arm_launch_path = PathJoinSubstitution([FindPackageShare('r1d1_arm_moveit_config'), 'launch', 'demo.launch.py'])
#     agv_arm_launch_path = PathJoinSubstitution([FindPackageShare('fairino3_v6_moveit2_config'), 'launch', 'demo.launch.py']) 
#     agv_arm_bringup_launch =IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(agv_arm_launch_path),)
    
#     return LaunchDescription(
#         [
#             agv_arm_bringup_launch,
#             # Node(
#             #     package="llm_input",
#             #     executable="llm_audio_input",
#             #     name="llm_audio_input",
#             #     output="screen",
#             # ),
#             Node(
#                 package="llm_model",
#                 executable="chatgpt",
#                 name="chatgpt",
#                 output="screen",
#             ),
#             Node(
#                 package="llm_model",
#                 executable="chatgpt_data_publisher",
#                 name="chatgpt_data_publisher",
#                 output="screen",
#             ),
#             time.sleep(10)
#             Node(
#                 package="hello_moveit",
#                 executable="hello_moveit",
#                 name="hello_moveit",
#                 output="screen",
#             ),
#             # Node(
#             #     package="llm_output",
#             #     executable="llm_audio_output",
#             #     name="llm_audio_output",
#             #     output="screen",
#             # ),
#             # Node(
#             #     package="llm_robot",
#             #     executable="agv_arm_robot",
#             #     name="agv_arm_robot",
#             #     output="screen",
#             # ),
            
#         ]
#     )



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
# This launch file is a part of the ROS-LLM project developed to control and interact with the turtlesim robot or your own robot.
# The launch file contains a LaunchDescription object which defines the ROS2 nodes to be executed.
#
# Node test Method:
# ros2 launch llm_bringup chatgpt_with_arm_robot.launch.py
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1

# Usage:
# To run the launch file:
#   ros2 launch llm_bringup chatgpt_with_arm_robot.launch.py
# To publish to the llm_state topic, use:
#   ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
# If you want to change the state being sent to chatgpt, replace 'listening' with your desired state.

# Author: Herman Ye @Auromix

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
# import time

def generate_launch_description():
    # Initialize nodes and launch descriptions
    
    # Define the path to the demo launch file of the robot arm
    agv_arm_launch_path = PathJoinSubstitution(
        [FindPackageShare('fairino3_v6_moveit2_config'), 'launch', 'demo.launch.py']
    )
    # agv_arm_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('r1d1_arm_moveit2_config'), 'launch', 'demo.launch.py']
    # )
    
    # Initialize the launch description for the robot arm
    agv_arm_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(agv_arm_launch_path),
    )
    
    # Initialize the ChatGPT node
    chatgpt_node = Node(
        package="llm_model",
        executable="chatgpt",
        name="chatgpt",
        output="screen",
    )
    
    # Initialize the ChatGPT data publisher node
    chatgpt_data_publisher_node = Node(
        package="llm_model",
        executable="chatgpt_data_publisher",
        name="chatgpt_data_publisher",
        output="screen",
    )
    
    # Delay to ensure the previous nodes are properly initialized
    # time.sleep(10)
    
    # Initialize the MoveIt! node for robot arm control
    hello_moveit_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        name="hello_moveit",
        output="screen",
    )
    
    # Initialize additional nodes (currently commented out)
    
    # Initialize the LLM audio input node
    # llm_audio_input_node = Node(
    #     package="llm_input",
    #     executable="llm_audio_input",
    #     name="llm_audio_input",
    #     output="screen",
    # )
    
    # Initialize the LLM audio output node
    # llm_audio_output_node = Node(
    #     package="llm_output",
    #     executable="llm_audio_output",
    #     name="llm_audio_output",
    #     output="screen",
    # )
    
    # Initialize the AGV arm robot node
    # agv_arm_robot_node = Node(
    #     package="llm_robot",
    #     executable="agv_arm_robot",
    #     name="agv_arm_robot",
    #     output="screen",
    # )
    
    # Return the launch description with all initialized nodes
    return LaunchDescription([
        agv_arm_bringup_launch,
        chatgpt_node,
        chatgpt_data_publisher_node,
        hello_moveit_node,
        
        # Uncomment these lines to add the respective nodes to the launch description
        # llm_audio_input_node,
        # llm_audio_output_node,
        # agv_arm_robot_node,
    ])
