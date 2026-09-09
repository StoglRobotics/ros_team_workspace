#!/bin/bash
#
# Copyright (c) 2021-2026, b»robotized group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import rclpy

from launch import LaunchDescription
from launch_ros.actions import Node  # noqa: F401
from launch.actions import TimerAction
import launch_testing.actions

# TODO: add any other imports needed (e.g. service/action types)


def generate_test_description():

    minimal_urdf = (  # noqa: F841
        """<?xml version="1.0"?><robot name="dummy_robot"><link name="base_link"/></robot>"""
    )
    minimal_srdf = """<robot name="dummy_robot"><group name="dummy_group"><chain base_link="base_link" tip_link="base_link"/></group></robot>"""  # noqa: F841

    # TODO: Replace with whatever nodes/launch files are needed to run your system.
    # If reusing an existing launch file, add parameters to disable UI components like rviz.
    # NODE_NAME = Node(
    #     package="$PKG_NAME$",
    #     executable="NODE_NAME",
    #     name="NODE_NAME",
    #     output="screen",
    # )

    # moveit_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         {"robot_description": minimal_urdf},
    #         {"robot_description_semantic": minimal_srdf},
    #         {"planning_pipelines": ["ompl"]},
    #         {"default_planning_pipeline": "ompl"},
    #         {"ompl": {"planning_plugins": ["ompl_interface/OMPLPlanner"]}},
    #         {"allow_trajectory_execution": False},
    #     ],
    # )

    # TODO: Adjust the period to however long your system takes to fully start up.
    return (
        LaunchDescription(
            [
                # moveit_node,
                # NODE_NAME,
                TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        # {"moveit_node": moveit_node, "custom_node": NODE_NAME},
    )


# Active tests
class TestPkgName(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_$PKG_NAME$_node")

        # TODO: Create a client for each service or action you want to test.
        # self.my_service_client = self.node.create_client(MyServiceType, "my_service")

    def tearDown(self):
        self.node.destroy_node()

    # TODO: Add your testcases as separate methods below. Testcases run sequentially,
    # so order matters when one test depends on another (e.g. add object before moving it).
    #
    # Each testcase should trigger a behavior and then assert the expected outcome.
    # def test_my_custom_service(self):
    #     self.assertTrue(
    #         self.my_service_client.wait_for_service(timeout_sec=5.0),
    #         "my_service is not available",
    #     )
    #
    #     request = MyServiceType.Request()
    #
    #     future = self.my_service_client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #
    #     response = future.result()
    #     self.assertIsNotNone(response, "No response received from my_service")
    #     self.assertEqual(response.result, 1, "my_service should succeed")
