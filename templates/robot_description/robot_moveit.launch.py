# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Authors: Nikola Banović, Dr. Denis
#
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    only_rviz = LaunchConfiguration("only_rviz")

    # Get SRDF via xacro
    robot_srdf_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("$PKG_NAME$"), "srdf", "$ROBOT_NAME$.srdf.xacro"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_srdf_content.perform(context)
    }

    moveit_config_package_name = "$ROBOT_NAME$_moveit_config"

    controllers_yaml = load_yaml(
        moveit_config_package_name, "config/$ROBOT_NAME$/controllers.yaml"
    )
    ompl_planning_yaml = load_yaml(
        moveit_config_package_name, "config/$ROBOT_NAME$/ompl_planning.yaml"
    )
    kinematics_yaml = load_yaml(moveit_config_package_name, "config/$ROBOT_NAME$/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    limits_yaml = load_yaml(moveit_config_package_name, "config/$ROBOT_NAME$/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": limits_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Moveit controllers Configuration
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    plan_execution = {
        "plan_execution.record_trajectory_state_frequency": 10.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            plan_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # PathJoinSubstitution(
            #     [FindPackageShare("$PKG_NAME$"), "$ROBOT_NAME$_moveit", "moveit_execution_and_control.yaml"]
            # ),
            # PathJoinSubstitution(
            #     [FindPackageShare("$PKG_NAME$"), "$ROBOT_NAME$_moveit", "planning_pipelines.yaml"]
            # ),
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(only_rviz),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("$PKG_NAME$"), "rviz", "$ROBOT_NAME$_moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            # PathJoinSubstitution(
            #     [FindPackageShare("$PKG_NAME$"), "$ROBOT_NAME$_moveit", "moveit_execution_and_control.yaml"]
            # ),
            # PathJoinSubstitution(
            #     [FindPackageShare("$PKG_NAME$"), "$ROBOT_NAME$_moveit", "planning_pipelines.yaml"]
            # ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz or only_rviz),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock instead of world clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch Rviz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "only_rviz",
            default_value="false",
            description="Start only rviz with MoveIt setup. This is useful when MoveIt is running on another machine, e.g., on the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
