import os
import yaml

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_package = LaunchConfiguration("moveit_package")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    # -------------------------------------------------------#
    #               MoveIt2 MoveGroup setup                 #
    # -------------------------------------------------------#

    # URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    # SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "srdf", semantic_description_file]
            ),
            
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content.perform(context)}

    # kinematics    
    kinematics_yaml = load_yaml("myrobot_moveit", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # limits
    limits_yaml = load_yaml("myrobot_moveit", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": limits_yaml}
   
    # planning
    ompl_planning_plugin_yaml = load_yaml("myrobot_moveit", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_plugin_yaml}

    # load Moveit2 controllers
    moveit_controllers = load_yaml("myrobot_moveit", "config/moveit_controllers.yaml")

    # configure PlanningSceneMonitor
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/MoveGroupExecuteTrajectoryAction"
    }

    # Start the actual move_group node/action server
    # -------------------------------------------------------#
    #                 Move Group Node                        #
    # -------------------------------------------------------#
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities
        ],
    )
    
    # RViz
    rviz_directory = os.path.join(
        get_package_share_directory(
            f"{moveit_package.perform(context)}"
        ), "rviz"
    )
    rviz_config = os.path.join(rviz_directory, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
        ],
    )

    return [
            move_group_node,
            rviz_node,
        ]


def generate_launch_description():
    #
    # ------------- Declare arguments  ------------- #
    #
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="myrobot_description",
            description="Package with the robot URDF/XACRO files. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="myrobot.urdf.xacro",
            description="URDF/XACRO description file \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_package",
            default_value="myrobot_moveit",
            description="MoveIt config package with robot SRDF/XACRO files. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value="myrobot.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
