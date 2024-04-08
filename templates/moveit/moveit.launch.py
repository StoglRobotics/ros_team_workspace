from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_package = LaunchConfiguration("moveit_package")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_sim_time = LaunchConfiguration("use_sim_time")
    logging_severity = LaunchConfiguration("severity")

    # -------------------------------------------------------#
    #               MoveIt2 MoveGroup setup                  #
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

    robot_description = {"robot_description": robot_description_content.perform(context)}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content.perform(context)}
    planning_config = PathJoinSubstitution([FindPackageShare(moveit_package), "config", "ompl_planning.yaml"])
    move_group_config = PathJoinSubstitution([FindPackageShare(moveit_package), "config", "move_group.yaml"])

    # -------------------------------------------------------#
    #                 Move Group Node                        #
    # -------------------------------------------------------#
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", logging_severity],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_config,
            move_group_config,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "rviz", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "WARN"],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_config,
            move_group_config,
            {"use_sim_time": use_sim_time},
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
            default_value="$DESCR_PKG_NAME$",
            description="Package with the robot URDF/XACRO files. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="$ROBOT_NAME$.urdf.xacro",
            description="URDF/XACRO description file \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_package",
            default_value="$PKG_NAME$",
            description="MoveIt config package with robot SRDF/XACRO files. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value="$ROBOT_NAME$.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot. \
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock instead of world clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "severity",
            default_value="INFO",
            choices=["INFO", "WARN", "ERROR", "DEBUG", "FATAL"],
            description="Logging level for the nodes.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
