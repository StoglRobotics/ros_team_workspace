$LICENSE$

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'controllers_package', default_value='ros2_control_demo_robot',
        description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom description.'))
    declared_arguments.append(DeclareLaunchArgument(
        'description_package', default_value='ros2_control_demo_robot',
        description='Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.'))
    declared_arguments.append(DeclareLaunchArgument(
        'prefix', default_value='""', description='Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers\' configuration \
        have to be updated.'))
    declared_arguments.append(DeclareLaunchArgument(
        'use_fake_hardware', default_value='true',
        description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(DeclareLaunchArgument(
        'fake_sensor_commands', default_value='false',
        description='Enable fake command interfaces for sensors used for simple simulations. \
            Used only if \'use_fake_hardware\' parameter is true.'))

    # Initialize Arguments
    controllers_package = LaunchConfiguration('controllers_package')
    description_package = LaunchConfiguration('description_package')
    prefix = LaunchConfiguration('prefix')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package),
                              'description', '$ROBOT_NAME$.urdf.xacro']),
        ' ',
        'prefix:=', prefix, ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'fake_sensor_commands:=', fake_sensor_commands, ' ',
        ])

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(controllers_package),
        'config',
        '$ROBOT_NAME$_controllers.yaml'
        ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        'rviz',
        '$ROBOT_NAME$.rviz'
        ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    robot_controller = 'forward_position_controller'
    # robot_controller = 'joint_trajectory_controller'
    robot_controller_spawner = Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=[robot_controller, '-c', '/controller_manager'])

    return LaunchDescription(
        declared_arguments +
        [
            control_node,
            robot_state_pub_node,
            rviz_node,
            joint_state_controller_spawner,
            robot_controller_spawner,
        ])
