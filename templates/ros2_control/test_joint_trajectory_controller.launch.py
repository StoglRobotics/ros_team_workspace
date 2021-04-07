$LICENSE$

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution([
        FindPackageShare('$PKG_NAME$'),
        'config',
        '$ROBOT_NAME$_joint_trajectory_publisher.yaml'
        ])

    return LaunchDescription([
      Node(
        package='ros2_control_test_nodes',
        executable='publisher_joint_trajectory_controller',
        parameters=[position_goals],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ])
