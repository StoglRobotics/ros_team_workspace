$LICENSE$

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    position_goals = os.path.join(
        get_package_share_directory('$PKG_NAME$'),
        'config',
        '$ROBOT_NAME$_forward_position_publisher.yaml'
        )

    return LaunchDescription([
      Node(
        package='ros2_control_test_nodes',
        executable='publisher_forward_position_controller',
        parameters=[position_goals],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ])
