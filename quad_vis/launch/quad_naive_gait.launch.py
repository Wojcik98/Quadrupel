import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('quad_vis'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        Node(  # it doesn't seem to work but let's keep it to be nice
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen', arguments=[urdf]
        ),
        Node(
            package='quad_inv_kin',
            node_executable='leg_inv_kin_srv',
            output='screen'
        ),
        Node(
            package='naive_crawl',
            node_executable='naive_crawl'
        ),
        Node(
            package='rviz2',
            node_executable='rviz2'
        ),
    ])
