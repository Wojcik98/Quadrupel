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
            arguments=[urdf]
        ),
        Node(
            package='rviz2',
            node_executable='rviz2'
        ),
        Node(
            package='quad_inv_kin',
            node_executable='leg_inv_kin_srv',
        ),
        Node(
            package='quad_gaits',
            node_executable='crab_gait',
            output='screen'
        ),
    ])
