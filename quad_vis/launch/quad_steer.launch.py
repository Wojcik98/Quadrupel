import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node


def run_rviz(_):
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        on_exit=launch.actions.Shutdown()
    )

    rviz = launch.LaunchDescription([
        rviz_node
    ])

    desc_source = launch.LaunchDescriptionSource(rviz)
    return launch.actions.IncludeLaunchDescription(
        launch_description_source=desc_source
    )


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('quad_vis'),
        'urdf',
        'robot.urdf'
    )

    joy_steer = Node(
        package='quad_steer',
        node_executable='joy_steer'
    )

    return launch.LaunchDescription([
        Node(  # it doesn't seem to work but let's keep it to be nice
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            arguments=[urdf]
        ),
        Node(
            package='tf2_ros_own',
            node_executable='transform'
        ),
        Node(
            package='quad_inv_kin',
            node_executable='leg_inv_kin_srv'
        ),
        Node(
            package='joy',
            node_executable='joy_node'
        ),
        Node(
            package='quad_steer',
            node_executable='translation',
            output='screen'
        ),
        joy_steer,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=joy_steer,
                on_stdout=run_rviz
            )
        ),
    ])
