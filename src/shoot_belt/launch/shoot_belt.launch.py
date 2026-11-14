from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('shoot_belt'),
        'config',
        'shoot_belt.yaml',
    )
    return LaunchDescription([
        Node(
            package='shoot_belt',
            executable='shoot_belt_sequence',
            name='shoot_belt_sequence',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='shoot_belt',
            executable='shoot_belt_joy_test',
            name='shoot_belt_joy_test',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='shoot_belt',
            executable='shoot_belt_can_rx',
            name='shoot_belt_can_rx',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='shoot_belt',
            executable='shoot_belt_can_tx',
            name='shoot_belt_can_tx',
            output='screen',
            parameters=[config],
        ),
    ])
