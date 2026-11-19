from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("shoot_belt_measurement"))
    parameter_file = package_share / "config" / "measurement.yaml"

    return LaunchDescription([
        Node(
            package="shoot_belt_measurement",
            executable="shoot_belt_measurement_node",
            name="shoot_belt_measurement_node",
            output="screen",
            parameters=[str(parameter_file)],
        ),
    ])
