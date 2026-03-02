from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot4-patrol-show"),
            "config",
            "patrol-config.yaml",
        ]
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to YAML config file with tb4_patrol parameters",
    )

    tb4_patrol_node = Node(
        package="turtlebot4-patrol-show",
        executable="tb4_patrol",
        name="tb4_patrol",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([
        config_arg,
        tb4_patrol_node,
    ])
