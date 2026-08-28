from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    default_config = PathJoinSubstitution(
        [FindPackageShare("lightning"), "config", "default_livox.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the Lightning-LM YAML configuration file",
            ),
            Node(
                package="lightning",
                executable="run_slam_online",
                name="lightning_slam",
                output="screen",
                arguments=["--config", config],
            ),
        ]
    )
