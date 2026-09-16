from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the online ROS 2 localization node."""
    config = LaunchConfiguration("config")
    default_config = PathJoinSubstitution(
        [FindPackageShare("lightning"), "config", "indoor_mid360s.yaml"]
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
                executable="run_loc_online",
                name="lightning_localization",
                output="screen",
                arguments=["--config", config],
            ),
        ]
    )
