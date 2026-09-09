from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch SLAM, RViz2, and optional rosbag2 playback for offline debugging."""
    bag = LaunchConfiguration("bag")
    config = LaunchConfiguration("config")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = FindPackageShare("lightning")
    default_config = PathJoinSubstitution(
        [package_share, "config", "indoor_mid360s.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "config", "lightning.rviz"]
    )

    slam_node = Node(
        package="lightning",
        executable="run_slam_online",
        name="lightning_slam",
        output="screen",
        arguments=["--config", config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag, "--clock"],
        condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                default_value="",
                description=(
                    "Path to a rosbag2 directory. Leave empty to disable "
                    "automatic playback."
                ),
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the Lightning-LM YAML configuration file",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to the RViz2 display configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulated clock published by rosbag2",
            ),
            slam_node,
            rviz_node,
            # Give the subscribers time to initialize before the first replayed
            # sensor messages are published.
            TimerAction(period=1.0, actions=[bag_play]),
        ]
    )
