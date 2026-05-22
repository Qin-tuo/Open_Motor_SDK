from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("khcan"), "config", "motor.toml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rate_hz", default_value="100.0"),
            DeclareLaunchArgument("auto_enable", default_value="false"),
            DeclareLaunchArgument("default_mode", default_value="-1"),
            Node(
                package="khcan",
                executable="motor_driver_node",
                name="motor_driver_node",
                output="screen",
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "auto_enable": ParameterValue(
                            LaunchConfiguration("auto_enable"), value_type=bool
                        ),
                        "default_mode": ParameterValue(
                            LaunchConfiguration("default_mode"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
