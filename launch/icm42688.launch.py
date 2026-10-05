import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# CPU3由isolcpus隔离，节点及其IIO读取线程统一继承该CPU亲和性。
CPU_CORE = 3


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("icm42688_driver"),
        "config",
        "icm42688_params.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="icm42688_driver",
            executable="icm42688_driver",
            name="icm42688_driver",
            output="screen",
            prefix=[f"taskset -c {CPU_CORE}"],
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
