import os
import launch
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def launch_node(context, *args, **kwargs):

    return [Node(
        package="pointcloud2_aggregator",
        namespace="monitor",
        executable="monitor",
        name="pointcloud2_monitor",
        parameters=[{"node_config": LaunchConfiguration('config').perform(context)}],
        # ros_arguments=["--log-level", "debug"]
    )]

def generate_launch_description():

    aggregator_config_path = os.path.join(
        get_package_share_directory('pointcloud2_aggregator'),
        'config', 'monitor.yaml'
    )

    # Aggregator Parameters
    aggregator_config_launch_arg =  DeclareLaunchArgument(
        'config', default_value=aggregator_config_path
    )

    # Must be started with the monitor
    component_container = Node(
        package="rclcpp_components",
        namespace="monitor",
        executable="component_container_mt"
    )

    # Launch
    return launch.LaunchDescription([
        aggregator_config_launch_arg,
        component_container,
        OpaqueFunction(function=launch_node)
    ])