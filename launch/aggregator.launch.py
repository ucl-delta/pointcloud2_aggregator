import os
import launch
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def launch_node(context, *args, **kwargs):

    aggregator_config = Path(
        get_package_share_directory('pointcloud2_aggregator'),
        'config',
        LaunchConfiguration('config').perform(context)
    )

    return [Node(
        package="pointcloud2_aggregator",
        namespace="aggregator",
        executable="aggregator",
        name="pointcloud2_aggregator",
        parameters=[aggregator_config],
        ros_arguments=["--log-level", "debug"]
    )]

def generate_launch_description():

    # Aggregator Parameters
    aggregator_config_launch_arg =  DeclareLaunchArgument(
        'config', default_value='aggregator.yaml'
    )

    # Enable Transform
    enable_sim_tf_launch_arg = DeclareLaunchArgument(
        'enable_sim_tf', default_value='false'
    )
    sim_tf_child_launch_arg = DeclareLaunchArgument(
        'sim_tf_child', default_value="livox_frame"
    )
    sim_tf_parent_launch_arg = DeclareLaunchArgument(
        'sim_tf_parent', default_value="base_frame"
    )

    enable_sim_tf_conditioned = ExecuteProcess(
        condition=LaunchConfigurationEquals(LaunchConfiguration("enable_sim_tf"), "true"),
        cmd=[[
            "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 ",
            LaunchConfiguration("sim_tf_child"),
            LaunchConfiguration("sim_tf_parent")
        ]],
        shell=True
    )

    # Launch
    return launch.LaunchDescription([
        aggregator_config_launch_arg,
        enable_sim_tf_launch_arg,
        sim_tf_child_launch_arg,
        sim_tf_parent_launch_arg,
        # enable_sim_tf_conditioned,
        OpaqueFunction(function=launch_node)
    ])