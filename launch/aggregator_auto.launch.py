import time
import rclpy
from pathlib import Path

from rclpy.node import Node as rclpyNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_namespace_from_topics(substring, slash_position=-1, no_split=False, retries=5, wait=2):
    # Filters topics by substring
    # Returns a list of namespaces by slash_position
    # i.e. /livox/lidar_xxx_xxx returns lidar_xxx_xxx with slash_position=2 (or -1)
    rclpy.init()
    node_dummy = rclpyNode("_ros2cli_node")
    for i in range(retries):
        topic_list = node_dummy.get_topic_names_and_types()
        namespaces = set()
        node_dummy.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            node_dummy.get_logger().info(topic_name)
            if substring in topic_name:          
                if no_split:
                    namespaces.add(topic_name)    
                else:
                    namespaces.add(topic_name.split('/')[slash_position])
        node_dummy.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        if len(namespaces) != 0 and i > 2:
            break
        time.sleep(wait)
        node_dummy.get_logger().info(f'No namespaces found, trying again for the {i+1}th time')
    node_dummy.destroy_node()
    rclpy.shutdown()
    return list(namespaces)

def launch_node(context, *args, **kwargs):

    names = get_namespace_from_topics(LaunchConfiguration("lidar_topic_substring").perform(context), no_split=True)

    aggregator_config = Path(
        get_package_share_directory('pointcloud2_aggregator'),
        'config',
        LaunchConfiguration('config').perform(context)
    )

    return [Node(
        package="pointcloud2_aggregator",
        namespace=f"{topic_name}/aggregator",
        executable="aggregator",
        name="pointcloud2_aggregator",
        parameters=[aggregator_config, {"pointcloud_topic": topic_name}],
        # ros_arguments=["--log-level", "debug"]
    ) for topic_name in names]

def generate_launch_description():

    # Aggregator Parameters
    aggregator_config_launch_arg =  DeclareLaunchArgument(
        'config', default_value='aggregator.yaml'
    )

    lidar_topic_substring_launch_arg =  DeclareLaunchArgument(
        'lidar_topic_substring', default_value='/livox/lidar'
    )

    # Launch
    return LaunchDescription([
        aggregator_config_launch_arg,
        lidar_topic_substring_launch_arg,
        OpaqueFunction(function=launch_node)
    ])