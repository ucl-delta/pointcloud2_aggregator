# Pointcloud Aggregation

This ros2 node is designed to aggregate PointCloud2 messages collected over time into one single output with an aggregated PointCloud2. 

This repository has been tested with ROS2 Humble on Ubuntu 22.04 using Livox Lidars

For the livox lidars, this assumes the use of the [livox_ros_driers2](https://github.com/Livox-SDK/livox_ros_driver2) repository to generate the ros streams as either a single stream, or one for each camera. 

## Installation

Clone this reposistory into your ros2 workspace and colcon build it

```bash
cd <your ros2 workspace>
cd src
git clone ... 
cd ../
colcon build --packages-select pointcloud2_aggregator
```

## Usage

### aggregator node

The aggregator node is the primary ros2 node of the system. It takes one `sensor_msgs.msg.PointCloud2` topic stream and aggregates it over a given `time_window`. 

The node parameters are as follows:

| Parameter               | Description                                                                    | Default       |
|-------------------------|--------------------------------------------------------------------------------|---------------|
| `publish_frequency`     | The Frequency to publish aggregated pointclouds at                             | 5hz           |
| `time_window`           | The time window to aggregate pointclouds for                                   | 1s            |
| `pointcloud_topic`      | The pointcloud topic to subscribe to                                           | `/livox/lidar |
| `pointcloud_topic_freq` | The frequency to sample the receive pointcloud msgs at over the pointcloud topic | 10hz          |

Unlike other aggregators, this system tries to minimise the number of copies being performed. 

When a pointcloud2 msg arrives, it is copied once into a ring buffer sized to accept a time window number of point clouds. 

On the publish frequency, we iterate through the ring buffer and block memcopy the raw PointCloud2.data vector into an aggregate msg. We do not iterate through and copy each individual cloud point.

A launch files is provided which reads the launch parameters off of `config/aggregate.yaml` 
```bash
ros2 launch pointcloud2_aggregator aggregator.launch.py 
```

The node can be run directly using 

```bash
ros2 run pointcloud2_aggregator aggregate --ros-args -p pointcloud_topic:=<my pointcloud topic>
```

> Note that a transform will be required between the pointcloud frame and the existing tf tree for accurate visualisation. 

> Note that the msg size very quickly increase >10mb. You may need to increase send buffers on some applications. e.g. for Foxglove: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=100000000`

To save on bandwidth, this node only subscribes to the source stream if more than one subscriber is detected on the aggregated stream. 

If you have multiple lidar streams, you can use the `aggregate_auto.launch.py` launch file which will read the currently available topics, and start an aggregation node for each lidar topic matching some substring (defaults to `/livox/lidar`). 

```bash
ros2 launch pointcloud2_aggregator aggregator.launch.py 
```

It reads from the `aggregate.yaml` config as before. 

> The substring can be changed by providing the `lidar_topic_substring:="my_new_substring"` after the launch command. Similarly the `config:=<path to config>` can be used to change the config file. 

> Note this only reacts to currently broadcasting topics at the time of running. It does not spin up and down aggregate nodes in real time (need to understand how composite nodes work). 

### rosbag to las script

There exists a rosbag to las script which attempts to convert the lidar entries in the rosbag to the LAS file format. You will need to install the following dependencies:

```
```
pip install rosbags laspy pointcloud2 tqdm
```

Call this python file with a link to the rosbag folder and the ros topic and where to output the las file

```
python3 rosbag2_to_LAS.py rosbag2_livox_people_detection/ /livox/lidar output_folder
```

> There is a `--max-points-per-file` which controls the number of points that should be included in each LAS file before a new one is created. 

It is also callable from inside ROS after buildingby using

```
ros2 run pointcloud2_aggregator rosbag2_to_LAS.py [args]
```


## Future Improvements

- A node which monitors the current topics and automatically spins up and down an aggregate node as a component when required. 