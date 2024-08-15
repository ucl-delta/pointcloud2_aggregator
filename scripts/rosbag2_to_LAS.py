#!/usr/bin/env python3
"""
Python file which attempts to convert a ros2 humble rosbag containing lidar data into the LAS file format. 

Requirements:
```
pip install rosbags laspy pointcloud2 tqdm
```

This script does not need a ros installation to work, it is all offline. 

Usage:

Call this python file with a link to the rosbag folder and the ros topic and where to output the las file

```
python3 rosbag2_to_LAS.py rosbag2_livox_people_detection/ /livox/lidar output
```
"""

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from sensor_msgs.msg import PointCloud2
import laspy
import argparse
import numpy as np
import pointcloud2 as pc2
import os
from tqdm import tqdm
from pprint import pprint

def convert_rosbag_to_las(rosbag_file, ros_topic, output_las_folder, max_points, start_time, duration):
    rosbag_file = os.path.normpath(rosbag_file)
    rosbag_file_basename = os.path.basename(rosbag_file)

    # Open the rosbag file
    bagpath = Path(rosbag_file)

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Create a LAS file for writing
    current_las_file_id = 0
    current_num_points = 0
    current_num_point_records = 0

    output_las_folder = os.path.join(output_las_folder, rosbag_file_basename, ros_topic.split("/")[-1])
    try:
        os.makedirs(output_las_folder)
    except Exception as e:
        print(e)
        pass
    
    output_las_name = os.path.join(output_las_folder, rosbag_file_basename)

    las_header = laspy.LasHeader(point_format=6, version="1.4")
    file_name = f"{output_las_name}_{current_las_file_id}.las"
    writer_las = laspy.open(file_name, mode='w', header=las_header)

    start_time_ns = start_time * 1e9
    if duration > 0:
        end_time_ns = start_time_ns + duration * 1e9
    else:
        end_time_ns = None

    try:
        # Create reader instance and open for reading.
        with AnyReader([bagpath], default_typestore=typestore) as reader:

            connections = [x for x in reader.connections if str(x.topic).startswith(ros_topic)]
            if len(connections) == 0:
                print("Topics Found:")
                pprint([r.topic for r in reader.connections])
                raise RuntimeError(f"No Connections with ros topic: {ros_topic}")
            for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections, start=start_time_ns, stop=end_time_ns), total=reader.message_count):
                if connection.msgtype == "sensor_msgs/msg/PointCloud2":
                    # msg is tof type sensor_msgs/msg/PointCloud2
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    # print(msg.header.frame_id)
                    # print(timestamp?)

                    msg_pointcloud = pc2.read_points(msg)
                    
                    # msg_pointcloud is a numpy array of tuples
                    # use msg_pointcloud.dtype to get the values
                    # current values are: [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8')]
                    # print(msg_pointcloud.dtype)
                    # print(len(msg_pointcloud))
                    # print(msg_pointcloud['timestamp'])
                    # return
                    
                    if current_num_points > max_points:
                        writer_las.close()
                        current_las_file_id += 1
                        current_num_points = 0
                        current_num_point_records = 0

                    if current_num_points == 0:
                        file_name = f"{output_las_name}_{current_las_file_id}.las"
                        writer_las = laspy.open(file_name, mode='w', header=las_header)
                    
                    if current_num_point_records == 1:
                        file_name = f"{output_las_name}_{current_las_file_id}.las"
                        writer_las = laspy.open(file_name, mode='a', header=las_header)
                    
                                            
                    # Create new point record for this 
                    point_record = laspy.ScaleAwarePointRecord.zeros(len(msg_pointcloud), header=las_header)
                    point_record.x = msg_pointcloud['x']
                    point_record.y = msg_pointcloud['y']
                    point_record.z = msg_pointcloud['z']
                    point_record.intensity = msg_pointcloud['intensity']
                    point_record.gps_time = msg_pointcloud['timestamp']

                    if current_num_points == 0:
                        writer_las.write_points(point_record)
                    else:
                        writer_las.append_points(point_record)

                    current_num_points += len(msg_pointcloud)
                    current_num_point_records += 1
    finally:    
        writer_las.close()


def main():
    parser = argparse.ArgumentParser(description='Convert ROS2 rosbag to LAS point cloud file')
    parser.add_argument('rosbag_folder', type=str, help='Path to ROS2 rosbag file')
    parser.add_argument('ros_topic', type=str, help='ROS topic containing PointCloud2 messages')
    parser.add_argument('output_las_folder', type=str, help='Output LAS folder path')
    parser.add_argument('--start_time', '-st', type=int, default=0, help="Seconds from the start of the bag to start conversion")
    parser.add_argument('--conversion_duration', '-d', type=int, default=-1, help="duration of the rosbag from the start time to convert")
    parser.add_argument("--max-points-per-file", "-m", type=int, default=10**7, help="Maximum number of points per las file")
    args = parser.parse_args()

    convert_rosbag_to_las(args.rosbag_folder, args.ros_topic, args.output_las_folder, args.max_points_per_file,
                          args.start_time, args.conversion_duration)

if __name__ == "__main__":
    main()
