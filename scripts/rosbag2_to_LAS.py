
"""
Python file which attempts to convert a ros2 humble rosbag containing lidar data into the LAS file format. 

Requirements:
```
pip install rosbags laspy pointcloud2
```

This script does not need a ros installation to work, it is all offline. 

Usage:

Call this python file with a link to the rosbag folder and the ros topic and where to output the las file

```
python3 rosbag2_to_LAS.py rosbag2_livox_people_detection/ /livox/lidar output.las
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


def convert_rosbag_to_las(rosbag_file, ros_topic, output_las_file):
    # Open the rosbag file
    bagpath = Path(rosbag_file)

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Create a LAS file for writing
    # las_file = File(output_las_file, mode='w', header=None)
    las_header = laspy.LasHeader(point_format=6, version="1.4")

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        with laspy.open(output_las_file, mode='w', header=las_header) as writer:
            connections = [x for x in reader.connections if str(x).startswith(ros_topic)]
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                if connection.msgtype == "sensor_msgs/msg/PointCloud2":
                    # msg is tof type sensor_msgs/msg/PointCloud2
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    print(msg.header.frame_id)

                    msg_pointcloud = pc2.read_points(msg)
                    
                    # msg_pointcloud is a numpy array of tuples
                    # use msg_pointcloud.dtype to get the values
                    # current values are: [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8')]
                    # print(msg_pointcloud.dtype)

                    # return
                    # Not sure what happens here? 
                    point_record = laspy.ScaleAwarePointRecord.zeros(my_data.shape[0], header=header)
                    point_record.x = msg.[:, 0]
                    point_record.y = my_data[:, 1]
                    point_record.z = my_data[:, 2]



def main():
    parser = argparse.ArgumentParser(description='Convert ROS2 rosbag to LAS point cloud file')
    parser.add_argument('rosbag_file', type=str, help='Path to ROS2 rosbag file')
    parser.add_argument('ros_topic', type=str, help='ROS topic containing PointCloud2 messages')
    parser.add_argument('output_las_file', type=str, help='Output LAS file path')
    args = parser.parse_args()

    convert_rosbag_to_las(args.rosbag_file, args.ros_topic, args.output_las_file)

if __name__ == "__main__":
    main()
