#ifndef TRANSFORMER_CPP
#define TRANSFORMER_CPP

#include <cstdio>

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloud2_Transformer: public rclcpp::Node 
{
    public:
        PointCloud2_Transformer();

    private:

        void pointcloud2_sub1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub1;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub;

        std::string topic1_name;
        std::string out_topic;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};

#endif