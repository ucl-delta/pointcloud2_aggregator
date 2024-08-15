#ifndef COMBINER_HPP
#define COMBINER_HPP

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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloud2_Combiner: public rclcpp::Node 
{
    public:
        PointCloud2_Combiner();

    private:

        void pointcloud2_sub1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
        void pointcloud2_sub2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

        std::shared_ptr<sensor_msgs::msg::PointCloud2>  combinePointClouds(
            sensor_msgs::msg::PointCloud2::SharedPtr cloud1,
            sensor_msgs::msg::PointCloud2::SharedPtr cloud2
        );
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub1;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub2;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub;

        std::string topic1_name;
        std::string topic2_name;
        std::string out_topic;
        std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud2_buffer;

        std::atomic<bool> combining = false;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};

#endif