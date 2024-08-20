#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"

class PointCloudCropper : public rclcpp::Node
{
public:
    PointCloudCropper()
    : Node("pointcloud_cropper")
    {
        this->declare_parameter("crop_max", 3.0);
        this->declare_parameter("crop_min", -3.0);
        this->declare_parameter("crop_axis", "x");

        auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());


        // Subscriber: listens to the input point cloud topic
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/input_pointcloud", default_qos, std::bind(&PointCloudCropper::pointCloudCallback, this, std::placeholders::_1));

        // Publisher: publishes the cropped point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropped_pointcloud", rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "Initialised Pointcloud2 Cropper");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
    {
        // Convert the ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *pcl_cloud);

        // Apply the z-axis cropping filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName(this->get_parameter("crop_axis").as_string());
        // pass.setFilterLimits(std::numeric_limits<float>::min(), this->get_parameter("crop_height").as_double());  // Crop points with z > 5.0
        pass.setFilterLimits(this->get_parameter("crop_min").as_double(), this->get_parameter("crop_max").as_double());  // Crop points with z > 5.0
        pass.filter(*pcl_cloud);

        // Convert the filtered PCL PointCloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*pcl_cloud, output_msg);
        output_msg.header = input_msg->header;  // Maintain the same header as the input

        // Publish the cropped point cloud
        pointcloud_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCropper>());
    rclcpp::shutdown();
    return 0;
}
