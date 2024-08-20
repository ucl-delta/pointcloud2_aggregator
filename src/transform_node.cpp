#include "transformer.hpp"

using namespace std::chrono_literals;

PointCloud2_Transformer::PointCloud2_Transformer(): 
    Node("pointcloud2_transformer"), 
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    // Declare Parameters
    this->declare_parameter("cloud_topic", "cloud1");
    this->declare_parameter("transform_frame_id", "base_link");
    this->declare_parameter("out_topic", "transformed_pointcloud");

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    this->topic1_name = this->get_parameter("cloud_topic").as_string();
    this->pointcloud2_sub1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                this->topic1_name, default_qos, std::bind(&PointCloud2_Transformer::pointcloud2_sub1_callback, this, std::placeholders::_1));

    this->out_topic = this->get_parameter("out_topic").as_string();
    this->merged_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->out_topic, rclcpp::SensorDataQoS());

    std::string frame_id = this->get_parameter("transform_frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "Initialised Transform Node [%s] --[%s]-> %s", 
                this->topic1_name.c_str(), frame_id.c_str(), this->out_topic.c_str());
}

void PointCloud2_Transformer::pointcloud2_sub1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    std::string frame_id = this->get_parameter("transform_frame_id").as_string();

    // Lookup the transform from the cloud2 to cloud1
    geometry_msgs::msg::TransformStamped transform_stamped;
    try{
        transform_stamped = tf_buffer_.lookupTransform(
            frame_id, cloud->header.frame_id, tf2::TimePointZero);
    } catch(tf2::LookupException) {
        RCLCPP_ERROR(this->get_logger(), "Lookup Exception %s not found", frame_id.c_str());
        return;
    }

    // Transform the cloud2 into cloud1 coords
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*cloud, transformed_cloud, transform_stamped);

    this->merged_pub->publish(transformed_cloud);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto monitor = std::make_shared<PointCloud2_Transformer>();
  exec->add_node(monitor);

  exec->spin();

  rclcpp::shutdown();

  return 0;
}