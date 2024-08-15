#include "combiner.hpp"

using namespace std::chrono_literals;

PointCloud2_Combiner::PointCloud2_Combiner(): 
    Node("pointcloud2_combiner"), 
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    // Declare Parameters
    this->declare_parameter("cloud1", "cloud1");
    this->declare_parameter("cloud2", "cloud2");
    this->declare_parameter("out", "combined_pointcloud2");

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    this->topic1_name = this->get_parameter("cloud1").as_string();
    this->pointcloud2_sub1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                this->topic1_name, default_qos, std::bind(&PointCloud2_Combiner::pointcloud2_sub1_callback, this, std::placeholders::_1));

    this->topic2_name = this->get_parameter("cloud2").as_string();
    this->pointcloud2_sub2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                this->topic2_name, default_qos, std::bind(&PointCloud2_Combiner::pointcloud2_sub2_callback, this, std::placeholders::_1));

    this->out_topic = this->get_parameter("out").as_string();
    this->merged_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->out_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Initialised Combining Node [%s] + [%s] -> %s", 
                this->topic1_name.c_str(), this->topic2_name.c_str(), this->out_topic.c_str());
}

void PointCloud2_Combiner::pointcloud2_sub1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    if(this->cloud2_buffer) {
        std::shared_ptr<sensor_msgs::msg::PointCloud2> out_pc
            = this->combinePointClouds(cloud, cloud2_buffer);
        this->merged_pub->publish(*out_pc);
    }
}

void PointCloud2_Combiner::pointcloud2_sub2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{   
    if(!this->combining) {
        this->cloud2_buffer = cloud;
    }
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> PointCloud2_Combiner::combinePointClouds(
  sensor_msgs::msg::PointCloud2::SharedPtr cloud1,
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2)
{
    this->combining = true;
    // assign cloud 1 to output cloud
    //   cloud_out = cloud1;

    // Lookup the transform from the cloud2 to cloud1
    geometry_msgs::msg::TransformStamped transform_stamped;
    try{
        transform_stamped = tf_buffer_.lookupTransform(
            cloud1->header.frame_id, cloud2->header.frame_id, tf2::TimePointZero);
    } catch(tf2::LookupException) {
        return cloud1;
    }

    // Transform the cloud2 into cloud1 coords
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*cloud2, transformed_cloud, transform_stamped);

    // get the lengths of each cloud
    size_t cloud1_len = cloud1->data.size();
    size_t cloud2_len = transformed_cloud.data.size();

    // set the parameters of output cloud
    //   cloud_out->height = 1;
    cloud1->width = cloud1->width * cloud1->height +
                        transformed_cloud.width * transformed_cloud.height;
    
    cloud1->row_step = cloud1->width * cloud1->point_step;

    cloud1->is_dense =
        (!cloud1->is_dense || !transformed_cloud.is_dense ) ? false : true;

    // reserve the memory
    cloud1->data.resize(cloud1_len + cloud2_len);

    // copy cloud 2 to output cloud
    std::memcpy(&cloud1->data[cloud1_len], &transformed_cloud.data[0], cloud2_len);
    this->combining = false;

    return cloud1;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto monitor = std::make_shared<PointCloud2_Combiner>();
  exec->add_node(monitor);

  exec->spin();

  rclcpp::shutdown();

  return 0;
}