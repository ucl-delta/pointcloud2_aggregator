#include "aggregator.hpp"

using namespace std::chrono_literals;

Pointcloud2_Aggregator:: Pointcloud2_Aggregator(): 
    Node("pointcloud2_aggregator"), 
    publish_frequency(10)
{
    // Declare Parameters
    this->declare_parameter("publish_frequency", 5.0);
    this->declare_parameter("time_window", 1.0);
    this->declare_parameter("pointcloud_topic_freq", 10.0);
    this->declare_parameter("pointcloud_topic", "/livox/lidar");

    // Set Frequency
    this->publish_frequency = this->get_parameter("publish_frequency").as_double();

    // Set Moving Window
    double time_window = this->get_parameter("time_window").as_double();
    this->time_window = std::make_shared<rclcpp::Duration>(std::chrono::duration<float>(time_window));

    // Set ring buffer size by estimating the number of messages to be received per time window
    double topic_freq = this->get_parameter("pointcloud_topic_freq").as_double();
    if (topic_freq < 1e-5) {
        RCLCPP_ERROR(this->get_logger(), "Error topic frequency parameter set to zero");
        throw std::runtime_error("Error topic frequency parameter set to zero");
    }
    double topic_rate = 1.0 / topic_freq;
    int messages_per_time_window = std::ceil(time_window / topic_rate);
    this->buffer = std::make_shared<ring<sensor_msgs::msg::PointCloud2::SharedPtr>>(messages_per_time_window);

    
    

    // Create Subscriber
    std::string topic_name = this->get_parameter("pointcloud_topic").as_string();
    this->pointcloud2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_name, 5, std::bind(&Pointcloud2_Aggregator::pointcloud2_sub_callback, this, std::placeholders::_1));

    // Create Publisher
    std::string topic_name_agg = this->get_parameter("pointcloud_topic").as_string();
    topic_name_agg.append("_aggregated");
    this->aggregator_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_agg, this->publish_frequency);

    // Create Timer
    this->aggregated_publish_timer = this->create_wall_timer(
                std::chrono::duration<float>(1.0/this->publish_frequency), std::bind(&Pointcloud2_Aggregator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Pointcloud2 Aggregator Initialised for %s", topic_name.c_str());
};

void Pointcloud2_Aggregator::pointcloud2_sub_callback(const sensor_msgs::msg::PointCloud2& msg) 
{
    // Copy the incoming message and make an entry
    auto shared_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(msg); 

    if(!this->pointcloud_template) {
        this->pointcloud_template = shared_msg;
    } else {
        // Verify same type (probably should check pointfield too...)
        if (
          this->pointcloud_template-> height != shared_msg->height || 
          this->pointcloud_template->point_step != shared_msg->point_step  
        ) {
            RCLCPP_ERROR(this->get_logger(), "Subsequent pointcloud msgs inconsistent: height %i!=%i or point_step %i!=%i", 
                this->pointcloud_template->height, shared_msg->height,
                this->pointcloud_template->point_step, shared_msg->point_step);
        } 
    }

    // Save the latest header
    this->latest_header = std::make_shared<std_msgs::msg::Header>(msg.header);

    // Insert the entry into the ring buffer 
    this->buffer->push(shared_msg);
}

void Pointcloud2_Aggregator::timer_callback() 
{
    auto pc = sensor_msgs::msg::PointCloud2();
    if(!this->pointcloud_template) {
        RCLCPP_INFO(this->get_logger(), "No Pointclouds Receied Yet");
    } else {
        pc.header = *(this->latest_header);
        pc.height = this->pointcloud_template->height;
        pc.width = 0;
        pc.fields = this->pointcloud_template->fields; // (Probably need to manual copy)
        pc.is_bigendian = this->pointcloud_template->is_bigendian;
        pc.point_step = this->pointcloud_template->point_step;
        pc.is_dense = this->pointcloud_template->is_dense;
        pc.row_step = 0;
    }

    if(this->buffer->size() == 0) {
        return;
    }

    for (size_t i = 0; i < this->buffer->size(); ++i) {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud = this->buffer->at(i);
        pc.width += cloud->width;
        pc.row_step += cloud->row_step;
    }

    // RCLCPP_INFO(this->get_logger(), "Total Cloud Width: %i, Total Row Step: %i", pc.width, pc.row_step);

    int copy_offset = 0;
    uint8_t data[(int)pc.row_step * (int)pc.height];
    for (size_t i = 0; i < this->buffer->size(); ++i) {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud = this->buffer->at(i);
        // RCLCPP_INFO(this->get_logger(), "COPY from %p to %p, size: %i", data+copy_offset, cloud->data.data(), cloud->row_step);
        memcpy(data+copy_offset, cloud->data.data(), cloud->row_step);
        copy_offset += cloud->point_step;
    }
    std::vector<uint8_t> datavec(data, data + pc.row_step);
    pc.data = datavec;

    this->aggregator_pub->publish(pc);
}