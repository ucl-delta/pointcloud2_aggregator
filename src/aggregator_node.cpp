#include "aggregator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pointcloud2_Aggregator>());
  rclcpp::shutdown();
  return 0;
}