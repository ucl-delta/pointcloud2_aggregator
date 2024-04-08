#include "aggregator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto aggregator = std::make_shared<pointcloud2::Pointcloud2_Aggregator>(options);
  exec.add_node(aggregator);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}