#include "sensor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <discovery.hpp>

int main(int argc, char ** argv)
{
  static ToFDiscovery discovery_helper_;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToFSensor>(discovery_helper_));
  rclcpp::shutdown();
  return 0;
}
