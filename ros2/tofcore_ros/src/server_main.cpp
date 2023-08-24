#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <discovery.hpp>

int main(int argc, char ** argv)
{

  std::shared_ptr<ToFDiscovery> discovery_helper =  std::make_shared<ToFDiscovery>() ;
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("discovery_server", "truesense");

  rclcpp::Service<tofcore_discovery::srv::DiscoveryRequest>::SharedPtr service =
    node->create_service<tofcore_discovery::srv::DiscoveryRequest>("discovery_request", std::bind(&ToFDiscovery::find_device_location, discovery_helper, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to discover sensors.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
