#ifndef __ROS2_DISCOVERY_H__
#define __ROS2_DISCOVERY_H__

#include <cstdio>

#include <tofcore/tof_sensor.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

#include <tofcore_discovery/srv/discovery_request.hpp>

struct SensorConnectionInfo
{
  boost::asio::ip::address if_addr;
  int32_t if_index;
  std::string name = "null";
  std::string location;
  std::string desc;
  std::string uri;
  bool usb_conn;
};
/// ToFDiscovery class for finding connected PreAct ToF sensor/camera
class ToFDiscovery
{
private:
  std::vector<tofcore::device_info_t> device_info_list;
  std::vector<SensorConnectionInfo> sensor_list;
  bool is_init = false;
  

public:
  /// Standard constructor
  ToFDiscovery();

private:

  void discover(std::vector<tofcore::device_info_t> &device_info_list, std::vector<SensorConnectionInfo> &sensor_list);

public:
  void find_device_name(const std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Request> request,
          std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Response> response);
  void find_device_location(const std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Request> request,
          std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Response> response);
};

#endif