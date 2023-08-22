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

struct SensorConnectionInfo
{
  boost::asio::ip::address if_addr;
  int32_t if_index;
  std::string name;
  std::string location;
  std::string desc;
  std::string uri;
  bool usb_conn;
};
/// ToFDiscovery class for finding connected PreAct ToF sensor/camera
class ToFDiscovery 
{
private:
  //std::shared_ptr<std::vector<tofcore::device_info_t>> device_info_list;
  //std::shared_ptr<std::vector<SensorConnectionInfo>> sensor_list;

  //std::shared_ptr<std::vector<tofcore::device_info_t>> device_info_list = std::make_shared<std::vector<tofcore::device_info_t>>();
  std::shared_ptr<std::vector<SensorConnectionInfo>> sensor_list = std::make_shared<std::vector<SensorConnectionInfo>>();
  std::shared_ptr<bool> is_init=std::make_shared<bool>(false);

public:
  /// Standard constructor
  ToFDiscovery();

private:

  void discover(std::shared_ptr<std::vector<SensorConnectionInfo>> &sensor_list);

public:
  std::optional<SensorConnectionInfo> find_device_name(std::string name);
  std::optional<SensorConnectionInfo>  find_device_location(std::string location);
};

#endif