#include "discovery.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;

ToFDiscovery::ToFDiscovery()

{
  this->discover(this->device_info_list, this->sensor_list);
}

/// Publish received temperature data in frame to the to four different temperature topics (pub_temps_) with timestamp stamp.
void ToFDiscovery::discover(std::vector<tofcore::device_info_t> &device_info_list, std::vector<SensorConnectionInfo> &sensor_list)
{
  this->device_info_list = tofcore::find_all_devices();
  std::unique_ptr<tofcore::Sensor> interface_;
  std::string usb_str = "/dev/ttyACM";
  TofComm::versionData_t versionData{};

  for (auto &device_info : device_info_list)
  {
    struct SensorConnectionInfo tempSensor;
    tempSensor.uri = device_info.connector_uri;
    interface_.reset(new tofcore::Sensor(1, tempSensor.uri));
    // Get sensor info
    interface_->getSensorInfo(versionData);

    std::optional<std::string> init_name = interface_->getSensorName();
    std::optional<std::string> init_location = interface_->getSensorLocation();
    if (init_name)
      tempSensor.name = *init_name;
    else
      continue;

    if (init_location)
      tempSensor.location = *init_location;
    else
      continue;

    if (tempSensor.uri.find(usb_str) != std::string::npos)
    {
      tempSensor.usb_conn = true;
    }
    else
    {
      tempSensor.usb_conn = false;
      // TODO: find out how to populate these eth params
      boost::system::error_code ec;
      tempSensor.if_addr = "127.0.0.1";//boost::asio::ip::address::from_string("127.0.0.1", ec);
      tempSensor.if_index = 0;
    }
    sensor_list.push_back(tempSensor);
  }
}
void ToFDiscovery::find_device_name(const std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Request> request,
                                    std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Response> response)
{
  for (auto &sensor : this->sensor_list)
  {
    if (sensor.name == request->name)
    {
      response->if_addr = sensor.if_addr;
      response->if_index = sensor.if_index;
      response->name = sensor.name;
      response->location = sensor.location;
      response->desc = sensor.desc;
      response->uri = sensor.uri;
      response->usb_conn = sensor.usb_conn;
      return;
    }
    return;
  }
}
void ToFDiscovery::find_device_location(const std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Request> request,
                                        std::shared_ptr<tofcore_discovery::srv::DiscoveryRequest::Response> response)
{

  for (auto &sensor : this->sensor_list)
  {

    if (sensor.location == request->location)
    {
      response->if_addr = sensor.if_addr;
      response->if_index = sensor.if_index;
      response->name = sensor.name;
      response->location = sensor.location;
      response->desc = sensor.desc;
      response->uri = sensor.uri;
      response->usb_conn = sensor.usb_conn;
      return;
    }
  }
  return;
}