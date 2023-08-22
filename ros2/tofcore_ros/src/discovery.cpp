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
  device_info_list = tofcore::find_all_devices();
  std::unique_ptr<tofcore::Sensor> interface_;
  std::string usb_str = "/dev/ttyACM";
  for (auto &device_info : device_info_list)
  {
    struct SensorConnectionInfo tempSensor;
    tempSensor.uri = device_info.connector_uri;
    interface_.reset(new tofcore::Sensor(1, tempSensor.uri));
    // Get sensor info
    TofComm::versionData_t versionData{};
    interface_->getSensorInfo(versionData);

    std::optional<std::string> init_name = interface_->getSensorName();
    std::optional<std::string> init_location = interface_->getSensorLocation();
    if (init_name)
      tempSensor.name = *init_name;
    else
      continue;
    std::cout << "Sensor Location: "<< *init_location <<std::endl;

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
      tempSensor.if_addr = boost::asio::ip::address::from_string("127.0.0.1", ec);
      tempSensor.if_index = 0;
    }
    sensor_list.push_back(tempSensor);
  }
}

std::optional<SensorConnectionInfo> ToFDiscovery::find_device_name(std::string name)
{
  for (auto &sensor : this->sensor_list)
  {
    if (sensor.name == name)
      return sensor;
  }
  std::cout << "Device with matching name not found :(" << std::endl;

  return {};
}
std::optional<SensorConnectionInfo> ToFDiscovery::find_device_location(std::string location)
{
  for (auto &sensor : this->sensor_list)
  {
    if (sensor.location == location)
      return sensor;
  }
  std::cout << "Device with matching location not found :(" << std::endl;
  return {};
}