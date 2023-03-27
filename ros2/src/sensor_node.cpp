#include "sensor_node.hpp"

#include <tofcore/tof_sensor.hpp>
#include <tofcore/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;
using namespace std::string_literals;

constexpr double SENSOR_PIXEL_SIZE_MM = 0.02; // camera sensor pixel size 20x20 um
constexpr int m_width = 320;
constexpr int HEIGHT = 240;
constexpr int LENS_CENTER_OFFSET_X = 0;
constexpr int LENS_CENTER_OFFSET_Y = 0;
constexpr int32_t MIN_INTEGRATION_TIME = 0;
constexpr int32_t MAX_INTEGRATION_TIME = 4000;

constexpr auto PARAM_STREAM_TYPE = "stream_type";
constexpr auto PARAM_INTEGRATION_TIME0 = "integration_time0";
constexpr auto PARAM_INTEGRATION_TIME1 = "integration_time1";
constexpr auto PARAM_INTEGRATION_TIME2 = "integration_time2";
constexpr auto PARAM_HDR_MODE = "hdr_mode";
constexpr auto PARAM_STREAMING = "streaming";
constexpr auto PARAM_LENS_TYPE = "lens_type";
constexpr auto PARAM_MODULATION_FREQ = "modulation_frequency";
constexpr auto PARAM_DISTANCE_OFFSET = "distance_offset";

/// Quick helper function that return true if the string haystack starts with the string needle 
bool begins_with(const std::string& needle, const std::string& haystack ) 
{
  return haystack.rfind(needle, 0) == 0;
}


ToFSensor::ToFSensor()
    : Node("tof_sensor", "tofcore")
{
  rclcpp::QoS pub_qos(10);
  pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  // Setup topic pulbishers
  pub_ambient_ = this->create_publisher<sensor_msgs::msg::Image>("ambient", pub_qos);
  pub_distance_ = this->create_publisher<sensor_msgs::msg::Image>("distance", pub_qos);
  pub_amplitude_ = this->create_publisher<sensor_msgs::msg::Image>("amplitude", pub_qos);
  pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", pub_qos);
  for(size_t i = 0; i != pub_dcs_.size(); i++) {
    std::string topic {"dcs"};
    topic += std::to_string(i);
    pub_dcs_[i] = this->create_publisher<sensor_msgs::msg::Image>(topic, pub_qos);
  }

  // connect to interface
  interface_.reset( new tofcore::Sensor(1, "/dev/ttyACM1"));
  (void)interface_->subscribeCameraInfo([&](std::shared_ptr<tofcore::CameraInfo> ci) -> void
                                       { (void)ci; /*updateCameraInfo(ci);*/ });
  (void)interface_->subscribeFrame([&](std::shared_ptr<tofcore::Frame> f) -> void
                                  { updateFrame(*f); });


  // Setup ROS parameters
  this->declare_parameter(PARAM_STREAM_TYPE, "distance_amplitude");
  this->declare_parameter(PARAM_INTEGRATION_TIME0, 100);
  this->declare_parameter(PARAM_INTEGRATION_TIME1, 0);
  this->declare_parameter(PARAM_INTEGRATION_TIME2, 0);
  this->declare_parameter(PARAM_HDR_MODE, "off");
  this->declare_parameter(PARAM_STREAMING, true);
  this->declare_parameter(PARAM_LENS_TYPE, "wf");
  this->declare_parameter(PARAM_MODULATION_FREQ, "12");
  this->declare_parameter(PARAM_DISTANCE_OFFSET, 0);

  // Setup a callback so that we can react to parameter changes from the outside world.
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ToFSensor::on_set_parameters_callback, this, std::placeholders::_1));

  // Update all parameters
  auto params = this->get_parameters(this->list_parameters({}, 1).names);
  this->on_set_parameters_callback(params);
}

rcl_interfaces::msg::SetParametersResult ToFSensor::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  // assume success, if any parameter set below fails this will be changed
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &parameter : parameters)
  {
    auto param_name = parameter.get_name();
    if (param_name == PARAM_STREAM_TYPE)
    {
      bool streaming = true;
      this->get_parameter(PARAM_STREAMING, streaming);
      if (streaming)
      {
        this->apply_stream_type_param(parameter, result);
      }
    }
    else if (begins_with("integration_time", param_name))
    {
      this->apply_integration_time_param(parameter, result);
    }
    else if( param_name == PARAM_HDR_MODE)
    {
      this->apply_hdr_mode_param(parameter, result);
    }
    else if( param_name == PARAM_STREAMING)
    {
      this->apply_streaming_param(parameter, result);
    }
    else if( param_name == PARAM_LENS_TYPE)
    {
      this->apply_lens_type_param(parameter, result);
    }
    else if( param_name == PARAM_MODULATION_FREQ)
    {
      this->apply_modulation_frequency_param(parameter, result);
    }
    else if( param_name == PARAM_DISTANCE_OFFSET)
    {
      this->apply_distance_offset_param(parameter, result);

    }
  }
  return result;
}


void ToFSensor::apply_stream_type_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : \"%s\"", parameter.get_name().c_str(), value.c_str());
  if (value == "distance")
  {
    interface_->streamDistance();
  }
  else if (value == "grayscale")
  {
    interface_->streamGrayscale();
  }
  else if (value == "distance_amplitude")
  {
    interface_->streamDistanceAmplitude();
  }
  else if (value == "dcs")
  {
    interface_->streamDCS();
  }
  else if (value == "dcs_ambient")
  {
    interface_->streamDCSAmbient();
  }
  else
  {
    result.successful = false;
    result.reason = "Unknown stream type: "s + value;
    RCLCPP_ERROR(this->get_logger(), result.reason.c_str());
  }
}


void ToFSensor::apply_integration_time_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result) 
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %li", parameter.get_name().c_str(), value);
  if (value < MIN_INTEGRATION_TIME || value > MAX_INTEGRATION_TIME) 
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  } 
  else 
  {
    uint16_t int_times[3] = {0};
    int_times[0] = (parameter.get_name() == PARAM_INTEGRATION_TIME0) ? value : get_parameter(PARAM_INTEGRATION_TIME0).as_int();
    int_times[1] = (parameter.get_name() == PARAM_INTEGRATION_TIME1) ? value : get_parameter(PARAM_INTEGRATION_TIME1).as_int();
    int_times[2] = (parameter.get_name() == PARAM_INTEGRATION_TIME2) ? value : get_parameter(PARAM_INTEGRATION_TIME1).as_int();
    interface_->setIntegrationTime(int_times[0], int_times[1], int_times[2], 500/*hard code greyscale int time for now*/);
  }
}


void ToFSensor::apply_hdr_mode_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result) 
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());
  if(begins_with("s", value)) //spacital 
  {
    interface_->setHDRMode(1);
  }
  else if(begins_with("t", value)) //temporal
  {
    interface_->setHDRMode(2);
  }
  else if(begins_with("o", value)) //off
  {
    interface_->setHDRMode(0);
  }
  else
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  }
}



void ToFSensor::apply_modulation_frequency_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result) 
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());

  int mod_freq_index = 0;
  if(begins_with("12", value))
  {
    mod_freq_index = 0;
  }
  else if(begins_with("24", value))
  {
    mod_freq_index = 1;
  }
  else if(begins_with("6", value))
  {
    mod_freq_index = 2;
  }
  else if(begins_with("3", value))
  {
    mod_freq_index = 3;
  }
  else if(begins_with("1.5", value))
  {
    mod_freq_index = 4;
  }
  else if(begins_with("0.75", value))
  {
    mod_freq_index = 5;
  }
  else 
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
    return;
  }
  interface_->setModulation(mod_freq_index, 0);
}

void ToFSensor::apply_streaming_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result) 
{
  try {
    auto value = parameter.as_bool();
    RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ?"true":"false"));
    if(value) {
      rclcpp::Parameter stream_type;
      rcl_interfaces::msg::SetParametersResult dummy_result;
      (void)this->get_parameter(PARAM_STREAM_TYPE, stream_type);
      this->apply_stream_type_param(stream_type, dummy_result);
    } else {
      interface_->stopStream();
    }
  }
  catch(std::exception& e) {
    result.successful = false;
    result.reason = e.what();
  }
}


void ToFSensor::apply_lens_type_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());

  //0 - wide field, 1 - standard field, 2 - narrow field
  if(begins_with("w", value)) //wide FOV 
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 0);
  }
  else if(begins_with("s", value)) //standard fov
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 1);
  }
  else if(begins_with("n", value)) //narrow fov
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 2);
  }
  else if(begins_with("r", value))
  {
    std::vector<double> rays_x, rays_y, rays_z;
    interface_->getLensInfo(rays_x, rays_y, rays_z);
    cartesianTransform_.initLensTransform(m_width, HEIGHT, rays_x, rays_y, rays_z);
  }
  else
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  }

}


void ToFSensor::apply_distance_offset_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult&)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);

  interface_->setOffset(value);
}



void ToFSensor::publish_amplData(const tofcore::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base_link";
  img.height = static_cast<uint32_t>(frame.m_height);
  img.width = static_cast<uint32_t>(frame.m_width);
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.m_px_size;
  img.is_bigendian = 0;
  img.data = frame.m_amplData;
  pub.publish(img);
}

void ToFSensor::publish_distData(const tofcore::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base_link";
  img.height = static_cast<uint32_t>(frame.m_height);
  img.width = static_cast<uint32_t>(frame.m_width);
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.m_px_size;
  img.is_bigendian = 1;
  img.data = frame.m_distData;
  pub.publish(img);
}

void ToFSensor::publish_pointCloud(const tofcore::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 cloud_msg{};
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.resize(frame.m_height * frame.m_width);
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "amplitude", 1, sensor_msgs::msg::PointField::UINT16,
      "ambient", 1, sensor_msgs::msg::PointField::INT16,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "distance", 1, sensor_msgs::msg::PointField::UINT16);

  // Note: For some reason setPointCloudFields doesn't set row_step
  //      and resets msg height and m_width so setup them here.
  cloud_msg.height = static_cast<uint32_t>(frame.m_height);
  cloud_msg.width = static_cast<uint32_t>(frame.m_width);
  cloud_msg.row_step = frame.m_width * 19; // 19 is the size in bytes of all the point cloud fields

  sensor_msgs::PointCloud2Iterator<float> it_x{cloud_msg, "x"};
  sensor_msgs::PointCloud2Iterator<float> it_y{cloud_msg, "y"};
  sensor_msgs::PointCloud2Iterator<float> it_z{cloud_msg, "z"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude{cloud_msg, "amplitude"};
  sensor_msgs::PointCloud2Iterator<int16_t> it_ambient{cloud_msg, "ambient"};
  sensor_msgs::PointCloud2Iterator<uint8_t> it_valid{cloud_msg, "valid"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_phase{cloud_msg, "distance"};

  auto it_d = frame.m_distData.begin();
  auto it_a = frame.m_amplData.begin();
  uint32_t count = 0;
  while (it_d != frame.m_distData.end())
  {
    auto distance = (*(it_d + 1) << 8) + (*it_d);
    auto y = count / frame.m_width;
    auto x = count % frame.m_width;
    int valid = 0;
    double px, py, pz;
    px = py = pz = 0.1;
    if (distance > 0 && distance < 64000)
    {
      cartesianTransform_.transformPixel(x, y, distance, px, py, pz);
      px /= 1000.0; // mm -> m
      py /= 1000.0; // mm -> m
      pz /= 1000.0; // mm -> m
      valid = 1;
    }

    *it_x = px;
    *it_y = py;
    *it_z = pz;
    if (frame.m_dataType == tofcore::Frame::AMPLITUDE)
    {
      *it_amplitude = (*(it_a + 1) << 8) + (*it_a);
      it_a += 2;
    }
    else
    {
      *it_amplitude = pz;
    }
    *it_ambient = 0;
    *it_phase = distance;
    *it_valid = valid;

    ++it_x;
    ++it_y;
    ++it_z;
    ++it_amplitude;
    ++it_ambient;
    ++it_phase;
    ++it_valid;
    ++count;
    it_d += 2;
  }

  pub.publish(cloud_msg);
}


void ToFSensor::publish_DCSData(const tofcore::Frame &frame, const rclcpp::Time& stamp)
{

  //TODO Need to figure out the best way to publish image meta-data including:
  //  temperature
  //  modulation_frequency
  //  integration_time
  //  binning
  //  vled_mv
  //  chip_id
  //
  // Also need to figure out how to publish an ambient frame which will be required for use with the calibration app.

  if(frame.m_dataType == tofcore::Frame::DCS) {
    for(auto i = 0; i != 4; ++i) {
        sensor_msgs::msg::Image img;
        img.header.stamp = stamp;
        img.header.frame_id = "base_link";
        img.height = static_cast<uint32_t>(frame.m_height);
        img.width = static_cast<uint32_t>(frame.m_width);
        img.encoding = sensor_msgs::image_encodings::MONO16;
        img.step = img.width * frame.m_px_size;
        img.is_bigendian = 0;
        auto frame_size = img.step * img.height;
        img.data.resize(frame_size);
        auto begin = frame.m_dcsData.begin() + (i*frame_size);
        auto end = begin + frame_size;
        std::copy(begin, end, img.data.begin());
        pub_dcs_[i]->publish(img);
    }
  }
}


void ToFSensor::updateFrame(const tofcore::Frame &frame)
{
  auto stamp = this->now();
  switch (frame.m_dataType)
  {
  case tofcore::Frame::GRAYSCALE:
  {
    publish_amplData(frame, *pub_ambient_, stamp);
    break;
  }
  case tofcore::Frame::AMPLITUDE:
  {
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, stamp);
    break;
  }
  case tofcore::Frame::DISTANCE:
  {
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, stamp);
    break;
  }
  case tofcore::Frame::DCS:
  {
    publish_DCSData(frame, stamp);
    break;
  }
  }
}