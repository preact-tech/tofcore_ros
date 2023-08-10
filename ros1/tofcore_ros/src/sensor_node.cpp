#include "sensor_node.hpp"

#include <tofcore/tof_sensor.hpp>
#include <tofcore/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>




using namespace std::chrono_literals;
using namespace std::string_literals;


constexpr double SENSOR_PIXEL_SIZE_MM = 0.02; // camera sensor pixel size 20x20 um
constexpr int m_width = 320;
constexpr int HEIGHT = 240;
constexpr int LENS_CENTER_OFFSET_X = 0;
constexpr int LENS_CENTER_OFFSET_Y = 0;
constexpr int32_t MIN_INTEGRATION_TIME = 0;
constexpr int32_t MAX_INTEGRATION_TIME = 4000;


//Read Only params
constexpr auto API_VERSION  = "api_version";
constexpr auto CHIP_ID  = "chip_id";
constexpr auto SENSOR_NAME  = "sensor_name";
constexpr auto SW_VERSION = "sw_version";
constexpr auto SENSOR_URL  = "sensor_url"; 



//Configurable params
constexpr auto CAPTURE_MODE = "capture_mode";
constexpr auto INTEGRATION_TIME = "integration_time";
constexpr auto STREAMING_STATE = "streaming";
constexpr auto MODULATION_FREQUENCY = "modulation_frequency";
constexpr auto DISTANCE_OFFSET = "distance_offset";
constexpr auto MINIMUM_AMPLITUDE = "minimum_amplitude";
constexpr auto FLIP_HORIZONTAL= "flip_hotizontal";
constexpr auto FLIP_VERITCAL = "flip_vertical";
constexpr auto BINNING  = "binning"; 


/// Quick helper function that return true if the string haystack starts with the string needle
bool begins_with(const std::string& needle, const std::string& haystack )
{
  return haystack.rfind(needle, 0) == 0;
}


ToFSensor::ToFSensor()
  //  : Node("tof_sensor", "tofcore")
{

  int pub_qos = 100;
  //pub_qos.reliability(ros::ReliabilityPolicy::Reliable);
  //pub_qos.durability(ros::DurabilityPolicy::TransientLocal);

  // Setup topic pulbishers
  *pub_ambient_ = n.advertise<sensor_msgs::Image>("ambient", pub_qos);
  *pub_distance_ = n.advertise<sensor_msgs::Image>("depth", pub_qos);//renamed this from distance to depth to match truesense node
  *pub_amplitude_ = n.advertise<sensor_msgs::Image>("amplitude", pub_qos);
  *pub_pcd_ = n.advertise<sensor_msgs::PointCloud2>("points", pub_qos);
  *pub_cust_pcd_ = n.advertise<sensor_msgs::PointCloud2>("points", pub_qos);
  //*pub_cust_pcd_ = n.advertise<msg::TofcorePointCloud2>("cust_points", pub_qos);

  for(size_t i = 0; i != pub_dcs_.size(); i++) {
    std::string topic {"dcs"};
    topic += std::to_string(i);
    *pub_dcs_[i] = n.advertise<sensor_msgs::Image>(topic, pub_qos);
  }

  *sensor_temperature_tl = n.advertise<sensor_msgs::Temperature>("sensor_temperature_tl", pub_qos);
  *sensor_temperature_tr = n.advertise<sensor_msgs::Temperature>("sensor_temperature_tr", pub_qos);
  *sensor_temperature_bl = n.advertise<sensor_msgs::Temperature>("sensor_temperature_bl", pub_qos);
  *sensor_temperature_br = n.advertise<sensor_msgs::Temperature>("sensor_temperature_br", pub_qos);

  interface_.reset( new tofcore::Sensor(1, "/dev/ttyACM0"));
  (void)interface_->subscribeMeasurement([&](std::shared_ptr<tofcore::Measurement_T> f) -> void
                                         { updateFrame(*f); });


  //Get sensor info
  TofComm::versionData_t versionData { };
  interface_->getSensorInfo(versionData);

  //use generic lens transform always
  std::vector<double> rays_x, rays_y, rays_z;
  interface_->getLensInfo(rays_x, rays_y, rays_z);
  cartesianTransform_.initLensTransform(m_width, HEIGHT, rays_x, rays_y, rays_z);
  
  // Setup ROS parameters
  //rcl_interfaces::msg::ParameterDescriptor readonly_descriptor;
  //readonly_descriptor.read_only = true;

  //Read Only params
  this->n.setParam(API_VERSION, versionData.m_softwareSourceID,readonly_descriptor);// TODO: Is this the right param
  this->n.setParam(CHIP_ID, std::to_string(versionData.m_sensorChipId),readonly_descriptor);
  this->n.setParam(SENSOR_NAME, versionData.m_modelName,readonly_descriptor);
  this->n.setParam(SW_VERSION, versionData.m_softwareVersion,readonly_descriptor);
  this->n.setParam(SENSOR_URL,  "/dev/ttyACM0",readonly_descriptor); // TODO: How can I get this from sensor?
  

  //Configurable params
  this->n.setParam(CAPTURE_MODE, "distance_amplitude");
  //TODO: Do I need to do this?
  // if(interface_->getIntegrationTimes())
  //   this->n.setParam(INTEGRATION_TIME, interface_->getIntegrationTimes()->at(0));
  // else
    this->n.setParam(INTEGRATION_TIME, 500);
  this->n.setParam(STREAMING_STATE, true);
  this->n.setParam(MODULATION_FREQUENCY, "12");
  this->n.setParam(DISTANCE_OFFSET, 0);
  this->n.setParam(MINIMUM_AMPLITUDE, 0);
  this->n.setParam(FLIP_HORIZONTAL, false);
  this->n.setParam(FLIP_VERITCAL, false);
  this->n.setParam(BINNING, false);

 

  // Setup a callback so that we can react to parameter changes from the outside world.
  //parameters_callback_handle_ = this->add_on_set_parameters_callback(
  //    std::bind(&ToFSensor::on_set_parameters_callback, this, std::placeholders::_1));

  // Update all parameters
  auto params = this->n.getParamNames(this->list_parameters({}, 1).names);
  this->on_set_parameters_callback(params);
}

void ToFSensor::on_set_parameters_callback(
    const std::vector<std::string> &parameters)
{
  // assume success, if any parameter set below fails this will be changed
  //rcl_interfaces::msg::SetParametersResult result;
  //result.successful = true;
  //result.reason = "success";

  for (const auto &parameter : parameters)
  {
    auto name = parameter;
    if (name == CAPTURE_MODE)
    {
      bool streaming = true;
      this->n.getParam(STREAMING_STATE, streaming);
      if (streaming)
      {
        this->apply_stream_type_param(parameter);
      }
    }
    else if (begins_with("integration_time", name))
    {
      this->apply_integration_time_param(parameter);
    }
    else if( name == STREAMING_STATE)
    {
      this->apply_streaming_param(parameter);
    }
    else if( name == MODULATION_FREQUENCY)
    {
      this->apply_modulation_frequency_param(parameter);
    }
    else if( name == DISTANCE_OFFSET)
    {
      this->apply_distance_offset_param(parameter);
    }
    else if( name == MINIMUM_AMPLITUDE)
    {
      this->apply_minimum_amplitude_param(parameter);
    }
    else if( name == FLIP_HORIZONTAL)
    {
      this->apply_flip_horizontal_param(parameter);

    }
    else if( name == FLIP_VERITCAL)
    {
      this->apply_flip_vertical_param(parameter);
    }
        else if( name == BINNING)
    {
      this->apply_binning_param(parameter);
    }
  }
  //return result;
}


void ToFSensor::apply_stream_type_param(const std::string& parameter)
{
  std::string value;
  this->n.getParam(parameter,value);
  ROS_INFO( "Handling parameter \"%s\" : \"%s\"", parameter.c_str(), value.c_str());
  if (value == "distance")
  {
    interface_->streamDistance();
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
    RCLCPP_ERROR( result.reason.c_str());
  }
}


void ToFSensor::apply_integration_time_param(const std::string& parameter)
{
  int value;
  this->n.getParam(parameter,value);
  ROS_INFO( "Handling parameter \"%s\" : %li", parameter.c_str(), value);
  if (value < MIN_INTEGRATION_TIME || value > MAX_INTEGRATION_TIME)
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  }
  else
  {
    uint16_t int_times = 0;
    int_times = (parameter.get_name() == INTEGRATION_TIME) ? value : get_parameter(INTEGRATION_TIME).as_int();
    interface_->setIntegrationTime(int_times);
  }
}


void ToFSensor::apply_hdr_mode_param(const std::string& parameter)
{
  std::string value;
  this->n.getParam(parameter,value);
  ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), value.c_str());
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



void ToFSensor::apply_modulation_frequency_param(const std::string& parameter)
{
  std::string value;
  this->n.getParam(parameter,value);
  
  ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), value.c_str());

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

void ToFSensor::apply_streaming_param(const std::string& parameter)
{
  try {
  bool value;
  this->n.getParam(parameter,value);
  
      ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), (value ?"true":"false"));
    if(value) {
      std::string stream_type;
    //  rcl_interfaces::msg::SetParametersResult dummy_result;
      (void)this->n.getParam(CAPTURE_MODE, stream_type);
      this->apply_stream_type_param(stream_type);
    } else {
      interface_->stopStream();
    }
  }
  catch(std::exception& e) {
    result.successful = false;
    result.reason = e.what();
  }
}


void ToFSensor::apply_lens_type_param(const std::string& parameter)
{
  std::string value;
  this->n.getParam(parameter,value);
    ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), value.c_str());

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
    //result.successful = false;
    //result.reason = parameter.get_name() + " value is out of range";
  }

}


void ToFSensor::apply_distance_offset_param(const std::string& parameter)
{
  int value;
  this->n.getParam(parameter,value);
    ROS_INFO( "Handling parameter \"%s\" : %ld", parameter.c_str(), value);

  interface_->setOffset(value);
}

void ToFSensor::apply_minimum_amplitude_param(const std::string& parameter)
{
  int value;
  this->n.getParam(parameter,value);
    ROS_INFO( "Handling parameter \"%s\" : %ld", parameter.c_str(), value);

  interface_->setMinAmplitude(value);
}
void ToFSensor::apply_flip_horizontal_param(const std::string& parameter)
{
   bool value;
  this->n.getParam(parameter,value);
  
      ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), (value ?"true":"false"));
  //TODO:  It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipHorizontally(value);
  std::string stream_type;
 // rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->n.getParam(CAPTURE_MODE, stream_type);
  this->apply_stream_type_param(stream_type);
}
void ToFSensor::apply_flip_vertical_param(const std::string& parameter)
{
  bool value;
  this->n.getParam(parameter,value);
  
      ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), (value ?"true":"false"));
  //TODO: It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipVertically(value);
  std::string stream_type;
  //rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->n.getParam(CAPTURE_MODE, stream_type);
  this->apply_stream_type_param(stream_type);
}
void ToFSensor::apply_binning_param(const std::string& parameter)
{
   bool value;
  this->n.getParam(parameter,value);
  
      ROS_INFO( "Handling parameter \"%s\" : %s", parameter.c_str(), (value ?"true":"false"));

  interface_->setBinning(value,value);

}
void ToFSensor::publish_tempData(const tofcore::Measurement_T &frame, const ros::Time& stamp)
{
  const std::array<float, 4> defaultTemps {0.0,0.0,0.0,0.0};
  auto temperatures = frame.sensor_temperatures().value_or(defaultTemps);
  int count = 0;

  for(const auto& i : temperatures) {
    sensor_msgs::Temperature tmp;
    tmp.header.stamp = stamp;
    tmp.header.frame_id = "base_link";
    tmp.temperature = i;
    tmp.variance = 0;
    switch (count) {
    case 0:
    {
      sensor_temperature_tl->publish(tmp);
      break;
    }
    case 1:
    {
      sensor_temperature_tr->publish(tmp);
      break;
    }
    case 2:
    {
      sensor_temperature_bl->publish(tmp);
      break;
    }
    case 3:
    {
      sensor_temperature_br->publish(tmp);
      break;
    }
    }
    count++;
  }
}

void ToFSensor::publish_amplData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time& stamp)
{
  sensor_msgs::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base_link";
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 0;
  auto amplitude_bv = frame.amplitude();
  img.data.resize(amplitude_bv.size() * sizeof(amplitude_bv.data()[0]));
  uint8_t* amplitude_begin = (uint8_t*)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);

}

//TODO: I thought we couldnt do ambient, or can we not do ambient and distance/amp at the same time? understand this?
void ToFSensor::publish_ambientData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time& stamp)
{
  sensor_msgs::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base_link";
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 0;
  auto amplitude_bv = frame.ambient();
  img.data.resize(amplitude_bv.size() * sizeof(amplitude_bv.data()[0]));
  uint8_t* amplitude_begin = (uint8_t*)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_distData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time& stamp)
{
  sensor_msgs::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base_link";
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 1;
  auto distance_bv = frame.distance();
  img.data.resize(distance_bv.size() * sizeof(distance_bv.data()[0]));
  uint8_t* dist_begin = (uint8_t*)distance_bv.data();
  std::copy_n(dist_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_pointCloud(const tofcore::Measurement_T &frame, ros::Publisher &pub,ros::Publisher &cust_pub, const ros::Time& stamp)
{
  // tofcore::TofcorePointCloud2 cloud_msg{};
  // cloud_msg.header.stamp = stamp;
  // cloud_msg.header.frame_id = "base_link";
  // cloud_msg.point_cloud.header.stamp = stamp;
  // cloud_msg.point_cloud.header.frame_id = "base_link";
  // cloud_msg.point_cloud.is_dense = true;
  // cloud_msg.point_cloud.is_bigendian = false;

  // //Adding this to the message for the auto exposure node
  // //Need to check if it exists because this is optional value
  // if (frame.integration_times())
  // {
  //   auto integration_times=*std::move(frame.integration_times());
  //   cloud_msg.integration_time=integration_times.at(0);
  // }

  // sensor_msgs::PointCloud2Modifier modifier(cloud_msg.point_cloud);
  // modifier.resize(frame.height() * frame.width());
  // modifier.setPointCloud2Fields(
  //     7,
  //     "x", 1, sensor_msgs::PointField::FLOAT32,
  //     "y", 1, sensor_msgs::PointField::FLOAT32,
  //     "z", 1, sensor_msgs::PointField::FLOAT32,
  //     "amplitude", 1, sensor_msgs::PointField::UINT16,
  //     "ambient", 1, sensor_msgs::PointField::INT16,
  //     "valid", 1, sensor_msgs::PointField::UINT8,
  //     "distance", 1, sensor_msgs::PointField::UINT16); //TODO: do we need phase here?

  // // Note: For some reason setPointCloudFields doesn't set row_step
  // //      and resets msg height and m_width so setup them here.
  // cloud_msg.point_cloud.height = static_cast<uint32_t>(frame.height());
  // cloud_msg.point_cloud.width = static_cast<uint32_t>(frame.width());
  // cloud_msg.point_cloud.row_step = frame.width() * 19; // 19 is the size in bytes of all the point cloud fields

  // sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg.point_cloud, "x");
  // sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg.point_cloud, "y");
  // sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg.point_cloud, "z");
  // sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude(cloud_msg.point_cloud, "amplitude");
  // sensor_msgs::PointCloud2Iterator<int16_t> it_ambient(cloud_msg.point_cloud, "ambient");
  // sensor_msgs::PointCloud2Iterator<uint8_t> it_valid(cloud_msg.point_cloud, "valid");
  // sensor_msgs::PointCloud2Iterator<uint16_t> it_phase(cloud_msg.point_cloud, "distance");

  // auto it_d = frame.distance().begin();
  // auto it_a = frame.amplitude().begin();
  // uint32_t count = 0;
  // while (it_d != frame.distance().end())
  // {
  //   auto distance = *it_d;
  //   auto y = count / frame.width();
  //   auto x = count % frame.width();
  //   int valid = 0;
  //   double px, py, pz;
  //   px = py = pz = 0.1;
  //   if (distance > 0 && distance < 64000)
  //   {
  //     cartesianTransform_.transformPixel(x, y, distance, px, py, pz);
  //     px /= 1000.0; // mm -> m
  //     py /= 1000.0; // mm -> m
  //     pz /= 1000.0; // mm -> m
  //     valid = 1;
  //   }

  //   *it_x = px;
  //   *it_y = py;
  //   *it_z = pz;
  //   if (frame.type() == tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
  //   {
  //     *it_amplitude = *it_a;
  //     it_a += 1;
  //   }
  //   else
  //   {
  //     *it_amplitude = pz;
  //   }
  //   *it_ambient = 0;
  //   *it_phase = distance;
  //   *it_valid = valid;

  //   ++it_x;
  //   ++it_y;
  //   ++it_z;
  //   ++it_amplitude;
  //   ++it_ambient;
  //   ++it_phase;
  //   ++it_valid;
  //   ++count;
  //   it_d += 1;
  // }
  // //This is dumb and redundant but I don't want to break rviz viewer
  // pub.publish(cloud_msg.point_cloud);
  // cust_pub.publish(cloud_msg);
}


void ToFSensor::publish_DCSData(const tofcore::Measurement_T &frame, const ros::Time& stamp)
{

  //TODO Need to figure out the best way to publish image meta-data including:
  //  modulation_frequency
  //  integration_time
  //  binning
  //  vled_mv
  //  chip_id
  //
  // Also need to figure out how to publish an ambient frame which will be required for use with the calibration app.
  //
  //Does feature/add-meta-data-publishers branch have work that should be used for this?

  if(frame.type() == tofcore::Measurement_T::DataType::DCS) {
    for(auto i = 0; i != 4; ++i) {
      sensor_msgs::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = "base_link";
      img.height = static_cast<uint32_t>(frame.height());
      // ROS_INFO( "Frame Height: %d", frame.height());

      img.width = static_cast<uint32_t>(frame.width());
      img.encoding = sensor_msgs::image_encodings::MONO16;
      img.step = img.width * frame.pixel_size();
      img.is_bigendian = 0;
      auto frame_size = img.step * img.height;
      img.data.resize ((frame_size));
      auto begin = reinterpret_cast<const uint8_t*>(frame.dcs(i).begin());
      auto end = begin + (frame_size) ;
      std::copy(begin, end, img.data.begin());
      pub_dcs_[i]->publish(img);
    }
  }
}

void ToFSensor::updateFrame(const tofcore::Measurement_T &frame)
{
  auto stamp = ros::Time::now();
  switch (frame.type())
  {
  case tofcore::Measurement_T::DataType::AMBIENT:
  {
    publish_ambientData(frame, *pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::GRAYSCALE:
  {
    publish_ambientData(frame, *pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE:
  {
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, *pub_cust_pcd_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::AMPLITUDE:
  {
    // Probably not the case we just stream amplitude, but its here
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE:
  {
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, *pub_cust_pcd_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DCS:
  {
    publish_DCSData(frame, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::UNKNOWN:
  {
    break;
  }
  }
}