#include "sensor_node.hpp"



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
constexpr auto MODEL_NAME  = "model_name";
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
constexpr auto SENSOR_NAME  = "sensor_name";
constexpr auto SENSOR_LOCATION  = "sensor_location";
constexpr auto DISCOVERY_FILTER  = "discovery_filter"; //TODO: Implement this


/// Quick helper function that return true if the string haystack starts with the string needle
bool begins_with(const std::string& needle, const std::string& haystack )
{
  return haystack.rfind(needle, 0) == 0;
}


ToFSensor::ToFSensor()
    : Node("tof_sensor", "truesense")
{
  rclcpp::QoS pub_qos(10);
  pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  // Setup topic pulbishers
  pub_ambient_ = this->create_publisher<sensor_msgs::msg::Image>("ambient", pub_qos);
  pub_distance_ = this->create_publisher<sensor_msgs::msg::Image>("depth", pub_qos);//renamed this from distance to depth to match truesense node
  pub_amplitude_ = this->create_publisher<sensor_msgs::msg::Image>("amplitude", pub_qos);
  pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", pub_qos);
  pub_integration_ = this->create_publisher<tofcore_msgs::msg::IntegrationTime>("frame_raw", pub_qos);

  for(size_t i = 0; i != pub_dcs_.size(); i++) {
    std::string topic {"dcs"};
    topic += std::to_string(i);
    pub_dcs_[i] = this->create_publisher<sensor_msgs::msg::Image>(topic, pub_qos);
  }

  sensor_temperature_tl = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_tl", pub_qos);
  sensor_temperature_tr = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_tr", pub_qos);
  sensor_temperature_bl = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_bl", pub_qos);
  sensor_temperature_br = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_br", pub_qos);


 
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
  rcl_interfaces::msg::ParameterDescriptor readonly_descriptor;
  readonly_descriptor.read_only = true;

  //Read Only params
  this->declare_parameter(API_VERSION, versionData.m_softwareSourceID,readonly_descriptor);// TODO: Update this when API version is availible
  this->declare_parameter(CHIP_ID, std::to_string(versionData.m_sensorChipId),readonly_descriptor);
  this->declare_parameter(MODEL_NAME, versionData.m_modelName,readonly_descriptor);
  this->declare_parameter(SW_VERSION, versionData.m_softwareVersion,readonly_descriptor);
  this->declare_parameter(SENSOR_URL,  "/dev/ttyACM0",readonly_descriptor); 
  

  //Configurable params
  this->declare_parameter(CAPTURE_MODE, "distance_amplitude");
  this->declare_parameter(STREAMING_STATE, true);
  this->declare_parameter(MODULATION_FREQUENCY, "12");
  this->declare_parameter(DISTANCE_OFFSET, 0);
  this->declare_parameter(MINIMUM_AMPLITUDE, 0);
  this->declare_parameter(FLIP_HORIZONTAL, false);
  this->declare_parameter(FLIP_VERITCAL, false);
  this->declare_parameter(BINNING, false);

  //Reading optional values from sensor
  std::optional<std::string> init_name = interface_->getSensorName();
  std::optional<std::string>  init_location = interface_->getSensorLocation();
  std::optional<std::vector<short unsigned int> >  init_integration = interface_->getIntegrationTimes();

  if(init_name)
    this->declare_parameter(SENSOR_NAME, *init_name);
  else
    this->declare_parameter(SENSOR_NAME, "Mojave");

  if(init_location)
  {
    this->declare_parameter(SENSOR_LOCATION, *init_location);
    this->sensor_location_=*init_location;
  }
  else
  {
    this->declare_parameter(SENSOR_LOCATION, "Unknown");
    this->sensor_location_="Unknown";
  }

  if(init_integration)
    this->declare_parameter(INTEGRATION_TIME, (*init_integration).at(0));
  else
    this->declare_parameter(INTEGRATION_TIME, 500);

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
    auto name = parameter.get_name();
    if (name == CAPTURE_MODE)
    {
      bool streaming = true;
      this->get_parameter(STREAMING_STATE, streaming);
      if (streaming)
      {
        this->apply_stream_type_param(parameter, result);
      }
    }
    else if (begins_with("integration_time", name))
    {
      this->apply_integration_time_param(parameter, result);
    }
    else if( name == STREAMING_STATE)
    {
      this->apply_streaming_param(parameter, result);
    }
    else if( name == MODULATION_FREQUENCY)
    {
      this->apply_modulation_frequency_param(parameter, result);
    }
    else if( name == DISTANCE_OFFSET)
    {
      this->apply_distance_offset_param(parameter, result);
    }
    else if( name == MINIMUM_AMPLITUDE)
    {
      this->apply_minimum_amplitude_param(parameter, result);
    }
    else if( name == FLIP_HORIZONTAL)
    {
      this->apply_flip_horizontal_param(parameter, result);

    }
    else if( name == FLIP_VERITCAL)
    {
      this->apply_flip_vertical_param(parameter, result);
    }
    else if( name == BINNING)
    {
      this->apply_binning_param(parameter, result);
    }
    else if( name == SENSOR_NAME)
    {
      this->apply_sensor_name_param(parameter, result);
    }
    else if( name == SENSOR_LOCATION)
    {
      this->apply_sensor_location_param(parameter, result);
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
    interface_->setIntegrationTime(value);
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
{      rcl_interfaces::msg::SetParametersResult dummy_result;

  try {
    auto value = parameter.as_bool();
    RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ?"true":"false"));
    if(value) {
      rclcpp::Parameter stream_type;
      (void)this->get_parameter(CAPTURE_MODE, stream_type);
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

void ToFSensor::apply_minimum_amplitude_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult&)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);

  interface_->setMinAmplitude(value);
}
void ToFSensor::apply_flip_horizontal_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult&)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ?"true":"false"));
  //TODO:  It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipHorizontally(value);
  rclcpp::Parameter stream_type;
  rclcpp::Parameter is_streaming;
  rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->get_parameter(STREAMING_STATE, is_streaming);
  (void)this->get_parameter(CAPTURE_MODE, stream_type);
  if(is_streaming.as_bool())
    this->apply_stream_type_param(stream_type, dummy_result);
}
void ToFSensor::apply_flip_vertical_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult&)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ?"true":"false"));
  //TODO: It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipVertically(value);
  rclcpp::Parameter stream_type;
  rclcpp::Parameter is_streaming;
  rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->get_parameter(STREAMING_STATE, is_streaming);
  (void)this->get_parameter(CAPTURE_MODE, stream_type);
  if(is_streaming.as_bool())
    this->apply_stream_type_param(stream_type, dummy_result);
}
void ToFSensor::apply_binning_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult&)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ?"true":"false"));

  interface_->setBinning(value,value);

}
void ToFSensor::apply_sensor_name_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& )
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str() );

  interface_->setSensorName(value);
  interface_->storeSettings();

}
void ToFSensor::apply_sensor_location_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& )
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str() );

  interface_->setSensorLocation(value);
  interface_->storeSettings();
  this->sensor_location_=value;

}
void ToFSensor::publish_tempData(const tofcore::Measurement_T &frame, const rclcpp::Time& stamp)
{
  const std::array<float, 4> defaultTemps {0.0,0.0,0.0,0.0};
  auto temperatures = frame.sensor_temperatures().value_or(defaultTemps);
  int count = 0;

  for(const auto& i : temperatures) {
    sensor_msgs::msg::Temperature tmp;
    tmp.header.stamp = stamp;
    tmp.header.frame_id = this->sensor_location_;
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

void ToFSensor::publish_amplData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
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

void ToFSensor::publish_ambientData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
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

void ToFSensor::publish_distData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
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

void ToFSensor::publish_pointCloud(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &pub,rclcpp::Publisher<tofcore_msgs::msg::IntegrationTime> &cust_pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 cloud_msg{};
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = this->sensor_location_;
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;
  tofcore_msgs::msg::IntegrationTime integration_msg{};
  integration_msg.header.stamp = stamp;
  integration_msg.header.frame_id = this->sensor_location_;
  //Adding this to the message for the auto exposure node
  //Need to check if it exists because this is optional value
  if (frame.integration_times())
  {
    auto integration_times=*std::move(frame.integration_times());
    integration_msg.integration_time=integration_times.at(0);
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.resize(frame.height() * frame.width());
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
  cloud_msg.height = static_cast<uint32_t>(frame.height());
  cloud_msg.width = static_cast<uint32_t>(frame.width());
  cloud_msg.row_step = frame.width() * 19; // 19 is the size in bytes of all the point cloud fields

  sensor_msgs::PointCloud2Iterator<float> it_x{cloud_msg, "x"};
  sensor_msgs::PointCloud2Iterator<float> it_y{cloud_msg, "y"};
  sensor_msgs::PointCloud2Iterator<float> it_z{cloud_msg, "z"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude{cloud_msg, "amplitude"};
  sensor_msgs::PointCloud2Iterator<int16_t> it_ambient{cloud_msg, "ambient"};
  sensor_msgs::PointCloud2Iterator<uint8_t> it_valid{cloud_msg, "valid"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_phase{cloud_msg, "distance"};

  auto it_d = frame.distance().begin();
  auto it_a = frame.amplitude().begin();
  uint32_t count = 0;
  while (it_d != frame.distance().end())
  {
    auto distance = *it_d;
    auto y = count / frame.width();
    auto x = count % frame.width();
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
    if (frame.type() == tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
    {
      *it_amplitude = *it_a;
      it_a += 1;
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
    it_d += 1;
  }
  //This is dumb and redundant but I don't want to break rviz viewer
  pub.publish(cloud_msg);
  cust_pub.publish(integration_msg);
}


void ToFSensor::publish_DCSData(const tofcore::Measurement_T &frame, const rclcpp::Time& stamp)
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
      sensor_msgs::msg::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = this->sensor_location_;
      img.height = static_cast<uint32_t>(frame.height());
      // RCLCPP_INFO(this->get_logger(), "Frame Height: %d", frame.height());

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
  auto stamp = this->now();
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
    publish_pointCloud(frame, *pub_pcd_, *pub_integration_, stamp);
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
    publish_pointCloud(frame, *pub_pcd_, *pub_integration_, stamp);
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