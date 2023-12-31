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

// Read Only params
constexpr auto API_VERSION = "/tof_sensor/api_version";
constexpr auto CHIP_ID = "/tof_sensor/chip_id";
constexpr auto MODEL_NAME = "/tof_sensor/model_name";
constexpr auto SW_VERSION = "/tof_sensor/sw_version";
constexpr auto SENSOR_URL = "/tof_sensor/sensor_url";

// Configurable params
constexpr auto CAPTURE_MODE = "/tof_sensor/capture_mode";
constexpr auto INTEGRATION_TIME = "/tof_sensor/integration_time";
constexpr auto STREAMING_STATE = "/tof_sensor/streaming";
constexpr auto MODULATION_FREQUENCY = "/tof_sensor/modulation_frequency";
constexpr auto DISTANCE_OFFSET = "/tof_sensor/distance_offset";
constexpr auto MINIMUM_AMPLITUDE = "/tof_sensor/minimum_amplitude";
constexpr auto FLIP_HORIZONTAL = "/tof_sensor/flip_hotizontal";
constexpr auto FLIP_VERITCAL = "/tof_sensor/flip_vertical";
constexpr auto BINNING = "/tof_sensor/binning";
constexpr auto SENSOR_NAME = "/tof_sensor/sensor_name";
constexpr auto SENSOR_LOCATION = "/tof_sensor/sensor_location";
constexpr auto DISCOVERY_FILTER = "/tof_sensor/discovery_filter"; // TODO: Implement this

// Filter parameters
constexpr auto MEDIAN_FILTER = "/tof_sensor/median_filter";
constexpr auto MEDIAN_KERNEL = "/tof_sensor/median_kernel";
constexpr auto BILATERAL_FILTER = "/tof_sensor/bilateral_filter";
constexpr auto BILATERAL_KERNEL = "/tof_sensor/bilateral_kernel";
constexpr auto BILATERAL_COLOR = "/tof_sensor/bilateral_color";
constexpr auto BILATERAL_SPACE = "/tof_sensor/bilateral_space";
constexpr auto TEMPORAL_FILTER = "/tof_sensor/temporal_filter";
constexpr auto TEMPORAL_ALPHA = "/tof_sensor/temporal_alpha";

/// Quick helper function that return true if the string haystack starts with the string needle
bool begins_with(const std::string &needle, const std::string &haystack)
{
  return haystack.rfind(needle, 0) == 0;
}

ToFSensor::ToFSensor(ros::NodeHandle nh)
{
  this->n_ = nh;
  int pub_queue = 100;

  // Setup topic pulbishers
  pub_ambient_ = this->n_.advertise<sensor_msgs::Image>("ambient", pub_queue);
  pub_distance_ = this->n_.advertise<sensor_msgs::Image>("depth", pub_queue); // renamed this from distance to depth to match truesense node
  pub_amplitude_ = this->n_.advertise<sensor_msgs::Image>("amplitude", pub_queue);
  pub_pcd_ = this->n_.advertise<sensor_msgs::PointCloud2>("points", pub_queue);
  pub_cust_pcd_ = this->n_.advertise<tofcore_ros1::TofcorePointCloud2>("cust_points", pub_queue);

  for (size_t i = 0; i != pub_dcs_.size(); i++)
  {
    std::string topic{"dcs"};
    topic += std::to_string(i);
    pub_dcs_[i] = this->n_.advertise<sensor_msgs::Image>(topic, pub_queue);
  }

  sensor_temperature_tl = this->n_.advertise<sensor_msgs::Temperature>("sensor_temperature_tl", pub_queue);
  sensor_temperature_tr = this->n_.advertise<sensor_msgs::Temperature>("sensor_temperature_tr", pub_queue);
  sensor_temperature_bl = this->n_.advertise<sensor_msgs::Temperature>("sensor_temperature_bl", pub_queue);
  sensor_temperature_br = this->n_.advertise<sensor_msgs::Temperature>("sensor_temperature_br", pub_queue);

  interface_.reset(new tofcore::Sensor(1, "/dev/ttyACM0"));
  interface_->stopStream();
  dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig>::CallbackType f_ = boost::bind(&ToFSensor::on_set_parameters_callback, this, boost::placeholders::_1, boost::placeholders::_2);
  dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig> *server_ = new dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig>(this->config_mutex, this->n_);
  server_->setCallback(f_);

  std::vector<double> rays_x, rays_y, rays_z;
  interface_->getLensInfo(rays_x, rays_y, rays_z);
  cartesianTransform_.initLensTransform(m_width, HEIGHT, rays_x, rays_y, rays_z);

  // Setup ROS parameters
  tofcore_ros1::tofcoreConfig config;
  server_->getConfigDefault(config);

  // Get sensor info
  TofComm::versionData_t versionData{};
  interface_->getSensorInfo(versionData);

  config.api_version = versionData.m_softwareSourceID;
  config.chip_id = std::to_string(versionData.m_sensorChipId);
  config.model_name = versionData.m_modelName;
  config.sw_version = versionData.m_softwareVersion;

  // Reading optional values from sensor
  std::optional<std::string> init_name = interface_->getSensorName();
  std::optional<std::string> init_location = interface_->getSensorLocation();
  std::optional<std::vector<short unsigned int>> init_integration = interface_->getIntegrationTimes();

  if (init_name)
    config.sensor_name = *init_name;
  else
    config.sensor_name = "Mojave";

  if (init_location)
  {
    config.sensor_location = *init_location;
    this->sensor_location_ = *init_location;
  }
  else
  {
    config.sensor_location = "Unknown";
    this->sensor_location_ = "Unknown";
  }

  if (init_integration)
    config.integration_time = (*init_integration).at(0);
  else
    config.integration_time = 500;

  server_->updateConfig(config);

  // Setup parameter server call back
  (void)interface_->subscribeMeasurement([&](std::shared_ptr<tofcore::Measurement_T> f) -> void
                                         { updateFrame(*f); });

  ROS_INFO("Initialized");
}

void ToFSensor::on_set_parameters_callback(tofcore_ros1::tofcoreConfig &config, uint32_t)
{
  // Keeping track of the old config items is clunky, but I can't find a better way to determine which of the parameters have changed
  std::vector<std::string> parameters;
  this->n_.getParamNames(parameters);

  for (const auto &parameter : parameters)
  {
    if (parameter == CAPTURE_MODE && config.capture_mode != this->oldConfig_.capture_mode)
    {
      bool streaming = config.streaming;
      if (streaming)
      {
        this->apply_stream_type_param(parameter, config);
      }
    }
    else if (parameter == INTEGRATION_TIME && config.integration_time != this->oldConfig_.integration_time)
    {
      this->apply_integration_time_param(parameter, config);
    }
    else if (parameter == STREAMING_STATE && config.streaming != this->oldConfig_.streaming)
    {
      this->apply_streaming_param(parameter, config);
    }
    else if (parameter == MODULATION_FREQUENCY && config.modulation_frequency != this->oldConfig_.modulation_frequency)
    {
      this->apply_modulation_frequency_param(parameter, config);
    }
    else if (parameter == DISTANCE_OFFSET && config.distance_offset != this->oldConfig_.distance_offset)
    {
      this->apply_distance_offset_param(parameter, config);
    }
    else if (parameter == MINIMUM_AMPLITUDE && config.minimum_amplitude != this->oldConfig_.minimum_amplitude)
    {
      this->apply_minimum_amplitude_param(parameter, config);
    }
    else if (parameter == FLIP_HORIZONTAL && config.flip_hotizontal != this->oldConfig_.flip_hotizontal)
    {
      this->apply_flip_horizontal_param(parameter, config);
    }
    else if (parameter == FLIP_VERITCAL && config.flip_vertical != this->oldConfig_.flip_vertical)
    {
      this->apply_flip_vertical_param(parameter, config);
    }
    else if (parameter == BINNING && config.binning != this->oldConfig_.binning)
    {
      this->apply_binning_param(parameter, config);
    }
    else if (parameter == SENSOR_NAME && config.sensor_name != this->oldConfig_.sensor_name)
    {
      this->apply_sensor_name_param(parameter, config);
    }
    else if (parameter == SENSOR_LOCATION && config.sensor_location != this->oldConfig_.sensor_location)
    {
      this->apply_sensor_location_param(parameter, config);
    }
    else if (parameter == MEDIAN_FILTER && config.median_filter != this->oldConfig_.median_filter)
    {
      bool value = config.median_filter;
      ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
      this->median_filter = value;
    }
    else if (parameter == MEDIAN_KERNEL && config.median_kernel != this->oldConfig_.median_kernel)
    {
      int value = config.median_kernel;
      ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), int(value / 2) * 2 + 1);
      this->median_kernel = int(value / 2) * 2 + 1;
    }
    else if (parameter == BILATERAL_FILTER && config.bilateral_filter != this->oldConfig_.bilateral_filter)
    {
      bool value = config.bilateral_filter;
      ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
      this->bilateral_filter = value;
    }
    else if (parameter == BILATERAL_COLOR && config.bilateral_color != this->oldConfig_.bilateral_color)
    {
      int value = config.bilateral_color;
      ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);
      this->bilateral_color = value;
    }
    else if (parameter == BILATERAL_KERNEL && config.bilateral_kernel != this->oldConfig_.bilateral_kernel)
    {
      int value = config.bilateral_kernel;
      ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), int(value / 2) * 2 + 1);
      this->bilateral_kernel = int(value / 2) * 2 + 1;
    }
    else if (parameter == BILATERAL_SPACE && config.bilateral_space != this->oldConfig_.bilateral_space)
    {
      int value = config.bilateral_space;
      ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);
      this->bilateral_space = value;
    }
    // else if (parameter == TEMPORAL_FILTER && config.temporal_filter != this->oldConfig_.temporal_filter)
    // {
    //   bool value = config.temporal_filter;
    //   ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
    //   this->temporal_filter = value;
    // }
    // else if (parameter == TEMPORAL_ALPHA && config.temporal_alpha != this->oldConfig_.temporal_alpha)
    // {
    //   int value = config.temporal_alpha;
    //   ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), value);
    //   this->temporal_alpha = value;
    // }
  }
  this->oldConfig_ = config;
}

void ToFSensor::apply_stream_type_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  std::string value;
  value = config.capture_mode;
  ROS_INFO("Handling parameter \"%s\" : \"%s\"", parameter.c_str(), value.c_str());
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
    ROS_ERROR("Unknown stream type: %s", value.c_str());
  }
}

void ToFSensor::apply_integration_time_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  int32_t value;
  value = config.integration_time;
  ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);
  if (value < MIN_INTEGRATION_TIME || value > MAX_INTEGRATION_TIME)
  {
    ROS_ERROR("Value out of range: %d", value);
  }
  else
  {
    interface_->setIntegrationTime(value);
  }
}

void ToFSensor::apply_modulation_frequency_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  uint16_t value;
  value = config.modulation_frequency;

  ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);

  interface_->setModulation(value);
}

void ToFSensor::apply_streaming_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  try
  {
    bool value;
    value = config.streaming;
    ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
    if (value)
    {
      this->apply_stream_type_param(CAPTURE_MODE, config);
    }
    else
    {
      interface_->stopStream();
    }
  }
  catch (std::exception &e)
  {
    // result.successful = false;
    // result.reason = e.what();
  }
}

void ToFSensor::apply_distance_offset_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  int32_t value;
  value = config.distance_offset;
  ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);
  interface_->setOffset(value);
}

void ToFSensor::apply_minimum_amplitude_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  int value;
  value = config.minimum_amplitude;
  ROS_INFO("Handling parameter \"%s\" : %d", parameter.c_str(), value);
  this->min_amplitude = value;
  interface_->setMinAmplitude(value);
}
void ToFSensor::apply_flip_horizontal_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  bool value;
  value = config.flip_hotizontal;
  ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
  interface_->stopStream();
  interface_->setFlipHorizontally(value);
  if (config.streaming)
    this->apply_stream_type_param(CAPTURE_MODE, config);
}
void ToFSensor::apply_flip_vertical_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  bool value;
  value = config.flip_vertical;
  ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
  interface_->stopStream();
  interface_->setFlipVertically(value);
  if (config.streaming)
    this->apply_stream_type_param(CAPTURE_MODE, config);
}
void ToFSensor::apply_binning_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  bool value;
  value = config.binning;
  ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), (value ? "true" : "false"));
  interface_->setBinning(value, value);
}
void ToFSensor::apply_sensor_name_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  auto value = config.sensor_name;
  ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), value.c_str());
  interface_->setSensorName(value);
  interface_->storeSettings();
}
void ToFSensor::apply_sensor_location_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config)
{
  auto value = config.sensor_location;
  ROS_INFO("Handling parameter \"%s\" : %s", parameter.c_str(), value.c_str());
  interface_->setSensorLocation(value);
  interface_->storeSettings();
  this->sensor_location_ = value;
}
void ToFSensor::publish_tempData(const tofcore::Measurement_T &frame, const ros::Time &stamp)
{
  const std::array<float, 4> defaultTemps{0.0, 0.0, 0.0, 0.0};
  auto temperatures = frame.sensor_temperatures().value_or(defaultTemps);
  int count = 0;

  for (const auto &i : temperatures)
  {
    sensor_msgs::Temperature tmp;
    tmp.header.stamp = stamp;
    tmp.header.frame_id = "base_link";
    tmp.temperature = i;
    tmp.variance = 0;
    switch (count)
    {
    case 0:
    {
      sensor_temperature_tl.publish(tmp);
      break;
    }
    case 1:
    {
      sensor_temperature_tr.publish(tmp);
      break;
    }
    case 2:
    {
      sensor_temperature_bl.publish(tmp);
      break;
    }
    case 3:
    {
      sensor_temperature_br.publish(tmp);
      break;
    }
    }
    count++;
  }
}

void ToFSensor::publish_amplData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp)
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
  uint8_t *amplitude_begin = (uint8_t *)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_ambientData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp)
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
  uint8_t *amplitude_begin = (uint8_t *)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_distData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp)
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
  uint8_t *dist_begin = (uint8_t *)distance_bv.data();
  std::copy_n(dist_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_pointCloud(const tofcore::Measurement_T &frame, ros::Publisher &pub, ros::Publisher &cust_pub, const ros::Time &stamp)
{
  tofcore_ros1::TofcorePointCloud2 cloud_msg{};
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.point_cloud.header.stamp = stamp;
  cloud_msg.point_cloud.header.frame_id = "base_link";
  cloud_msg.point_cloud.is_dense = true;
  cloud_msg.point_cloud.is_bigendian = false;

  // Adding this to the message for the auto exposure node
  // Need to check if it exists because this is optional value
  if (frame.integration_times())
  {
    auto integration_times = *std::move(frame.integration_times());
    cloud_msg.integration_time = integration_times.at(0);
  }

  cv::Mat dist_frame = cv::Mat(frame.height(), frame.width(), CV_16UC1, (void *)frame.distance().begin());

  if (this->median_filter)
  {
    // cv::Mat src = cv::Mat::zeros(dist_frame.size(), CV_16U);
    // cv::Mat dst = cv::Mat::zeros(dist_frame.size(), CV_16U);
    cv::medianBlur(dist_frame, dist_frame, this->median_kernel);
  }
  if (this->bilateral_filter)
  {
    cv::Mat src = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
    cv::Mat dst = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
    dist_frame.convertTo(src, CV_32FC1);
    cv::bilateralFilter(src, dst, this->bilateral_kernel, this->bilateral_color, this->bilateral_space);
    dst.convertTo(dist_frame, CV_16UC1);
  }
  if (this->temporal_filter)
  {
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg.point_cloud);
  modifier.resize(frame.height() * frame.width());
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "amplitude", 1, sensor_msgs::PointField::UINT16,
      "ambient", 1, sensor_msgs::PointField::INT16,
      "valid", 1, sensor_msgs::PointField::UINT8,
      "distance", 1, sensor_msgs::PointField::UINT16); // TODO: do we need phase here?

  // Note: For some reason setPointCloudFields doesn't set row_step
  //      and resets msg height and m_width so setup them here.
  cloud_msg.point_cloud.height = static_cast<uint32_t>(frame.height());
  cloud_msg.point_cloud.width = static_cast<uint32_t>(frame.width());
  cloud_msg.point_cloud.row_step = frame.width() * 19; // 19 is the size in bytes of all the point cloud fields

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg.point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg.point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg.point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude(cloud_msg.point_cloud, "amplitude");
  sensor_msgs::PointCloud2Iterator<int16_t> it_ambient(cloud_msg.point_cloud, "ambient");
  sensor_msgs::PointCloud2Iterator<uint8_t> it_valid(cloud_msg.point_cloud, "valid");
  sensor_msgs::PointCloud2Iterator<uint16_t> it_phase(cloud_msg.point_cloud, "distance");

  auto it_d = (const unsigned short *)dist_frame.datastart;
  auto it_a = frame.amplitude().begin();
  uint32_t count = 0;
  while (it_d != (const unsigned short *)dist_frame.dataend)
  {
    if (*it_a < this->min_amplitude)
    {
      *it_x = std::numeric_limits<float>::quiet_NaN();
      *it_y = std::numeric_limits<float>::quiet_NaN();
      *it_z = std::numeric_limits<float>::quiet_NaN();
      *it_amplitude = std::numeric_limits<short unsigned int>::quiet_NaN();
      it_a += 1;
      *it_ambient = std::numeric_limits<short int>::quiet_NaN();
      *it_phase = std::numeric_limits<short unsigned int>::quiet_NaN();
      *it_valid = std::numeric_limits<unsigned char>::quiet_NaN();
    }
    else
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
    }
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
  // This is dumb and redundant but I don't want to break rviz viewer
  pub.publish(cloud_msg.point_cloud);
  cust_pub.publish(cloud_msg);
}

void ToFSensor::publish_DCSData(const tofcore::Measurement_T &frame, const ros::Time &stamp)
{

  // TODO Need to figure out the best way to publish image meta-data including:
  //   modulation_frequency
  //   integration_time
  //   binning
  //   vled_mv
  //   chip_id
  //
  //  Also need to figure out how to publish an ambient frame which will be required for use with the calibration app.
  //
  // Does feature/add-meta-data-publishers branch have work that should be used for this?

  if (frame.type() == tofcore::Measurement_T::DataType::DCS)
  {
    for (auto i = 0; i != 4; ++i)
    {
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
      img.data.resize((frame_size));
      auto begin = reinterpret_cast<const uint8_t *>(frame.dcs(i).begin());
      auto end = begin + (frame_size);
      std::copy(begin, end, img.data.begin());
      pub_dcs_[i].publish(img);
    }
  }
}

void ToFSensor::updateFrame(const tofcore::Measurement_T &frame)
{
  // ROS_INFO("Updating Frame ");
  auto stamp = ros::Time::now();
  switch (frame.type())
  {
  case tofcore::Measurement_T::DataType::AMBIENT:
  {
    publish_ambientData(frame, pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::GRAYSCALE:
  {
    publish_ambientData(frame, pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE:
  {
    publish_amplData(frame, pub_amplitude_, stamp);
    publish_distData(frame, pub_distance_, stamp);
    publish_pointCloud(frame, pub_pcd_, pub_cust_pcd_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::AMPLITUDE:
  {
    // Probably not the case we just stream amplitude, but its here
    publish_amplData(frame, pub_amplitude_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE:
  {
    publish_distData(frame, pub_distance_, stamp);
    publish_pointCloud(frame, pub_pcd_, pub_cust_pcd_, stamp);
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