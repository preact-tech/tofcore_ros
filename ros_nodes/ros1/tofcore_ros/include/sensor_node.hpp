#ifndef __ROS2_SENSOR_NODE_H__
#define __ROS2_SENSOR_NODE_H__

#include <cstdio>

#include <tofcore/tof_sensor.hpp>
#include <tofcore/Measurement_T.hpp>
#include <tofcore/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Temperature.h>
#include <tofcore_ros1/TofcorePointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <tofcore_ros1/tofcoreConfig.h>
#include <nonblocking_queue.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

/// ToFSensor ROS1 node class for interacting with a PreAct ToF sensor/camera
class ToFSensor // : public ros::Node
{
private:
  ros::Publisher pub_ambient_;
  ros::Publisher pub_distance_;
  ros::Publisher pub_amplitude_;
  ros::Publisher pub_cust_pcd_;
  ros::Publisher pub_pcd_;
  std::array<ros::Publisher, 4> pub_dcs_;
  ros::Publisher sensor_temperature_tl;
  ros::Publisher sensor_temperature_tr;
  ros::Publisher sensor_temperature_bl;
  ros::Publisher sensor_temperature_br;

  std::unique_ptr<tofcore::Sensor> interface_;
  tofcore::CartesianTransform cartesianTransform_;
  ros::NodeHandle n_;
  std::string sensor_location_;
  boost::recursive_mutex config_mutex;
  std::mutex config_lock;
  // dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig> server_;
  // dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig>::CallbackType f_;
  tofcore_ros1::tofcoreConfig oldConfig_;
  dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig> *server_;
  // Filter parameters
  bool median_filter;
  int median_kernel = 3;
  bool bilateral_filter;
  int bilateral_kernel = 5;
  int bilateral_color = 75;
  int bilateral_space = 75;
  int maximum_amplitude = 2000;
  bool gradient_filter = true;
  int gradient_kernel = 1;
  int gradient_threshold = 1000;
  int gradient_filter_support = 6;
  bool temporal_filter;
  int temporal_alpha;
  int min_amplitude = 0;
  // HDR Parameters
  bool hdr_enable;
  std::string hdr_integrations;
  long unsigned int hdr_count;
  std::vector<std::tuple<cv::Mat, cv::Mat>> hdr_frames;
  int saturation_thresh = 2000;
  std::mutex m;
  // Auto-Exposure Parameters
  std::thread ae_update_thread;
  BlockingQueue<int> ae_integrations;
  bool ae_enable = false;
  int ae_target_mean_amp = 800;
  float ae_target_exp_avg_alpha = 0.1;
  float ae_rc_speed_factor = 0.2;
  float ae_rc_speed_factor_fast = 0.9;
  float ae_rc_rel_error_thresh = 0.1;
  float ae_rc_min_amp = 50.0;
  bool ae_rc_apply_min_reflect_thresh = true;

  float ae_min_integration_time_us = 10;
  float ae_max_integration_time_us = 4000;
  int ae_roi_top_px = 0;
  int ae_roi_bottom_px = 0;
  int ae_roi_left_px = 0;
  int ae_roi_right_px = 0;

  //== = measure
  float amp_measure_max = 0.0;
  float amp_measure_mean = 0.0;
  float error_measure = 0.0;

public:
  /// Standard constructor
  ToFSensor(ros::NodeHandle nh);

private:
  /// Callback method to be called when a parameter is changed.
  // OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  void on_set_parameters_callback(tofcore_ros1::tofcoreConfig &config, uint32_t level);
  /// Publish received temperature data in frame to the to four different temperature topics (pub_temps_) with timestamp stamp.
  void publish_tempData(const tofcore::Measurement_T &frame, const ros::Time &stamp);

  /// Publish received amplitude data in frame to the topic publisher pub with timestamp stamp.
  void publish_amplData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp);

  /// Publish received ambient data in frame to the topic publisher pub with timestamp stamp.
  void publish_ambientData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp);

  /// Publish received distance data in frame to the topic publisher pub with timestamp stamp.
  void publish_distData(const tofcore::Measurement_T &frame, ros::Publisher &pub, const ros::Time &stamp);

  /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
  void publish_pointCloud(const tofcore::Measurement_T &frame, ros::Publisher &pub, ros::Publisher &cust_pub, const ros::Time &stamp);
  /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
  void publish_pointCloudHDR(const tofcore::Measurement_T &frame, ros::Publisher &pub, ros::Publisher &cust_pub, ros::Publisher &pub_amp, ros::Publisher &pub_dist, const ros::Time &stamp);

  /// Publish a dcs data in frame to the four dcs topic publishers with timestamp stamp.
  void publish_DCSData(const tofcore::Measurement_T &frame, const ros::Time &stamp);

  /// Callback method provided to the tofcore library to notify us when new frame data has come in
  void updateFrame(const tofcore::Measurement_T &frame);

  /// Helper methods to send parameter updates down to the sensor
  void apply_stream_type_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_integration_time_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_streaming_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_modulation_frequency_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_distance_offset_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_minimum_amplitude_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_flip_horizontal_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_flip_vertical_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_binning_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_sensor_name_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_sensor_location_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);
  void apply_vsm_param(const std::string &parameter, tofcore_ros1::tofcoreConfig &config);

  // Auto-Exposure Functions
  void process_ae(short unsigned int integration_time_us, cv::Mat& ampimg, float timestep );
  float measure_from_avg(cv::Mat ampimg, int int_us);
  float obtain_error(float amp_measure_max, float amp_measure_mean, int int_us);
  int control_recursive(int integration_time_us , float amp_measure_mean);
  float measured_error();
  std::thread spawn_ae_update();
  void ae_watchdog();
};

#endif