#ifndef __ROS2_SENSOR_NODE_H__
#define __ROS2_SENSOR_NODE_H__

#include <cstdio>

#include <tofcore/tof_sensor.hpp>
#include <tofcore/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Temperature.h>
#include <tofcore_ros1/TofcorePointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <tofcore_ros1/tofcoreConfig.h>


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
    ros::NodeHandle n; 
    std::string sensor_location_;
    dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig> server;
    dynamic_reconfigure::Server<tofcore_ros1::tofcoreConfig>::CallbackType f;
    public:
    /// Standard constructor
    ToFSensor();

  private:
    /// Callback method to be called when a parameter is changed.
    //OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    void on_set_parameters(
        const std::vector<std::string> &parameters);
    void on_set_parameters_callback(tofcore_ros1::tofcoreConfig &config, uint32_t level);
    /// Publish received temperature data in frame to the to four different temperature topics (pub_temps_) with timestamp stamp.
    void publish_tempData(const tofcore::Measurement_T& frame, const ros::Time& stamp);

    /// Publish received amplitude data in frame to the topic publisher pub with timestamp stamp.
    void publish_amplData(const tofcore::Measurement_T& frame, ros::Publisher& pub, const ros::Time& stamp);

    /// Publish received ambient data in frame to the topic publisher pub with timestamp stamp.
    void publish_ambientData(const tofcore::Measurement_T& frame, ros::Publisher& pub, const ros::Time& stamp);

    /// Publish received distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_distData(const tofcore::Measurement_T& frame, ros::Publisher& pub, const ros::Time& stamp);

    /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_pointCloud(const tofcore::Measurement_T& frame, ros::Publisher& pub, ros::Publisher& cust_pub, const ros::Time& stamp);
    
    /// Publish a dcs data in frame to the four dcs topic publishers with timestamp stamp.
    void publish_DCSData(const tofcore::Measurement_T &frame, const ros::Time& stamp);

    /// Callback method provided to the tofcore library to notify us when new frame data has come in
    void updateFrame(const tofcore::Measurement_T& frame);

    /// Helper methods to send parameter updates down to the sensor
    void apply_stream_type_param(const std::string& parameter);
    void apply_integration_time_param(const std::string& parameter); 
    void apply_hdr_mode_param(const std::string& parameter); 
    void apply_streaming_param(const std::string& parameter);
    void apply_lens_type_param(const std::string& parameter);
    void apply_modulation_frequency_param(const std::string& parameter);
    void apply_distance_offset_param(const std::string& parameter);
    void apply_minimum_amplitude_param(const std::string& parameter);
    void apply_flip_horizontal_param(const std::string& parameter);
    void apply_flip_vertical_param(const std::string& parameter);
    void apply_binning_param(const std::string& parameter);
    void apply_sensor_name_param(const std::string& parameter);
    void apply_sensor_location_param(const std::string& parameter);

};

#endif