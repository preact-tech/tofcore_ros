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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tofcore_msgs/msg/tofcore_point_cloud2.hpp>


/// ToFSensor ROS1 node class for interacting with a PreAct ToF sensor/camera
class ToFSensor // : public ros::Node
{
  private:
  
    ros::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ambient_;
    ros::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_distance_;
    ros::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amplitude_;
    ros::Publisher<tofcore_msgs::msg::TofcorePointCloud2>::SharedPtr pub_cust_pcd_;
    ros::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
    std::array<ros::Publisher<sensor_msgs::msg::Image>::SharedPtr, 4> pub_dcs_;
    ros::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_tl;
    ros::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_tr;
    ros::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_bl;
    ros::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_br;

    std::unique_ptr<tofcore::Sensor> interface_;
    tofcore::CartesianTransform cartesianTransform_;

  public:
    /// Standard constructor
    ToFSensor();

  private:
    /// Callback method to be called when a parameter is changed.
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
        const std::vector<ros::Parameter> &parameters);

    /// Publish received temperature data in frame to the to four different temperature topics (pub_temps_) with timestamp stamp.
    void publish_tempData(const tofcore::Measurement_T& frame, const ros::Time& stamp);

    /// Publish received amplitude data in frame to the topic publisher pub with timestamp stamp.
    void publish_amplData(const tofcore::Measurement_T& frame, ros::Publisher<sensor_msgs::msg::Image>& pub, const ros::Time& stamp);

    /// Publish received ambient data in frame to the topic publisher pub with timestamp stamp.
    void publish_ambientData(const tofcore::Measurement_T& frame, ros::Publisher<sensor_msgs::msg::Image>& pub, const ros::Time& stamp);

    /// Publish received distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_distData(const tofcore::Measurement_T& frame, ros::Publisher<sensor_msgs::msg::Image>& pub, const ros::Time& stamp);

    /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_pointCloud(const tofcore::Measurement_T& frame, ros::Publisher<sensor_msgs::msg::PointCloud2>& pub, ros::Publisher<tofcore_msgs::msg::TofcorePointCloud2>& cust_pub, const ros::Time& stamp);
    
    /// Publish a dcs data in frame to the four dcs topic publishers with timestamp stamp.
    void publish_DCSData(const tofcore::Measurement_T &frame, const ros::Time& stamp);

    /// Callback method provided to the tofcore library to notify us when new frame data has come in
    void updateFrame(const tofcore::Measurement_T& frame);

    /// Helper methods to send parameter updates down to the sensor
    void apply_stream_type_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_integration_time_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result); 
    void apply_hdr_mode_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result); 
    void apply_streaming_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_lens_type_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_modulation_frequency_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_distance_offset_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_minimum_amplitude_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_flip_horizontal_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_flip_vertical_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_binning_param(const ros::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);

};

#endif