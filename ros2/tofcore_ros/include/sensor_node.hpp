#ifndef __ROS2_SENSOR_NODE_H__
#define __ROS2_SENSOR_NODE_H__

#include <cstdio>

#include <discovery.hpp>

#include <tofcore/tof_sensor.hpp>
#include <tofcore/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <tofcore_msgs/msg/integration_time.hpp>

/// ToFSensor ROS2 node class for interacting with a PreAct ToF sensor/camera
class ToFSensor : public rclcpp::Node
{
  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ambient_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_distance_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amplitude_;
    rclcpp::Publisher<tofcore_msgs::msg::IntegrationTime>::SharedPtr pub_integration_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 4> pub_dcs_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_tl;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_tr;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_bl;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sensor_temperature_br;
        
    std::unique_ptr<tofcore::Sensor> interface_;
    tofcore::CartesianTransform cartesianTransform_;
    std::string sensor_location_;
    ToFDiscovery discovery_helper_;

  public:
    /// Standard constructor
    ToFSensor();

  private:
    /// Callback method to be called when a parameter is changed.
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    /// Publish received temperature data in frame to the to four different temperature topics (pub_temps_) with timestamp stamp.
    void publish_tempData(const tofcore::Measurement_T& frame, const rclcpp::Time& stamp);

    /// Publish received amplitude data in frame to the topic publisher pub with timestamp stamp.
    void publish_amplData(const tofcore::Measurement_T& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub, const rclcpp::Time& stamp);

    /// Publish received ambient data in frame to the topic publisher pub with timestamp stamp.
    void publish_ambientData(const tofcore::Measurement_T& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub, const rclcpp::Time& stamp);

    /// Publish received distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_distData(const tofcore::Measurement_T& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub, const rclcpp::Time& stamp);

    /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_pointCloud(const tofcore::Measurement_T& frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub, rclcpp::Publisher<tofcore_msgs::msg::IntegrationTime>& cust_pub, const rclcpp::Time& stamp);
    
    /// Publish a dcs data in frame to the four dcs topic publishers with timestamp stamp.
    void publish_DCSData(const tofcore::Measurement_T &frame, const rclcpp::Time& stamp);

    /// Callback method provided to the tofcore library to notify us when new frame data has come in
    void updateFrame(const tofcore::Measurement_T& frame);

    /// Helper methods to send parameter updates down to the sensor
    void apply_stream_type_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_integration_time_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result); 
    void apply_hdr_mode_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result); 
    void apply_streaming_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_lens_type_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_modulation_frequency_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_distance_offset_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_minimum_amplitude_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_flip_horizontal_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_flip_vertical_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_binning_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_sensor_name_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);
    void apply_sensor_location_param(const rclcpp::Parameter& parameter, rcl_interfaces::msg::SetParametersResult& result);

};

#endif