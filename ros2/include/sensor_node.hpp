#ifndef __ROS2_T10_SENSOR_NODE_H__
#define __ROS2_T10_SENSOR_NODE_H__

#include <cstdio>

#include <t10utils/t10_sensor.hpp>
#include <t10utils/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <truesense_msgs/msg/frame.hpp>


/// T10Sensor ROS2 node class for interacting with a T10 sensor/camera
class T10Sensor : public rclcpp::Node
{
  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ambient_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amplitude_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 4> pub_dcs_;

    t10utils::Sensor interface_;
    t10utils::CartesianTransform cartesianTransform_;
    int lensCenterOffsetX_ = 0;
    int lensCenterOffsetY_ = 0;
    int lensType_ = 0;  //0- wide field, 1- standard field, 2 - narrow field

  public:
    /// Standard constructor
    T10Sensor();

  private:
    /// Callback method to be called when a parameter is changed.
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    /// Publish received amplitude data in frame to the topic publisher pub with timestamp stamp.
    void publish_amplData(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub, const rclcpp::Time& stamp);

    /// Publish received distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_distData(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub, const rclcpp::Time& stamp);

    /// Publish a PointCloud using distance data in frame to the topic publisher pub with timestamp stamp.
    void publish_pointCloud(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub, const rclcpp::Time& stamp);
    
    /// Publish a dcs data in frame to the four dcs topic publishers with timestamp stamp.
    void publish_DCSData(const t10utils::Frame &frame, const rclcpp::Time& stamp);

    /// Callback method provided to the t10utils library to notify us when new frame data has come in
    void updateFrame(const t10utils::Frame& frame);
};

#endif