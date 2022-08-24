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


class T10Sensor : public rclcpp::Node
{
  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ambient_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amplitude_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
    rclcpp::Publisher<truesense_msgs::msg::Frame>::SharedPtr pub_dcs_;

    t10utils::Sensor interface_;
    t10utils::CartesianTransform cartesianTransform_;
    int lensCenterOffsetX_ = 0;
    int lensCenterOffsetY_ = 0;
    int lensType_ = 1;  //0- wide field, 1- standard field, 2 - narrow field

  public:
    T10Sensor();

  private:
    /// Callback method to be called when a parameter is changed.
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    void publish_amplData(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub);

    void publish_distData(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::Image>& pub);

    void publish_pointCloud(const t10utils::Frame& frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub);
    
    void publish_DCSData(const t10utils::Frame &frame, rclcpp::Publisher<truesense_msgs::msg::Frame> &pub);

    void updateFrame(const t10utils::Frame& frame);
};

#endif