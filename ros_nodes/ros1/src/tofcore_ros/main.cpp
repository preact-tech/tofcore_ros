#include "sensor_node.hpp"
#include <ros/ros.h>
#include <memory>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "tof_sensor");
  //ros::spin(std::make_shared<ToFSensor>());
  //auto tofsensor = std::make_shared<ToFSensor>();
  ros::NodeHandle nh("~");
  ToFSensor tofsensor=ToFSensor(nh);
  ros::spin();
  ros::shutdown();
  return 0;
}
