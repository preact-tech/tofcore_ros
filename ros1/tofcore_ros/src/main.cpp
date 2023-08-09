#include "sensor_node.hpp"
#include <ros/ros.h>
#include <memory>

int main(int argc, char ** argv)
{
  ros::init(argc, argv);
  ros::spin(std::make_shared<ToFSensor>());
  ros::shutdown();
  return 0;
}
