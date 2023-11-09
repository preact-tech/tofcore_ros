# ToFCore ROS1 Package

## To build

### Prerequisits

- A release of ROS installed and active. So far all testing has been done with with the Noetic release.

*Navigate to ros_nodes directory
  ```
  make provision_ros1
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 

*Navigate to ros_nodes directory
```
make ros1
```

### Testing

One run the node by running the following commands: 

Terminal 1:
```
cd ros1
source ./install/setup.bash
roscore
```
Terminal 2:
```
cd ros1
source ./install/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```
Terminal 3:

Default rviz config file located at ros1/rviz/default_config.rviz
```
cd ros1
source ./install/setup.bash
rosrun rviz rviz
```
Terminal 4:
```
cd ros1
source ./install/setup.bash
rosrun tofcore_ros1 tof_sensor
```
