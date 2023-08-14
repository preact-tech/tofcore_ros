# ToFCore ROS1 Package

## To build

### Prerequisits

- A release of ROS installed and active. So far all testing has been done with
  with the Noetic release.
- [libtofcore] cloned to the subdirectory ./libtofcore .
  This can be achieved using the `make provision_ros1` command
  ```
  make provision_ros1
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
make ros1
cd ros1
source ./install/setup.bash
```

### Testing

One can quickly view a pointcloud generated from distance data generated on the device using the `tofcore.launch.py` launch file like so: 

Terminal 1:
```
roscore
```
Terminal 2:
```
rosrun rqt_reconfigure rqt_reconfigure
```
Terminal 3:
```
rosrun tofcore_ros1 tof_sensor
```
