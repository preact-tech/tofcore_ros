# ToFCore ROS2 Package

## To build

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.
- [libtofcore] cloned to the subdirectory ./libtofcore .
  This can be achieved using the `make provision` command
  ```
  make provision
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
make release
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
rosrun --prefix 'gdb --args' tofcore_ros1 tof_sensor
```

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin_make -DCMAKE_BUILD_TYPE=Debug
