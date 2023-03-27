# ToFCore ROS2 Package

## To build

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.
- [libtofcore](https://bitbucket.org/preact-tech/libtofcore/src/develop/) either built and installed to CMAKE_INSTALL_PREFIX or cloned to the subdirectory ./libtofcore .

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
colcon build
source ./install/setup.bash
```

### Testing

One can quickly view a pointcloud generated from distance data generated on the device using the `tofcore.launch.py` launch file like so: 

```
ros2 launch tofcore tofcore.launch.py
```