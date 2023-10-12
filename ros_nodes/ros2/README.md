# ToFCore ROS2 Package

## To build

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.

*Navigate to ros_nodes directory
  ```
  make provision
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 

*Navigate to ros_nodes directory
```
make ros2
```

### Testing

One can quickly view a pointcloud generated from distance data generated on the device using the `tofcore.launch.py` launch file like so: 

```
cd ros2
source ./install/setup.bash
ros2 launch truesense tofcore.launch.py
```