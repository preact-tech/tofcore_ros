# T10 ROS2 Package

## To build

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.
- [t10utils](https://bitbucket.org/preact-tech/libt10/src/develop/) library built and/or installed.
- truesense_msgs ROS package installed, this can be done by installing a .deb package release or cloning and building the source
  as part of your ROS workspace.

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
colcon build
source ./install/setup.bash
```

### Testing

One can quickly view a pointcloud generated from distance data generated on the device using the `pcd-test.launch.py` launch file like so: 

```
ros2 launch t10 pcd-test.launch.py
```

You can use `dcs-test.launch.py` to test streaming DCS frames from the device and have the `truesense_processor` node generate distance frames
on the PC. This same pattern can be used to test other processing algorithms that have been turned into ROS nodes. 


A couple of things to note:

- The ROS node assumes the IP address of the device is 10.10.31.180 and there currently isn't any way to change this.
  You do need to make sure your PC ethernet devices is correctly configured for this subnet. 
