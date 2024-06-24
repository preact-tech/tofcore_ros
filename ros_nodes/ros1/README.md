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

Stream a single sensor with the following command:
```
cd ros1
source ./install/setup.bash
roslaunch ros1/tofcore_ros/launch/tofcore.launch 
```

Stream two sensor with the following command:
```
cd ros1
source ./install/setup.bash
roslaunch ros1/tofcore_ros/launch/tofcore_dual.launch 
```