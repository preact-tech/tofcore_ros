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
source ros1/devel/setup.bash
roslaunch tofcore_ros1 tofcore.launch
```

Stream two sensor with the following command:
```
source ros1/devel/setup.bash
roslaunch tofcore_ros1 tofcore_dual.launch 
```