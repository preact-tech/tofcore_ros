# tofcore_ros
Sample nodes for interacting with PreAct ToF sensors in Ros1 and Ros2 


## Prerequisites


Clone libtofcore repo in the same directory as this 
```
cd ros_nodes/ros2 && ln -fs ../../../libtofcore tofcore_ros/libtofcore
cd ../..
cd ros_nodes/ros1 && ln -fs ../../../libtofcore tofcore_ros/libtofcore
cd ../..
```
Sourced appropriate ros setup.bash script.

## Building

### Ros1
```
cd ros_nodes/ros1
colcon build
```

### Ros2
```
cd ros_nodes/ros1
colcon build
```