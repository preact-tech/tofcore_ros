# ToFCore ROS support

Project to build ROS packages for the sensors that work with libtofcore

Further details on execution can be found at [ros1/README.md](ros1/README.md) or [ros2/README.md](ros2/README.md)

## Building Ros1

### Prerequisits

- A release of ROS installed and active. So far all testing has been done with
  with the Noetic release.
  ```
  make provision_ros1
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
make ros1
```

## Building Ros2

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.

  ```
  make provision
  ```

### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
make ros2
cd ros2
source ./install/setup.bash
```