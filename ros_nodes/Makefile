SHELL := /bin/bash
# Version info for appImage
VERSION_MAJOR ?= 0
VERSION_MINOR ?= 0
BITBUCKET_BUILD_NUMBER ?= 0
REPO_REF ?= develop
export VERSION_MAJOR
export VERSION_MINOR
export BITBUCKET_BUILD_NUMBER
export REPO_REF

TARGET_DISTRO_VER ?= $(shell lsb_release -sc)
SET_ROS1=source /opt/ros/noetic/setup.bash 
SET_ROS2=source /opt/ros/galactic/setup.bash 

.PHONY: pipeline
pipeline:
	$(MAKE) provision
	$(MAKE) build



.PHONY: provision
provision:
	vcs import < required.repos . --recursive
	vcs --nested custom --git --args show
	rm -rf ros2/tofcore_ros/libtofcore ros2/tofcore_discovery/libtofcore ros1/tofcore_ros/libtofcore
	cd ros2 && ln -fs ../../libtofcore tofcore_ros/libtofcore

.PHONY: provision_bridge
provision_bridge: provision ##	Install required tools, packages and git repos to build the truenense package as well as the ROS bridge package
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(TARGET_DISTRO_VER) main" > /etc/apt/sources.list.d/ros-noetic.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update && sudo apt install -y ros-noetic-ros-base
	- git clone -b master https://github.com/ros2/ros1_bridge.git

.PHONY: provision_ros1
provision_ros1: provision ##	Install required tools, packages and git repos to build the truenense package as well as the ROS bridge package
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(TARGET_DISTRO_VER) main" > /etc/apt/sources.list.d/ros-noetic.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update && sudo apt install -y ros-noetic-desktop-full python3-catkin-pkg
	cd ros1 && ln -fs ../../libtofcore tofcore_ros/libtofcore


.PHONY: ros2
ros2:
	 PREACT_CPPCHECK_ENABLE="OFF" && cd ros2 && colcon build --packages-skip ros1_bridge preact_alg detection_py lens_calibration 
	 
.PHONY: ros1
ros1:
	 PREACT_CPPCHECK_ENABLE="OFF" && cd ros1 &&  source /opt/ros/noetic/setup.bash && colcon build --parallel-workers 20

.PHONY: bridge
bridge: build ##	Build the ROS1 bridge, build environment must have both ROS1 and ROS2 installed
	$(SET_ROS1) &> /dev/null; $(SET_ROS2) &> /dev/null; colcon build --packages-select ros1_bridge --cmake-force-configure


.PHONY: clean
clean:
	rm -rf build install log ros2/build ros2/install ros2/log ros1/build ros1/tofcore_ros/build ros1/install ros1/log
	rm -r -f ros2/build
	rm -r -f dist
	rm -r -f ros2/install
	rm -r -f ros2/log
	rm -r -f ros1/build
	rm -r -f ros1/install
	rm -r -f ros1/log
	rm -r -f target
	rm -r -f ros2/debian
	rm -r -f ros2/.obj-x86_64-linux-gnu
	rm -r -f ros2/target
	rm -r -f tofcore_msgs/debian
	rm -r -f tofcore_msgs/.obj-x86_64-linux-gnu
	rm -r -f ros-galactic-ros1-bridge* 
	rm -r -f ros-galactic-tofcore* 
	rm -r -f AppDir
	rm -r -f appimage-builder-cache
	rm -r -f *.AppImage
	rm -r -f *.zsync
.PHONY: clobber
clobber: clean
	rm -rf ros2/tofcore_ros/libtofcore ros1/tofcore_ros/libtofcore algorithm ros2/algorithm
	rm -r -f ros1_bridge
