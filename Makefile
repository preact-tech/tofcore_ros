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
	rm -rf ros2/tofcore_ros/libtofcore ros1/tofcore_ros/libtofcore
	cd ros2 && ln -fs ../../libtofcore tofcore_ros/libtofcore
	cd ros1 && ln -fs ../../libtofcore tofcore_ros/libtofcore

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
	

tofcore_msgs_deb: ##	Build the tofcore_msgs ROS2 package as a debian package
	rosdep update --include-eol-distros
	cd tofcore_msgs; bloom-generate rosdebian
	cd tofcore_msgs; fakeroot debian/rules binary
	sudo apt install ./ros-galactic-tofcore-msgs_${VERSION_MAJOR}.${VERSION_MINOR}.${BITBUCKET_BUILD_NUMBER}-0focal_amd64.deb

tofcore_deb: tofcore_msgs_deb ##	Build the truensense ROS2 package as a debian package
	echo "tofcore_msgs:" > /tmp/rosdep.yaml
	echo "  ubuntu: [ros-galactic-tofcore-msgs]" >> /tmp/rosdep.yaml
	echo "yaml file:///tmp/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-my-packages.list
	rosdep update --include-eol-distros
	cd ros2; bloom-generate rosdebian
	cd ros2; fakeroot debian/rules binary

.PHONY: build
build:
	 cd ros2 && colcon build --packages-skip ros1_bridge 

.PHONY: ros1
ros1:
	 cd ros1 &&  source /opt/ros/noetic/setup.bash && colcon build


.PHONY: bridge
bridge: build ##	Build the ROS1 bridge, build environment must have both ROS1 and ROS2 installed
	$(SET_ROS1) &> /dev/null; $(SET_ROS2) &> /dev/null; colcon build --packages-select ros1_bridge --cmake-force-configure


.PHONY: clean
clean:
	rm -rf build install log ros2/build ros2/install ros2/log ros1/build ros1/tofcore_ros/build ros1/install ros1/log
	rm -r -f build
	rm -r -f dist
	rm -r -f install
	rm -r -f log
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
	rm -rf libtofcore ros2/tofcore_ros/libtofcore
	rm -rf libtofcore ros1/tofcore_ros/libtofcore
	rm -r -f ros1_bridge
