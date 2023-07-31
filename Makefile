
.PHONY: pipeline
pipeline:
	$(MAKE) provision
	$(MAKE) release

.PHONY: provision
provision:
	vcs import < required.repos . --recursive
	rm -rf ros2/libtofcore
	ln -fs "${PWD}/libtofcore" "${PWD}/ros2/libtofcore"

.PHONY: release
release:
	cd ros2
	colcon build 
	cd ..

.PHONY: clean
clean:
	rm -rf build install log ros2/build ros2/install ros2/log

.PHONY: clobber
clobber: clean
	rm -rf libtofcore ros2/libtofcore