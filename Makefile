
.PHONY: pipeline
pipeline:
	$(MAKE) provision
	$(MAKE) release

.PHONY: provision
provision:
	vcs import < required.repos ros2 --recursive
	cd ros2/libtofcore && cmake -B build
	cd ros2/libtofcore/build && make && sudo make install

.PHONY: release
release:
	colcon build 

.PHONY: clean
clean:
	rm -rf build install

.PHONY: clobber
clobber: clean
	rm -rf ros2/build ros2/install ros2/libtofcore