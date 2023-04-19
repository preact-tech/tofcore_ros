
.PHONY: pipeline
pipeline:
	$(MAKE) provision
	$(MAKE) release

.PHONY: provision
provision:
	vcs import < required.repos . --recursive
	ln -fs ../libtofcore ros2/libtofcore

.PHONY: release
release:
	colcon build 

.PHONY: libtofcore
libtofcore:
	cd ros2/libtofcore && cmake -B build && cd build && make && sudo make install

.PHONY: clean
clean:
	rm -rf build install log ros2/build ros2/install ros2/log

.PHONY: clobber
clobber: clean
	rm -rf libtofcore ros2/libtofcore