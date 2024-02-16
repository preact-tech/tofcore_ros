#
# examples/tof-edge
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(tof-edge tof-edge.cpp)
target_link_libraries(tof-edge tofcore Threads::Threads Boost::program_options)
