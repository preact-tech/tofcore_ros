#
# examples/tof-log
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(tof-log tof-log.cpp)
target_link_libraries(tof-log tofcrust Threads::Threads Boost::program_options)
target_include_directories(tof-log PRIVATE ../../../src)
