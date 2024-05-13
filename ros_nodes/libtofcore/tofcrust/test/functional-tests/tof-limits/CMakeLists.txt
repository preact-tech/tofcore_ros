#
# examples/tof-limits
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-limits tof-limits.cpp)
target_link_libraries(tof-limits tofcrust Threads::Threads Boost::program_options)
