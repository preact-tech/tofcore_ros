#
# examples/tof-sdram-test
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(tof-sdram-test tof-sdram-test.cpp)
target_link_libraries(tof-sdram-test tofcrust Threads::Threads  Boost::program_options)
#target_include_directories(tof-sdram-test PRIVATE ../../../src)
