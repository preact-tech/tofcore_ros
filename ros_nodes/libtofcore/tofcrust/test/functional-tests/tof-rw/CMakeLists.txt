#
# examples/tof-rw
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(tof-rw tof-rw.cpp)
target_link_libraries(tof-rw tofcrust Threads::Threads Boost::program_options)
target_include_directories(tof-rw PRIVATE ../../../src)
