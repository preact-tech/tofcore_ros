#
# examples/tof-therm
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-therm tof-therm.cpp)
target_link_libraries(tof-therm tofcrust Threads::Threads Boost::program_options)
