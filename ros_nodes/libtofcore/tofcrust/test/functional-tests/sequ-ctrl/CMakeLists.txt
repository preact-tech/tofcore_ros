#
# examples/sequ-ctrl
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(sequ-ctrl sequ-ctrl.cpp)
target_link_libraries(sequ-ctrl tofcrust Threads::Threads Boost::program_options)
