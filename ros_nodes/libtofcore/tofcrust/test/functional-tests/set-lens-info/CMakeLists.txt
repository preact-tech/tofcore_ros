#
# examples/set-lens-info
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(set-lens-info set-lens-info.cpp)
target_link_libraries(set-lens-info tofcrust Threads::Threads Boost::program_options)
