#
# examples/jmp-to-boot
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(ib-test ib-test.cpp)
target_link_libraries(ib-test tofcrust Threads::Threads Boost::program_options)
