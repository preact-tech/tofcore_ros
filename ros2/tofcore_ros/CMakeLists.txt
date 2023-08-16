cmake_minimum_required(VERSION 3.8)
project(tofcore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tofcore_msgs REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)


find_package(ToFCore)
if(NOT TARGET ToFCore::ToFCore)
  if (EXISTS "${CMAKE_SOURCE_DIR}/libtofcore" AND EXISTS "${CMAKE_SOURCE_DIR}/libtofcore/CMakeLists.txt")
    set(BUILD_PYTHON_BINDINGS OFF)
    add_subdirectory(libtofcore/tofcore)
  else()
    message(FATAL_ERROR "ToFCore library must be installed to ${CMAKE_INSTALL_PREFIX} OR present at ${CMAKE_SOURCE_DIR}/libtofcore")
  endif()
endif()


add_executable(tof_sensor src/sensor_node.cpp src/main.cpp)
target_include_directories(tof_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(tof_sensor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(tof_sensor rclcpp std_msgs sensor_msgs tofcore_msgs)
target_link_libraries(tof_sensor tofcore)
# rosidl_target_interfaces(tof_sensor
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS tof_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz2 config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
