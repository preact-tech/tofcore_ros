cmake_minimum_required(VERSION 3.8)
project(t10)

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
find_package(truesense_msgs REQUIRED)

find_package(T10Utils)
if(NOT TARGET T10Utils::T10Utils)
  if (EXISTS "${CMAKE_SOURCE_DIR}/libt10" AND EXISTS "${CMAKE_SOURCE_DIR}/libt10/CMakeLists.txt")
    add_subdirectory(libt10)
  else()
    message(FATAL_ERROR "T10Utils library must be installed to ${CMAKE_INSTALL_PREFIX} OR present at ${CMAKE_SOURCE_DIR}/libt10")
  endif()
endif()


add_executable(t10_sensor src/sensor_node.cpp src/main.cpp)
target_include_directories(t10_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(t10_sensor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(t10_sensor rclcpp std_msgs sensor_msgs truesense_msgs)
target_link_libraries(t10_sensor t10utils) # T10Utils::T10Utils)

install(TARGETS t10_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz2
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
