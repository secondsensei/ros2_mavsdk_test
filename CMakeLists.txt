cmake_minimum_required(VERSION 3.8)
project(ros2_mavsdk_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Find MAVSDK using CMake (preferred method since config files exist)
find_package(MAVSDK REQUIRED)

# Create MAVSDK Mission Diagnostics executable
add_executable(mavsdk_mission_diagnostics src/mavsdk_mission_diagnostics.cpp)

# Add ROS2 dependencies
ament_target_dependencies(mavsdk_mission_diagnostics
  rclcpp
  rmw
  px4_msgs
  sensor_msgs
  geometry_msgs
)

# Link MAVSDK library (using the CMake target)
target_link_libraries(mavsdk_mission_diagnostics MAVSDK::mavsdk)

# Install executables
install(TARGETS
  mavsdk_mission_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()