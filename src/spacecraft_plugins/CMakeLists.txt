cmake_minimum_required(VERSION 3.16)
project(spacecraft_plugins)
cmake_policy(SET CMP0167 OLD)  # allow FindBoost


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)  # Gazebo Classic
find_package(ignition-math6 REQUIRED)
find_package(spacecraft_msgs REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
)

add_library(apply_force_plugin SHARED src/apply_force_plugin.cpp)
add_library(print_plugin SHARED src/print_plugin.cpp)


ament_target_dependencies(apply_force_plugin
  rclcpp
  geometry_msgs
  gazebo_ros
  ignition-math6
  spacecraft_msgs
)

ament_target_dependencies(print_plugin gazebo_ros)


# 🔧 Correct for Gazebo Classic:
target_link_libraries(apply_force_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS apply_force_plugin
  LIBRARY DESTINATION lib
)

install(TARGETS print_plugin
  LIBRARY DESTINATION lib
)

ament_package()
