cmake_minimum_required(VERSION 3.22)
project(lbr_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)


add_executable(hello_moveit src/hello_moveit.cpp)

ament_target_dependencies(hello_moveit
  geometry_msgs  
  moveit_ros_planning_interface
  rclcpp
  moveit_msgs
  sensor_msgs
  std_msgs
  tf2
  moveit_core
  moveit_ros_planning
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
