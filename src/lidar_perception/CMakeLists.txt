cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

cmake_policy(SET CMP0074 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(topic_tools REQUIRED)

include_directories(
  include
)

add_executable(lidar_perception_node src/lidar_perception_node.cpp)
target_include_directories(lidar_perception_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_perception_node ${PCL_LIBRARIES})
ament_target_dependencies(lidar_perception_node
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
  pcl_conversions
  pcl_ros
  topic_tools
)

install(TARGETS lidar_perception_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
