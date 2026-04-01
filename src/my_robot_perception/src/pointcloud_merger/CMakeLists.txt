cmake_minimum_required(VERSION 3.8)
project(pointcloud_merger)

cmake_policy(SET CMP0074 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp)
target_include_directories(pointcloud_merger_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_merger_node ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_merger_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS pointcloud_merger_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
