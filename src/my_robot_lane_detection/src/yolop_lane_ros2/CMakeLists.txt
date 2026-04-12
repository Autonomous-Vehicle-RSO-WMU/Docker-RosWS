cmake_minimum_required(VERSION 3.8)
project(yolop_lane_ros2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneData.msg"
)

install(PROGRAMS
  yolop_lane_ros2/yolop_lane_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolop_lane_node
)

install(PROGRAMS
  yolop_lane_ros2/video_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME video_pub
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
