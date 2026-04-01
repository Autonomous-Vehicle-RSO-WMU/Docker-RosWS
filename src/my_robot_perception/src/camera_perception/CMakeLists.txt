cmake_minimum_required(VERSION 3.8)
project(camera_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(camera_perception)

install(PROGRAMS
  camera_perception/camera_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_perception_node
)

install(PROGRAMS
  camera_perception/zed_od_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zed_od_adapter_node
)

install(DIRECTORY config launch resource models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()