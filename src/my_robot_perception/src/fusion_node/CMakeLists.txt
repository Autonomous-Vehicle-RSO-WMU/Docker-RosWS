cmake_minimum_required(VERSION 3.8)
project(fusion_node)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(fusion_node)

install(PROGRAMS
  fusion_node/fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fusion_node
)

install(DIRECTORY config launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()