cmake_minimum_required(VERSION 3.8)
project(bitt_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ackermann_cmd_node.py
  scripts/sim_esp.py
  scripts/sim_odom.py
  scripts/sim_camera_info.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
