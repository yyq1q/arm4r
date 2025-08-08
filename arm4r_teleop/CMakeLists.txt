cmake_minimum_required(VERSION 3.5)
project(arm4r_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  arm4r_teleop.py
  arm4r_teleop_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()