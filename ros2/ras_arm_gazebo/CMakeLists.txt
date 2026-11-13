cmake_minimum_required(VERSION 3.16)
project(ras_arm_gazebo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/demo_cycle.py DESTINATION lib/${PROJECT_NAME})

ament_package()
