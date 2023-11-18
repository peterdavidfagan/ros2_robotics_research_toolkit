cmake_minimum_required(VERSION 3.22)
project(lite6_motion_planning_demos)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/blend_plans.py
  src/motion_planning.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
