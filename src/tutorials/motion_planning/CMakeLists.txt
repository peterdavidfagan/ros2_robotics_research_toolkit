cmake_minimum_required(VERSION 3.22)
project(panda_motion_planning_demos)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/motion_planning.py
  src/motion_planning_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
