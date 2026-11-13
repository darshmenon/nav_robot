cmake_minimum_required(VERSION 3.8)
project(nav_robot)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config urdf worlds maps behavior_trees web DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/patrol.py scripts/dashboard.py scripts/spawn_obstacle.sh scripts/gen_world.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
