cmake_minimum_required(VERSION 3.8)
project(nav_robot)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config urdf worlds maps behavior_trees web docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/patrol.py scripts/dashboard.py scripts/spawn_obstacle.sh scripts/gen_world.py scripts/make_demo_gif.py scripts/scan_watchdog.py scripts/explore.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
