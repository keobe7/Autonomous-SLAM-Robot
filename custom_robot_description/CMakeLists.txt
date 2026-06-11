cmake_minimum_required(VERSION 3.8)
project(custom_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY urdf launch worlds config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/frontier_exploration.py
    scripts/frontier_search.py
    scripts/mission.py
    scripts/mission2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
