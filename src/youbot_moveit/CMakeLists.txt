cmake_minimum_required(VERSION 3.22)
project(youbot_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(graph_msgs REQUIRED)
  
set(dependencies
  rclcpp
  moveit_ros_planning_interface
  nav_msgs
  tf2
  tf2_ros
  graph_msgs  
)

add_executable(script scripts/script.cpp)
ament_target_dependencies(script ${dependencies})

ament_package()

install(TARGETS script
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
