cmake_minimum_required(VERSION 3.5)
project(agv_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# =====================================================
#  Find Dependencies
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveCmd.action"
)

# =====================================================
#  Build C++ Executables
# =====================================================

# Node 1: robot_controller — đọc bàn phím, publish /cmd_vel
add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller
  rclcpp
  geometry_msgs
)

# Node Action Server: Điều khiển xe theo lệnh
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action geometry_msgs nav_msgs)
target_include_directories(action_server PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(action_server "${cpp_typesupport_target}")

# Node Action Client: Gửi lệnh lên server
add_executable(action_robot src/action_robot.cpp)
ament_target_dependencies(action_robot rclcpp rclcpp_action)
target_link_libraries(action_robot "${cpp_typesupport_target}")

# Node 2: tf2_listener — TF2 Buffer + TransformListener
add_executable(tf2_listener src/tf2_listener.cpp)
ament_target_dependencies(tf2_listener
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
)
target_include_directories(tf2_listener PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Node 3: slam_robot — Graph SLAM / Pose Graph Optimization + tránh vật cản
file(GLOB_RECURSE GRAPH_SLAM_SRCS "library/SLAM_Graph_Based/cpp/*.cpp")
file(GLOB_RECURSE DWA_SRCS "library/DWA/cpp/*.cpp")
file(GLOB_RECURSE FRONTIER_SRCS "library/frontier_based/cpp/*.cpp")
file(GLOB_RECURSE ASTAR_SRCS "library/A_star_algorithm/cpp/*.cpp")
add_executable(slam_robot src/slam_robot.cpp ${GRAPH_SLAM_SRCS} ${DWA_SRCS} ${FRONTIER_SRCS} ${ASTAR_SRCS})
ament_target_dependencies(slam_robot
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_include_directories(slam_robot PUBLIC
  library/SLAM_Graph_Based/include
  library/DWA/include
  library/frontier_based/include
  library/A_star_algorithm/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Node 4: navigation_robot — Gửi mục tiêu đến Nav2
add_executable(navigation_robot src/navigation_robot.cpp)
ament_target_dependencies(navigation_robot
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
)

# =====================================================
#  Install
# =====================================================

# Install compiled C++ executables
install(TARGETS
  robot_controller
  tf2_listener
  action_server
  action_robot
  slam_robot
  navigation_robot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, urdf, rviz directories
install(DIRECTORY
  launch
  urdf
  rviz
  worlds
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
