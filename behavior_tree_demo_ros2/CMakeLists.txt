cmake_minimum_required(VERSION 3.8)
project(behavior_tree_demo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    std_msgs 
    geometry_msgs
    nav2_behavior_tree
    nav2_msgs
    sensor_msgs
    tf2 
    tf2_ros
    tf2_geometry_msgs
    yaml-cpp
) 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
add_executable(bt_main src/bt_main.cpp)

install (TARGETS
  bt_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  behavior_tree_xml
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(bt_main ${dependencies})

ament_package()
