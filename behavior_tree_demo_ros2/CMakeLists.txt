cmake_minimum_required(VERSION 3.8)
project(behavior_tree_demo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  behaviortree_cpp
) 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  #find_package(ament_lint_auto REQUIRED)
endif()


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
