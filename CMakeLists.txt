cmake_minimum_required(VERSION 3.5)
project(ros2_behavior_tree_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Needs C++17 for BT
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  generate_parameter_library
  behaviortree_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_srvs
  tf2_ros
)

# for each dependency, find and include the package
foreach(DEPENDENCY ${THIS_PACKAGE_DEPS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(Boost COMPONENTS coroutine QUIET)

generate_parameter_library(
  bt_parameters
  config/parameters.yaml
)

set(executable_name behavior_tree_example)
set(library_name ${PROJECT_NAME})

add_executable(
  ${executable_name}
  src/main.cpp
)

add_library(
  ${library_name} SHARED
  src/plugins/pong_received_bt_node.cpp
  src/plugins/pong_received_executor_bt_node.cpp
  src/plugins/ping_bt_node.cpp
  src/plugins/log_status_bt_node.cpp
  src/bt_ros_node.cpp
)

rclcpp_components_register_nodes(${library_name} "polymath::bt_ros_example::BtRosNode")

include_directories(
  include
)

ament_target_dependencies(
  ${executable_name}
  ${THIS_PACKAGE_DEPS}
)

target_link_libraries(${executable_name} ${library_name} bt_parameters)

ament_target_dependencies(
  ${library_name}
  ${THIS_PACKAGE_DEPS}
)

target_link_libraries(${PROJECT_NAME}  bt_parameters)

#target_include_directories(${PROJECT_NAME} PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)

install(
  TARGETS ${library_name} bt_parameters
  EXPORT ${library_name}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME}Targets  HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()
