cmake_minimum_required(VERSION 3.20)
project(bdd_bt_executor_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bdd_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/bdd_bt_executor.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  bdd_ros2_interfaces
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_ros_planning
  control_msgs
  simulation_interfaces
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(bdd_bt_action_server src/main.cpp)
target_link_libraries(bdd_bt_action_server ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS bdd_bt_action_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  bdd_ros2_interfaces
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_ros_planning
  control_msgs
  simulation_interfaces
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  yaml_cpp_vendor
  yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes ${PROJECT_NAME})
  ament_add_gtest(test_moveit_readiness test/test_moveit_readiness.cpp)
  target_link_libraries(test_moveit_readiness ${PROJECT_NAME})
  ament_add_gtest(test_motion_actions test/test_motion_actions.cpp)
  target_link_libraries(test_motion_actions ${PROJECT_NAME})
  target_compile_definitions(test_motion_actions PRIVATE
    NAMED_GOALS_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/named_goals.xml")
  ament_add_gtest(test_entity_pose test/test_entity_pose.cpp)
  target_link_libraries(test_entity_pose ${PROJECT_NAME})
  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp)
  target_link_libraries(test_planning_scene ${PROJECT_NAME})
  ament_add_gtest(test_scene_entities test/test_scene_entities.cpp)
  target_link_libraries(test_scene_entities ${PROJECT_NAME})
  target_compile_definitions(test_scene_entities PRIVATE
    EXAMPLE_ENTITY_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/example_entities.yaml")
endif()

ament_package()
