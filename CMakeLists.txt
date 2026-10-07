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
find_package(control_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

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
  control_msgs
  simulation_interfaces
  geometry_msgs)

target_link_libraries(${PROJECT_NAME} tinyxml2::tinyxml2)

add_executable(bdd_bt_action_server src/main.cpp)
target_link_libraries(bdd_bt_action_server ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS bdd_bt_action_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  bdd_ros2_interfaces
  rclcpp
  rclcpp_action
  moveit_msgs
  control_msgs
  simulation_interfaces
  geometry_msgs
  tinyxml2_vendor
  TinyXML2)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes ${PROJECT_NAME})
  ament_add_gtest(test_motion_actions test/test_motion_actions.cpp)
  target_link_libraries(test_motion_actions ${PROJECT_NAME})
  target_compile_definitions(test_motion_actions PRIVATE
    NAMED_GOALS_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/named_goals.xml")
  ament_add_gtest(test_entity_pose test/test_entity_pose.cpp)
  target_link_libraries(test_entity_pose ${PROJECT_NAME})
endif()

ament_package()
