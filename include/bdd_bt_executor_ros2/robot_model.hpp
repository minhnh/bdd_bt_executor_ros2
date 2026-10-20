// Copyright 2026 Minh Nguyen
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdexcept>
#include <string>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

namespace bdd_bt_executor_ros2 {

inline moveit::core::RobotModelPtr loadRobotModel(const rclcpp::Node::SharedPtr &node)
{
    std::string urdf, srdf;
    if (
      !node->get_parameter("robot_description", urdf) || urdf.empty()
      || !node->get_parameter("robot_description_semantic", srdf) || srdf.empty()
    ) {
        throw std::invalid_argument("Requires robot_description and robot_description_semantic");
    }
    robot_model_loader::RobotModelLoader::Options options(urdf, srdf);
    options.load_kinematics_solvers = false;
    robot_model_loader::RobotModelLoader loader(node, options);
    auto                                 model = loader.getModel();
    if (!model) throw std::invalid_argument("Invalid robot description");
    return model;
}

} // namespace bdd_bt_executor_ros2
