// Copyright 2026 Minh Nguyen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cmath>
#include <map>
#include <string>
#include <utility>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/action/move_group.hpp>

#include "bdd_bt_executor_ros2/robot_model.hpp"
#include "bdd_bt_executor_ros2/nodes/ros_action.hpp"
#include "bdd_bt_executor_ros2/nodes/get_entity_poses.hpp"

namespace bdd_bt_executor_ros2 {

class MoveGroupAction : public RosAction<moveit_msgs::action::MoveGroup>
{
  public:
    using Action = moveit_msgs::action::MoveGroup;
    MoveGroupAction(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : RosAction(name, config, std::move(node))
    {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Action::Goal>("goal"),
                 BT::InputPort<std::string>("group"),
                 BT::InputPort<std::string>("named_target"),
                 BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
                 BT::InputPort<std::string>("link_name"),
                 BT::InputPort<double>("position_tolerance", 0.005, "Position tolerance in metres"),
                 BT::InputPort<double>(
                   "orientation_tolerance", 0.05, "Orientation tolerance in radians"
                 ),
                 BT::InputPort<double>("planning_time", 5.0, "Target planning time"),
                 BT::InputPort<double>("velocity_scaling", 0.2, "Target velocity scaling"),
                 BT::InputPort<double>("acceleration_scaling", 0.2, "Target acceleration scaling"),
                 BT::InputPort<std::string>(
                   "action_name", "/move_action", "MoveGroup action server name"
                 ),
                 BT::InputPort<double>("timeout", 30.0, "Steady-clock timeout in seconds"),
                 BT::InputPort<double>(
                   "cancel_timeout", 2.0, "Callback retention after cancellation, seconds"
                 ),
                 BT::OutputPort<std::string>("feedback_state"),
                 BT::OutputPort<int>("error_code"),
                 BT::OutputPort<std::string>("status_message") };
    }

  private:
    bool readGoal(Action::Goal &goal, std::string &error) override
    {
        setOutput("feedback_state", std::string());
        setOutput("error_code", 0);
        const bool native = config().input_ports.count("goal") != 0;
        const bool named  = config().input_ports.count("named_target") != 0;
        const bool pose   = config().input_ports.count("target_pose") != 0;
        if (
          static_cast<int>(native) + static_cast<int>(named) + static_cast<int>(pose) != 1
          || (native && config().input_ports.count("group"))
        ) {
            error =
              "Specify exactly one of goal, group + named_target, or group + target_pose + "
              "link_name";
            return false;
        }
        if (named) {
            if (!readNamedGoal(goal, error)) return false;
        } else if (pose) {
            if (!readPoseGoal(goal, error)) return false;
        } else if (!getInput("goal", goal)) {
            error = "Invalid native goal";
            return false;
        }
        const auto &request = goal.request;
        auto        scaling = [](double value) {
            return std::isfinite(value) && value >= 0.0 && value <= 1.0;
        };
        if (
          request.group_name.empty() || request.goal_constraints.empty()
          || !std::isfinite(request.allowed_planning_time) || request.allowed_planning_time <= 0.0
          || !scaling(request.max_velocity_scaling_factor)
          || !scaling(request.max_acceleration_scaling_factor)
        ) {
            error =
              "MoveGroup goal requires a group, constraints, positive planning time, and scaling "
              "in [0,1]";
            return false;
        }
        return true;
    }

    bool readNamedGoal(Action::Goal &goal, std::string &error)
    {
        std::string group, target;
        if (
          !getInput("group", group) || group.empty() || !getInput("named_target", target)
          || target.empty()
        ) {
            error = "Named target requires nonempty group and named_target";
            return false;
        }
        const auto                    model       = loadRobotModel(node_);
        const auto                    joint_group = model->getJointModelGroup(group);
        std::map<std::string, double> positions;
        if (
          !joint_group || !joint_group->getVariableDefaultPositions(target, positions)
          || positions.empty()
        ) {
            error = "Unknown or empty named target: " + group + "/" + target;
            return false;
        }
        goal                               = Action::Goal();
        goal.request.group_name            = group;
        goal.request.start_state.is_diff   = true;
        goal.request.num_planning_attempts = 5;
        if (
          !getInput("planning_time", goal.request.allowed_planning_time)
          || !getInput("velocity_scaling", goal.request.max_velocity_scaling_factor)
          || !getInput("acceleration_scaling", goal.request.max_acceleration_scaling_factor)
        ) {
            error = "Invalid named target planning options";
            return false;
        }
        moveit_msgs::msg::Constraints constraints;
        constraints.name = target;
        for (const auto &[name, position] : positions) {
            const auto *joint = model->getJointOfVariable(name);
            if (!joint || joint->getVariableCount() != 1 || !std::isfinite(position)) {
                error = "Named targets require finite scalar joint positions";
                return false;
            }
            moveit_msgs::msg::JointConstraint constraint;
            constraint.joint_name      = joint->getName();
            constraint.position        = position;
            constraint.tolerance_above = constraint.tolerance_below = 0.001;
            constraint.weight                                       = 1.0;
            constraints.joint_constraints.push_back(constraint);
        }
        goal.request.goal_constraints.push_back(constraints);
        goal.planning_options.planning_scene_diff.is_diff             = true;
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
        return true;
    }

    bool readPoseGoal(Action::Goal &goal, std::string &error)
    {
        try {
            geometry_msgs::msg::PoseStamped target;
            std::string                     group, link;
            double                          position_tolerance, orientation_tolerance;
            if (
              !getInput("target_pose", target) || target.header.frame_id.empty()
              || !getInput("group", group) || group.empty() || !getInput("link_name", link)
              || link.empty() || !getInput("position_tolerance", position_tolerance)
              || !getInput("orientation_tolerance", orientation_tolerance)
              || !std::isfinite(position_tolerance) || position_tolerance <= 0
              || !std::isfinite(orientation_tolerance) || orientation_tolerance <= 0
            )
                throw std::invalid_argument(
                  "pose target requires a frame, group, link_name, and finite positive tolerances"
                );
            validatePose(target.pose);
            goal                               = Action::Goal();
            goal.request.group_name            = group;
            goal.request.start_state.is_diff   = true;
            goal.request.num_planning_attempts = 5;
            if (
              !getInput("planning_time", goal.request.allowed_planning_time)
              || !getInput("velocity_scaling", goal.request.max_velocity_scaling_factor)
              || !getInput("acceleration_scaling", goal.request.max_acceleration_scaling_factor)
            )
                throw std::invalid_argument("invalid pose target planning options");
            moveit_msgs::msg::Constraints        constraints;
            moveit_msgs::msg::PositionConstraint position;
            position.header    = target.header;
            position.link_name = link;
            position.weight    = 1;
            shape_msgs::msg::SolidPrimitive sphere;
            sphere.type       = sphere.SPHERE;
            sphere.dimensions = { position_tolerance };
            position.constraint_region.primitives.push_back(sphere);
            auto region          = target.pose;
            region.orientation.x = region.orientation.y = region.orientation.z = 0;
            region.orientation.w                                               = 1;
            position.constraint_region.primitive_poses.push_back(region);
            constraints.position_constraints.push_back(position);
            moveit_msgs::msg::OrientationConstraint orientation;
            orientation.header                      = target.header;
            orientation.link_name                   = link;
            orientation.orientation                 = target.pose.orientation;
            orientation.weight                      = 1;
            orientation.absolute_x_axis_tolerance   = orientation.absolute_y_axis_tolerance =
              orientation.absolute_z_axis_tolerance = orientation_tolerance;
            constraints.orientation_constraints.push_back(orientation);
            goal.request.goal_constraints.push_back(constraints);
            goal.planning_options.planning_scene_diff.is_diff             = true;
            goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
            return true;
        } catch (const std::exception &exception) {
            error = exception.what();
            return false;
        }
    }

    void writeFeedback(const Action::Feedback &feedback) override
    { setOutput("feedback_state", feedback.state); }

    bool writeResult(const Action::Result &result) override
    {
        setOutput("error_code", result.error_code.val);
        return result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }

    std::string feedbackMessage(const Action::Feedback &feedback) override
    { return feedback.state; }
};

class GripperCommandAction : public RosAction<control_msgs::action::GripperCommand>
{
  public:
    using Action = control_msgs::action::GripperCommand;
    GripperCommandAction(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : RosAction(name, config, std::move(node))
    {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("position"),
                 BT::InputPort<double>("max_effort", 0.0, "Maximum effort"),
                 BT::InputPort<std::string>(
                   "action_name", "/gripper_cmd", "GripperCommand action server name"
                 ),
                 BT::InputPort<double>("timeout", 30.0, "Steady-clock timeout in seconds"),
                 BT::InputPort<double>(
                   "cancel_timeout", 2.0, "Callback retention after cancellation, seconds"
                 ),
                 BT::OutputPort<double>("result_position"),
                 BT::OutputPort<bool>("reached_goal"),
                 BT::OutputPort<bool>("stalled"),
                 BT::OutputPort<std::string>("status_message") };
    }

  private:
    bool readGoal(Action::Goal &goal, std::string &error) override
    {
        setOutput("result_position", 0.0);
        setOutput("reached_goal", false);
        setOutput("stalled", false);
        if (
          !getInput("position", goal.command.position)
          || !getInput("max_effort", goal.command.max_effort)
          || !std::isfinite(goal.command.position) || goal.command.position < 0.0
          || !std::isfinite(goal.command.max_effort) || goal.command.max_effort < 0.0
        ) {
            error = "position and max_effort must be finite and nonnegative";
            return false;
        }
        return true;
    }

    void writeFeedback(const Action::Feedback &feedback) override
    { setOutput("result_position", feedback.position); }

    bool writeResult(const Action::Result &result) override
    {
        setOutput("result_position", result.position);
        setOutput("reached_goal", result.reached_goal);
        setOutput("stalled", result.stalled);
        return true; // Command completion; physical grasp is checked by observations.
    }

    std::string feedbackMessage(const Action::Feedback &feedback) override
    { return "gripper position " + std::to_string(feedback.position); }
};

} // namespace bdd_bt_executor_ros2
