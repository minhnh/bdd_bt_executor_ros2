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
#include <string>
#include <sstream>
#include <locale>
#include <set>
#include <tinyxml2.h>
#include <utility>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/action/move_group.hpp>

#include "bdd_bt_executor_ros2/nodes/ros_action.hpp"

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
        return {
            BT::InputPort<Action::Goal>("goal"),
            BT::InputPort<std::string>("group"),
            BT::InputPort<std::string>("named_target"),
            BT::InputPort<double>("planning_time", 5.0, "Named target planning time"),
            BT::InputPort<double>("velocity_scaling", 0.2, "Named target velocity scaling"),
            BT::InputPort<double>("acceleration_scaling", 0.2, "Named target acceleration scaling"),
            BT::InputPort<std::string>(
              "action_name", "/move_action", "MoveGroup action server name"
            ),
            BT::InputPort<double>("timeout", 30.0, "Steady-clock timeout in seconds"),
            BT::InputPort<double>(
              "cancel_timeout", 2.0, "Callback retention after cancellation, seconds"
            ),
            BT::OutputPort<std::string>("feedback_state"),
            BT::OutputPort<int>("error_code"),
            BT::OutputPort<std::string>("status_message")
        };
    }

  private:
    bool readGoal(Action::Goal &goal, std::string &error) override
    {
        setOutput("feedback_state", std::string());
        setOutput("error_code", 0);
        const bool native = config().input_ports.count("goal") != 0;
        const bool named  = config().input_ports.count("group") != 0
                            || config().input_ports.count("named_target") != 0;
        if (native && named) {
            error = "Specify either goal or group + named_target, not both";
            return false;
        }
        if (named) {
            if (!readNamedGoal(goal, error)) { return false; }
        } else if (!getInput("goal", goal)) {
            error = "Provide a native goal or group + named_target";
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
        std::string group, target, srdf;
        if (
          !getInput("group", group) || group.empty() || !getInput("named_target", target)
          || target.empty()
        ) {
            error = "Named target requires nonempty group and named_target";
            return false;
        }
        if (!node_->get_parameter("robot_description_semantic", srdf) || srdf.empty()) {
            error = "Named target requires robot_description_semantic on the executor";
            return false;
        }
        tinyxml2::XMLDocument document;
        if (
          document.Parse(srdf.c_str()) != tinyxml2::XML_SUCCESS
          || !document.FirstChildElement("robot")
        ) {
            error = "Invalid robot_description_semantic XML";
            return false;
        }
        const tinyxml2::XMLElement *selected = nullptr;
        for (auto state = document.FirstChildElement("robot")->FirstChildElement("group_state");
             state;
             state = state->NextSiblingElement("group_state")) {
            if (
              state->Attribute("group") && state->Attribute("name")
              && group == state->Attribute("group") && target == state->Attribute("name")
            ) {
                if (selected) {
                    error = "Duplicate named target in SRDF";
                    return false;
                }
                selected = state;
            }
        }
        if (!selected) {
            error = "Unknown named target: " + group + "/" + target;
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
        std::set<std::string> joints;
        for (auto joint = selected->FirstChildElement("joint"); joint;
             joint      = joint->NextSiblingElement("joint")) {
            const auto                        name  = joint->Attribute("name");
            const auto                        value = joint->Attribute("value");
            moveit_msgs::msg::JointConstraint constraint;
            // SRDF decimals always use a dot, independently of the process locale.
            std::istringstream input(value ? value : "");
            input.imbue(std::locale::classic());
            if (
              !name || !*name || !joints.insert(name).second || !(input >> constraint.position)
              || !std::isfinite(constraint.position) || !(input >> std::ws).eof()
            ) {
                error = "Named targets require unique joints with one finite scalar position";
                return false;
            }
            constraint.joint_name      = name;
            constraint.tolerance_above = constraint.tolerance_below = 0.001;
            constraint.weight                                       = 1.0;
            constraints.joint_constraints.push_back(constraint);
        }
        if (constraints.joint_constraints.empty()) {
            error = "Empty named target";
            return false;
        }
        goal.request.goal_constraints.push_back(constraints);
        goal.planning_options.planning_scene_diff.is_diff             = true;
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
        return true;
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
