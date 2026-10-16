// Copyright 2026 Minh Nguyen
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <moveit_msgs/action/move_group.hpp>
#include "bdd_bt_executor_ros2/robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace bdd_bt_executor_ros2 {

class WaitForMoveItReady : public BT::StatefulActionNode
{
  public:
    using Action = moveit_msgs::action::MoveGroup;
    WaitForMoveItReady(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : BT::StatefulActionNode(name, config), node_(std::move(node))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action_name", "/move_action", "MoveGroup endpoint"),
            BT::InputPort<std::string>("joint_states_topic", "/joint_states", "Joint state topic"),
            BT::InputPort<std::string>("group"),
            BT::InputPort<std::string>("target_frame"),
            BT::InputPort<std::string>("end_effector_frame"),
            BT::InputPort<double>("max_age", 1.0, "Maximum source age in node-clock seconds"),
            BT::InputPort<double>("timeout", 30.0, "Node-clock timeout in seconds"),
            BT::OutputPort<std::string>("status_message")
        };
    }

    BT::NodeStatus onStart() override
    {
        onHalted();
        try {
            std::string endpoint, topic, group;
            if (
              !getInput("action_name", endpoint) || endpoint.empty()
              || !getInput("joint_states_topic", topic) || topic.empty()
              || !getInput("group", group) || group.empty() || !getInput("target_frame", target_)
              || target_.empty() || !getInput("end_effector_frame", tool_) || tool_.empty()
              || !getInput("max_age", max_age_) || !std::isfinite(max_age_) || max_age_ <= 0
              || !getInput("timeout", timeout_) || !std::isfinite(timeout_) || timeout_ <= 0
            ) {
                return finish(
                  BT::NodeStatus::FAILURE,
                  "require endpoints, frames, group and positive finite max_age/timeout"
                );
            }
            const auto model       = loadRobotModel(node_);
            const auto joint_group = model ? model->getJointModelGroup(group) : nullptr;
            if (!joint_group || joint_group->getActiveJointModels().empty()) {
                return finish(
                  BT::NodeStatus::FAILURE, "Unknown or empty active joint group: " + group
                );
            }
            joints_.clear();
            for (const auto *joint : joint_group->getActiveJointModels()) {
                if (joint->getVariableCount() != 1) {
                    return finish(
                      BT::NodeStatus::FAILURE,
                      "JointState readiness requires scalar joints: " + joint->getName()
                    );
                }
                joints_.push_back(joint->getName());
            }
            started_ = last_time_ = node_->now().seconds();
            state_                = std::make_shared<JointSamples>();
            subscription_         = node_->create_subscription<sensor_msgs::msg::JointState>(
              topic,
              rclcpp::SensorDataQoS(),
              [state = state_](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) return;
                  std::lock_guard<std::mutex> lock(state->mutex);
                  const double                stamp = rclcpp::Time(msg->header.stamp).seconds();
                  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                      if (std::isfinite(msg->position[i])) state->stamps[msg->name[i]] = stamp;
                  }
              }
            );
            buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, node_, false);
            client_   = rclcpp_action::create_client<Action>(node_, endpoint);
            return onRunning();
        } catch (const std::exception &error) {
            return finish(BT::NodeStatus::FAILURE, error.what());
        }
    }

    BT::NodeStatus onRunning() override
    {
        const double now = node_->now().seconds();
        if (now < last_time_) {
            // Discard samples and deadlines from the previous simulation timeline.
            started_ = now;
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->stamps.clear();
            buffer_->clear();
        }
        last_time_ = now;
        std::string missing;
        if (!client_->action_server_is_ready()) missing = "MoveGroup action server unavailable";
        if (missing.empty()) {
            std::lock_guard<std::mutex> lock(state_->mutex);
            for (const auto &joint : joints_) {
                const auto found = state_->stamps.find(joint);
                if (found == state_->stamps.end() || !fresh(found->second, now)) {
                    missing = "missing fresh joint state: " + joint;
                    break;
                }
            }
        }
        if (missing.empty()) {
            try {
                const auto transform = buffer_->lookupTransform(target_, tool_, tf2::TimePointZero);
                if (!fresh(rclcpp::Time(transform.header.stamp).seconds(), now)) {
                    missing = "missing fresh TF: " + target_ + " -> " + tool_;
                }
            } catch (const tf2::TransformException &) {
                missing = "missing TF: " + target_ + " -> " + tool_;
            }
        }
        if (missing.empty()) return finish(BT::NodeStatus::SUCCESS, "MoveIt ready");
        if (now - started_ >= timeout_) {
            return finish(BT::NodeStatus::FAILURE, "timed out: " + missing);
        }
        message("waiting: " + missing);
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        subscription_.reset();
        listener_.reset();
        buffer_.reset();
        client_.reset();
        state_.reset();
    }

  private:
    struct JointSamples
    {
        std::mutex                    mutex;
        std::map<std::string, double> stamps;
    };
    bool fresh(double stamp, double now) const
    { return stamp > 0 && stamp >= started_ && stamp <= now && now - stamp <= max_age_; }
    void message(const std::string &text)
    {
        setOutput("status_message", text);
        config().blackboard->set("behaviour_status", name() + ": " + text);
    }
    BT::NodeStatus finish(BT::NodeStatus status, const std::string &text)
    {
        message(text);
        onHalted();
        return status;
    }
    rclcpp::Node::SharedPtr                                       node_;
    rclcpp_action::Client<Action>::SharedPtr                      client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::Buffer>                              buffer_;
    std::unique_ptr<tf2_ros::TransformListener>                   listener_;
    std::shared_ptr<JointSamples>                                 state_;
    std::vector<std::string>                                      joints_;
    std::string                                                   target_, tool_;
    double started_{}, last_time_{}, max_age_{}, timeout_{};
};
} // namespace bdd_bt_executor_ros2
