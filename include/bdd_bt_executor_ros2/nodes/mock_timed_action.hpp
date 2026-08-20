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

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace bdd_bt_executor_ros2 {

class MockTimedAction : public BT::StatefulActionNode
{
  public:
    MockTimedAction(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : BT::StatefulActionNode(name, config), node_(std::move(node))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("duration", "Duration in seconds"),
                 BT::InputPort<double>("heartbeat_duration", "Heartbeat period in seconds"),
                 BT::InputPort<std::string>("message", "Heartbeat message") };
    }

    BT::NodeStatus onStart() override
    {
        double duration;
        double heartbeat_duration;
        if (
          !getInput("duration", duration) || duration < 0.0
          || !getInput("heartbeat_duration", heartbeat_duration) || heartbeat_duration <= 0.0
          || !getInput("message", message_)
        ) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "MockTimedAction requires duration >= 0, heartbeat_duration > 0, and message"
            );
            return BT::NodeStatus::FAILURE;
        }

        const auto now = std::chrono::steady_clock::now();
        end_           = now
                         + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                           std::chrono::duration<double>(duration)
                         );
        heartbeat_     = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(heartbeat_duration)
        );
        next_heartbeat_ = now + heartbeat_;
        return duration == 0.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= end_) { return BT::NodeStatus::SUCCESS; }
        if (now >= next_heartbeat_) {
            RCLCPP_INFO(node_->get_logger(), "%s", message_.c_str());
            do {
                next_heartbeat_ += heartbeat_;
            } while (next_heartbeat_ <= now);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

  private:
    rclcpp::Node::SharedPtr               node_;
    std::string                           message_;
    std::chrono::steady_clock::time_point end_;
    std::chrono::steady_clock::time_point next_heartbeat_;
    std::chrono::steady_clock::duration   heartbeat_;
};

} // namespace bdd_bt_executor_ros2
