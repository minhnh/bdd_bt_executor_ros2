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

#include <string>
#include <utility>

#include <bdd_ros2_interfaces/msg/event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace bdd_bt_executor_ros2 {

class PublishBddEvent : public BT::SyncActionNode
{
  public:
    using Publisher = rclcpp::Publisher<bdd_ros2_interfaces::msg::Event>;

    PublishBddEvent(
      const std::string                &name,
      const BT::NodeConfig             &config,
      rclcpp::Node::SharedPtr           node,
      Publisher::SharedPtr              publisher,
      unique_identifier_msgs::msg::UUID scenario_context_id
    )
      : BT::SyncActionNode(name, config), node_(std::move(node)), publisher_(std::move(publisher)),
        scenario_context_id_(std::move(scenario_context_id))
    {}

    static BT::PortsList providedPorts()
    { return { BT::InputPort<std::string>("iri", "Full event IRI") }; }

    BT::NodeStatus tick() override
    {
        std::string iri;
        if (!getInput("iri", iri) || iri.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "PublishBddEvent: missing or empty input port [iri]");
            return BT::NodeStatus::FAILURE;
        }

        bdd_ros2_interfaces::msg::Event event;
        event.scenario_context_id = scenario_context_id_;
        event.stamp               = node_->get_clock()->now();
        event.uri                 = iri;
        publisher_->publish(event);
        return BT::NodeStatus::SUCCESS;
    }

  private:
    rclcpp::Node::SharedPtr           node_;
    Publisher::SharedPtr              publisher_;
    unique_identifier_msgs::msg::UUID scenario_context_id_;
};

} // namespace bdd_bt_executor_ros2
