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

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <bdd_ros2_interfaces/action/behaviour.hpp>
#include <bdd_ros2_interfaces/msg/event.hpp>
#include <bdd_ros2_interfaces/msg/trinary_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace bdd_bt_executor_ros2 {

class BddBtExecutor : public rclcpp::Node
{
  public:
    using BehaviourAction = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandle      = rclcpp_action::ServerGoalHandle<BehaviourAction>;
    using RegisterNodes =
      std::function<void(BT::BehaviorTreeFactory &, const rclcpp::Node::SharedPtr &)>;

    explicit BddBtExecutor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void set_node_registrar(RegisterNodes registrar);

  private:
    void execute_goal(const std::shared_ptr<GoalHandle> &goal_handle);
    bdd_ros2_interfaces::msg::TrinaryStamped
      make_result(const std::shared_ptr<GoalHandle> &goal_handle, uint8_t value) const;

    rclcpp_action::Server<BehaviourAction>::SharedPtr             action_server_;
    rclcpp::Publisher<bdd_ros2_interfaces::msg::Event>::SharedPtr event_publisher_;
    RegisterNodes                                                 register_nodes_;
    std::atomic<bool>                                             tree_running_{ false };
    std::string                                                   tree_xml_;
    int                                                           tick_rate_hz_;
};

} // namespace bdd_bt_executor_ros2
