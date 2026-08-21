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

#include "bdd_bt_executor_ros2/bdd_bt_executor.hpp"

#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <bdd_ros2_interfaces/msg/trinary.hpp>

#include "bdd_bt_executor_ros2/nodes/mock_timed_action.hpp"
#include "bdd_bt_executor_ros2/nodes/publish_bdd_event.hpp"

namespace bdd_bt_executor_ros2 {

BddBtExecutor::BddBtExecutor(const rclcpp::NodeOptions &options)
  : rclcpp::Node("bdd_bt_action_server", options), tick_rate_hz_(30)
{
    tree_xml_              = declare_parameter<std::string>("tree_xml", "");
    tick_rate_hz_          = declare_parameter<int>("tick_rate_hz", 30);
    const auto server_name = declare_parameter<std::string>("bhv_server_name", "bhv_server");
    const auto event_topic = declare_parameter<std::string>("event_topic", "/bdd/events");

    if (event_topic.empty()) {
        throw std::invalid_argument("Parameter 'event_topic' must not be empty");
    }
    if (server_name.empty()) {
        throw std::invalid_argument("Parameter 'bhv_server_name' must not be empty");
    }

    event_publisher_ = create_publisher<bdd_ros2_interfaces::msg::Event>(event_topic, 10);
    action_server_   = rclcpp_action::create_server<BehaviourAction>(
      this,
      server_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const BehaviourAction::Goal>) {
          if (tree_running_.load(std::memory_order_acquire)) {
              RCLCPP_INFO(get_logger(), "A behaviour tree is already running; rejecting goal");
              return rclcpp_action::GoalResponse::REJECT;
          }
          if (tree_xml_.empty()) {
              RCLCPP_ERROR(get_logger(), "Parameter 'tree_xml' is empty; rejecting goal");
              return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
          RCLCPP_INFO(get_logger(), "Cancel requested for active behaviour goal");
          return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
          auto self = std::static_pointer_cast<BddBtExecutor>(shared_from_this());
          std::thread([self, goal_handle]() { self->execute_goal(goal_handle); }).detach();
      }
    );
}

void BddBtExecutor::set_node_registrar(RegisterNodes registrar)
{ register_nodes_ = std::move(registrar); }

void BddBtExecutor::execute_goal(const std::shared_ptr<GoalHandle> &goal_handle)
{
    if (tree_running_.exchange(true, std::memory_order_acq_rel)) {
        auto result    = std::make_shared<BehaviourAction::Result>();
        result->result = make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::UNKNOWN);
        goal_handle->abort(result);
        return;
    }
    const auto running_guard =
      std::unique_ptr<void, std::function<void(void *)>>(this, [this](void *) {
          tree_running_.store(false, std::memory_order_release);
      });

    auto result                   = std::make_shared<BehaviourAction::Result>();
    auto feedback                 = std::make_shared<BehaviourAction::Feedback>();
    feedback->scenario_context_id = goal_handle->get_goal()->scenario_context_id;

    try {
        BT::BehaviorTreeFactory factory;
        auto                    node = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
        factory.registerBuilder<MockTimedAction>(
          "MockTimedAction", [node](const std::string &name, const BT::NodeConfig &config) {
              return std::make_unique<MockTimedAction>(name, config, node);
          }
        );

        const auto context_id = goal_handle->get_goal()->scenario_context_id;
        factory.registerBuilder<PublishBddEvent>(
          "PublishBddEvent",
          [node, publisher = event_publisher_, context_id](
            const std::string &name, const BT::NodeConfig &config
          ) { return std::make_unique<PublishBddEvent>(name, config, node, publisher, context_id); }
        );
        if (register_nodes_) { register_nodes_(factory, node); }

        auto                tree = factory.createTreeFromFile(tree_xml_);
        BT::Groot2Publisher groot_publisher(tree);
        const auto          tick_period =
          std::chrono::milliseconds(std::max(1, 1000 / std::max(1, tick_rate_hz_)));

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                tree.haltTree();
                result->result =
                  make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::UNKNOWN);
                goal_handle->canceled(result);
                return;
            }

            const auto status = tree.tickOnce();
            feedback->status  = std::string("tree status: ") + BT::toStr(status, true);
            goal_handle->publish_feedback(feedback);

            if (status == BT::NodeStatus::SUCCESS) {
                result->result = make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::TRUE);
                goal_handle->succeed(result);
                return;
            }
            if (status == BT::NodeStatus::FAILURE) {
                tree.haltTree();
                result->result = make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::FALSE);
                goal_handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(tick_period);
        }

        tree.haltTree();
        result->result = make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::UNKNOWN);
        goal_handle->abort(result);
    } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "Failed to execute behaviour tree: %s", error.what());
        result->result = make_result(goal_handle, bdd_ros2_interfaces::msg::Trinary::FALSE);
        goal_handle->abort(result);
    }
}

bdd_ros2_interfaces::msg::TrinaryStamped
  BddBtExecutor::make_result(const std::shared_ptr<GoalHandle> &goal_handle, uint8_t value) const
{
    bdd_ros2_interfaces::msg::TrinaryStamped result;
    result.scenario_context_id = goal_handle->get_goal()->scenario_context_id;
    result.stamp               = get_clock()->now();
    result.trinary.value       = value;
    return result;
}

} // namespace bdd_bt_executor_ros2
