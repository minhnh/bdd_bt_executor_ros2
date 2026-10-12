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
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bdd_bt_executor_ros2 {

class ApplyPlanningScene : public BT::StatefulActionNode
{
  public:
    using Service = moveit_msgs::srv::ApplyPlanningScene;
    using Clock   = std::chrono::steady_clock;
    ApplyPlanningScene(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : BT::StatefulActionNode(name, config), node_(std::move(node))
    {}
    ~ApplyPlanningScene() override { clearRequest(); }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<moveit_msgs::msg::PlanningScene>("scene"),
                 BT::InputPort<std::string>(
                   "service_name", "/apply_planning_scene", "MoveIt service endpoint"
                 ),
                 BT::InputPort<double>("timeout", 5.0, "Steady-clock timeout in seconds"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus onStart() override
    {
        clearRequest();
        try {
            double      timeout;
            std::string endpoint;
            const auto  now     = Clock::now();
            auto        request = std::make_shared<Service::Request>();
            if (
              !getInput("scene", request->scene) || !getInput("service_name", endpoint)
              || endpoint.empty() || !getInput("timeout", timeout) || !std::isfinite(timeout)
              || timeout <= 0
              || timeout >= std::chrono::duration<double>(Clock::time_point::max() - now).count()
            ) {
                throw std::invalid_argument(
                  "Require scene, nonempty service_name and finite positive timeout"
                );
            }
            deadline_ =
              now
              + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
            if (!client_ || endpoint_ != endpoint) {
                auto client = node_->create_client<Service>(endpoint);
                client_     = std::move(client);
                endpoint_   = endpoint;
            }
            message_ = std::move(request);
            return onRunning();
        } catch (const std::exception &error) {
            return fail(error.what());
        }
    }
    BT::NodeStatus onRunning() override
    {
        try {
            if (
              future_.valid()
              && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready
            ) {
                const bool success = future_.get()->success;
                clearRequest();
                if (!success) { return fail("scene update rejected"); }
                statusMessage("succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            if (Clock::now() >= deadline_) {
                return fail(
                  request_id_ ? "planning scene request timed out"
                              : "service unavailable before timeout"
                );
            }
            if (!request_id_ && client_->service_is_ready()) {
                auto pending = client_->async_send_request(message_);
                request_id_  = pending.request_id;
                future_      = std::move(pending.future);
            }
            statusMessage(
              request_id_ ? "waiting for scene acknowledgement"
                          : "waiting for planning scene service"
            );
            return BT::NodeStatus::RUNNING;
        } catch (const std::exception &error) {
            return fail(error.what());
        }
    }
    void onHalted() override
    {
        clearRequest();
        statusMessage("halted; pending scene response discarded; application not canceled");
    }

  private:
    void clearRequest()
    {
        // MoveIt may still apply a request after its local future is discarded.
        if (client_ && request_id_) { client_->remove_pending_request(*request_id_); }
        request_id_.reset();
        future_ = {};
        message_.reset();
    }
    void statusMessage(const std::string &message)
    {
        setOutput("status_message", message);
        config().blackboard->set("behaviour_status", name() + ": " + message);
    }
    BT::NodeStatus fail(const std::string &message)
    {
        clearRequest();
        statusMessage(message);
        return BT::NodeStatus::FAILURE;
    }
    rclcpp::Node::SharedPtr                   node_;
    rclcpp::Client<Service>::SharedPtr        client_;
    std::shared_ptr<Service::Request>         message_;
    std::future<Service::Response::SharedPtr> future_;
    std::optional<int64_t>                    request_id_;
    std::string                               endpoint_;
    Clock::time_point                         deadline_;
};
} // namespace bdd_bt_executor_ros2
