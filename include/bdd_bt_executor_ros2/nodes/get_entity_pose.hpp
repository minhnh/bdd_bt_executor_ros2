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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>

namespace bdd_bt_executor_ros2 {

class GetEntityPose : public BT::StatefulActionNode
{
  public:
    using Service = simulation_interfaces::srv::GetEntityState;
    using Clock   = std::chrono::steady_clock;

    GetEntityPose(
      const std::string      &name,
      const BT::NodeConfig   &config,
      rclcpp::Node::SharedPtr node
    )
      : BT::StatefulActionNode(name, config), node_(std::move(node))
    {}

    ~GetEntityPose() override { clearRequest(); }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("entity_id"),
            BT::InputPort<std::string>("service_namespace", "/", "Simulation service namespace"),
            BT::InputPort<double>("timeout", 5.0, "Steady-clock timeout in seconds"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
            BT::OutputPort<std::string>("status_message")
        };
    }

    BT::NodeStatus onStart() override
    {
        clearRequest();
        try {
            double      timeout;
            std::string ns;
            const auto  now = Clock::now();
            if (
              !getInput("entity_id", entity_) || entity_.empty()
              || !getInput("service_namespace", ns) || !getInput("timeout", timeout)
              || !std::isfinite(timeout) || timeout <= 0.0
              || timeout >= std::chrono::duration<double>(Clock::time_point::max() - now).count()
            ) {
                throw std::invalid_argument(
                  "Require nonempty entity_id and finite positive timeout"
                );
            }
            deadline_ =
              now
              + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
            const bool absolute = !ns.empty() && ns.front() == '/';
            while (!ns.empty() && ns.back() == '/') { ns.pop_back(); }
            const auto endpoint = ns.empty() ? (absolute ? "/get_entity_state" : "get_entity_state")
                                             : ns + "/get_entity_state";
            if (!client_ || endpoint_ != endpoint) {
                auto client = node_->create_client<Service>(endpoint);
                client_     = std::move(client);
                endpoint_   = endpoint;
            }
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
                const auto response = future_.get();
                clearRequest();
                if (response->result.result != simulation_interfaces::msg::Result::RESULT_OK) {
                    return fail(
                      "Entity query failed (" + std::to_string(response->result.result)
                      + "): " + response->result.error_message
                    );
                }
                geometry_msgs::msg::PoseStamped pose;
                pose.header = response->state.header;
                pose.pose   = response->state.pose;
                setOutput("pose", pose);
                statusMessage("succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            if (Clock::now() >= deadline_) {
                return fail(
                  request_id_ ? "entity query timed out" : "service unavailable before timeout"
                );
            }
            if (!request_id_) {
                if (!client_->service_is_ready()) {
                    statusMessage("waiting for entity-state service");
                    return BT::NodeStatus::RUNNING;
                }
                auto request    = std::make_shared<Service::Request>();
                request->entity = entity_;
                auto pending    = client_->async_send_request(request);
                request_id_     = pending.request_id;
                future_         = std::move(pending.future);
            }
            statusMessage("waiting for entity pose");
            return BT::NodeStatus::RUNNING;
        } catch (const std::exception &error) {
            return fail(error.what());
        }
    }

    void onHalted() override
    {
        clearRequest();
        statusMessage("halted; pending pose response discarded");
    }

  private:
    void clearRequest()
    {
        // Removing a pending request releases its promise; late replies are ignored by rclcpp.
        if (client_ && request_id_) { client_->remove_pending_request(*request_id_); }
        request_id_.reset();
        future_ = {};
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
    std::future<Service::Response::SharedPtr> future_;
    std::optional<int64_t>                    request_id_;
    std::string                               entity_, endpoint_;
    Clock::time_point                         deadline_;
};

} // namespace bdd_bt_executor_ros2
