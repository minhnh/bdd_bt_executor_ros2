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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


namespace bdd_bt_executor_ros2 {

// Shared lifecycle for the two action types; callbacks never access the BT node.
template<class Action> class RosAction : public BT::StatefulActionNode
{
  protected:
    using Client = rclcpp_action::Client<Action>;
    using Handle = rclcpp_action::ClientGoalHandle<Action>;
    using Clock  = std::chrono::steady_clock;
    struct Invocation
    {
        std::mutex                                mutex;
        rclcpp::Node::SharedPtr                   owner;
        typename Client::SharedPtr                client;
        typename Handle::SharedPtr                handle;
        typename Action::Feedback::ConstSharedPtr feedback;
        typename Handle::WrappedResult            result;
        rclcpp::TimerBase::SharedPtr              cleanup;
        bool                                      sent             = false;
        bool                                      retired          = false;
        bool                                      complete         = false;
        bool                                      rejected         = false;
        bool                                      cancel_requested = false;
        bool                                      cancel_sent      = false;
        std::string                               error;
    };

    RosAction(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node)
      : BT::StatefulActionNode(name, config), node_(std::move(node))
    {}

    virtual bool readGoal(typename Action::Goal &goal, std::string &error) = 0;
    virtual void writeFeedback(const typename Action::Feedback &feedback)  = 0;
    virtual bool writeResult(const typename Action::Result &result)        = 0;

    void statusMessage(const std::string &message)
    {
        setOutput("status_message", message);
        if (config().blackboard) {
            config().blackboard->set("behaviour_status", name() + ": " + message);
        }
    }

    static void release(const std::shared_ptr<Invocation> &state)
    {
        rclcpp::Node::SharedPtr      owner;
        typename Client::SharedPtr   client;
        typename Handle::SharedPtr   handle;
        rclcpp::TimerBase::SharedPtr cleanup;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->retired = true;
            client         = std::move(state->client);
            handle         = std::move(state->handle);
            cleanup        = std::move(state->cleanup);
            owner          = std::move(state->owner);
        }
        // Destroy ROS resources without holding the callback-state mutex.
        if (cleanup) { cleanup->cancel(); }
        if (client && handle) { client->stop_callbacks(handle); }
    }

    static void sendCancel(const std::shared_ptr<Invocation> &state, rclcpp::Logger logger)
    {
        typename Client::SharedPtr client;
        typename Handle::SharedPtr handle;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (
              !state->client || !state->cancel_requested || state->cancel_sent || !state->handle
              || state->complete
            ) {
                return;
            }
            state->cancel_sent = true;
            client             = state->client;
            handle             = state->handle;
        }
        try {
            client->async_cancel_goal(handle, [state, logger](auto response) {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (response->return_code != 0 || response->goals_canceling.empty()) {
                    state->error = "cancellation rejected; stop unconfirmed";
                    RCLCPP_ERROR(logger, "%s", state->error.c_str());
                } else {
                    RCLCPP_INFO(logger, "Cancellation accepted; physical stop unconfirmed");
                }
            });
        } catch (const std::exception &error) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->error = std::string("cancellation failed; stop unconfirmed: ") + error.what();
            RCLCPP_ERROR(logger, "%s", state->error.c_str());
        }
    }

    void cancelInvocation()
    {
        if (!state_) { return; }
        auto       state  = state_;
        const auto logger = node_->get_logger();
        bool       finished;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            finished = state->complete || !state->sent;
            if (!finished && state->cancel_requested) { return; }
            state->cancel_requested = true;
            if (!finished) {
                // Retain late acceptance/cancellation callbacks for a bounded interval.
                const auto end = Clock::now() + cancel_timeout_;
                state->cleanup = node_->create_wall_timer(
                  std::min(cancel_timeout_, Clock::duration(std::chrono::milliseconds(10))),
                  [state, logger, end]() {
                      {
                          std::lock_guard<std::mutex> guard(state->mutex);
                          if (!state->complete && Clock::now() < end) { return; }
                          if (!state->complete) {
                              RCLCPP_WARN(
                                logger, "Cancellation deadline expired; stop unconfirmed"
                              );
                          }
                      }
                      release(state);
                  }
                );
            }
        }
        if (finished) {
            release(state);
            return;
        }
        RCLCPP_WARN(logger, "Cancellation requested; stop unconfirmed");
        sendCancel(state, logger);
    }

  public:
    ~RosAction() override
    {
        try {
            cancelInvocation();
        } catch (const std::exception &error) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "Unable to retain cancellation callbacks: %s; stop unconfirmed",
              error.what()
            );
            if (state_) { release(state_); }
        }
    }

    BT::NodeStatus onStart() override
    {
        cancelInvocation();
        state_                = std::make_shared<Invocation>();
        state_->owner         = node_;
        double timeout        = 0.0;
        double cancel_timeout = 0.0;
        try {
            const auto now = Clock::now();
            const auto max_seconds =
              std::chrono::duration<double>(Clock::time_point::max() - now).count();
            if (
              !getInput("timeout", timeout) || !std::isfinite(timeout) || timeout <= 0.0
              || timeout >= max_seconds || !getInput("cancel_timeout", cancel_timeout)
              || !std::isfinite(cancel_timeout) || cancel_timeout <= 0.0
              || cancel_timeout >= max_seconds
            ) {
                throw std::invalid_argument(
                  "timeouts must be finite, positive steady-clock durations"
                );
            }
            deadline_ =
              now
              + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
            cancel_timeout_ = std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(cancel_timeout)
            );
            std::string endpoint;
            if (!getInput("action_name", endpoint) || endpoint.empty()) {
                throw std::invalid_argument("action_name must not be empty");
            }
            std::string error;
            if (!readGoal(goal_, error)) { throw std::invalid_argument(error); }
            if (!client_ || endpoint_ != endpoint) {
                auto client = rclcpp_action::create_client<Action>(node_, endpoint);
                client_     = std::move(client);
                endpoint_   = std::move(endpoint);
            }
            state_->client = client_;
            return onRunning();
        } catch (const std::exception &error) {
            statusMessage(std::string("invalid input or action request: ") + error.what());
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onRunning() override
    {
        auto                                      state = state_;
        typename Client::SharedPtr                client;
        typename Action::Feedback::ConstSharedPtr feedback;
        typename Handle::WrappedResult            result;
        bool                                      complete;
        bool                                      rejected;
        std::string                               error;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            complete = state->complete;
            rejected = state->rejected;
            error    = state->error;
            feedback = state->feedback;
            result   = state->result;
            client   = state->client;
        }
        if (feedback) { writeFeedback(*feedback); }
        if (complete) {
            release(state);
            const bool action_success = result.result && writeResult(*result.result);
            const bool success =
              !rejected && result.code == rclcpp_action::ResultCode::SUCCEEDED && action_success;
            statusMessage(success ? "succeeded" : (rejected ? "goal rejected" : "action failed"));
            return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if (!error.empty() || Clock::now() >= deadline_) {
            cancelInvocation();
            statusMessage(
              error.empty() ? (state->sent ? "timeout; cancellation requested; stop unconfirmed"
                                           : "action server unavailable before timeout")
                            : error
            );
            return BT::NodeStatus::FAILURE;
        }
        if (!state->sent) {
            if (!client->action_server_is_ready()) {
                statusMessage("waiting for action server");
                return BT::NodeStatus::RUNNING;
            }
            typename Client::SendGoalOptions options;
            const auto                       logger     = node_->get_logger();
            const auto                       weak_state = std::weak_ptr<Invocation>(state);
            options.goal_response_callback =
              [weak_state,
               logger,
               weak_client = std::weak_ptr<Client>(client)](typename Handle::SharedPtr handle) {
                  auto state   = weak_state.lock();
                  bool retired = !state;
                  if (state) {
                      std::lock_guard<std::mutex> lock(state->mutex);
                      retired = state->retired;
                      if (!retired) {
                          state->handle = handle;
                          if (!handle) { state->rejected = state->complete = true; }
                      }
                  }
                  if (retired) {
                      if (auto old_client = weak_client.lock(); old_client && handle) {
                          old_client->stop_callbacks(handle);
                      }
                      return;
                  }
                  sendCancel(state, logger);
              };
            options.feedback_callback =
              [weak_state](auto, typename Action::Feedback::ConstSharedPtr feedback_msg) {
                  if (auto state = weak_state.lock()) {
                      std::lock_guard<std::mutex> lock(state->mutex);
                      if (!state->retired) { state->feedback = std::move(feedback_msg); }
                  }
              };
            options.result_callback = [weak_state,
                                       logger](const typename Handle::WrappedResult &wrapped) {
                if (auto state = weak_state.lock()) {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->retired) { return; }
                    state->result   = wrapped;
                    state->complete = true;
                    if (state->cancel_requested) {
                        RCLCPP_INFO(
                          logger,
                          "Action finished after cancellation request (result code %d)",
                          static_cast<int>(wrapped.code)
                        );
                    }
                    state->handle.reset();
                }
            };
            try {
                state->sent = true;
                client->async_send_goal(goal_, options);
            } catch (const std::exception &exception) {
                cancelInvocation();
                statusMessage(std::string("goal submission failed: ") + exception.what());
                return BT::NodeStatus::FAILURE;
            }
        }
        statusMessage(feedback ? feedbackMessage(*feedback) : "waiting for acceptance/result");
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        cancelInvocation();
        statusMessage("halted; cancellation requested; stop unconfirmed");
    }

  protected:
    virtual std::string     feedbackMessage(const typename Action::Feedback &) = 0;
    rclcpp::Node::SharedPtr node_;

  private:
    typename Client::SharedPtr  client_;
    std::string                 endpoint_;
    typename Action::Goal       goal_;
    std::shared_ptr<Invocation> state_;
    Clock::time_point           deadline_;
    Clock::duration             cancel_timeout_ = std::chrono::seconds(2);
};

} // namespace bdd_bt_executor_ros2
