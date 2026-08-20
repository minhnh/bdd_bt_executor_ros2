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

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <bdd_ros2_interfaces/msg/event.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bdd_bt_executor_ros2/nodes/mock_timed_action.hpp"
#include "bdd_bt_executor_ros2/nodes/publish_bdd_event.hpp"

using namespace std::chrono_literals;

class BtNodesTest : public testing::Test
{
  protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(BtNodesTest, MockTimedActionCompletesAndRejectsInvalidHeartbeat)
{
    auto           node = std::make_shared<rclcpp::Node>("test_mock_timed_action");
    BT::NodeConfig config;
    config.input_ports["duration"]           = "0.02";
    config.input_ports["heartbeat_duration"] = "0.001";
    config.input_ports["message"]            = "working";
    bdd_bt_executor_ros2::MockTimedAction action("mock", config, node);

    EXPECT_EQ(action.executeTick(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(25ms);
    EXPECT_EQ(action.executeTick(), BT::NodeStatus::SUCCESS);

    config.input_ports["heartbeat_duration"] = "0";
    bdd_bt_executor_ros2::MockTimedAction invalid("invalid", config, node);
    EXPECT_EQ(invalid.executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, PublishBddEventPublishesContextAndIri)
{
    auto node      = std::make_shared<rclcpp::Node>("test_publish_bdd_event");
    auto publisher = node->create_publisher<bdd_ros2_interfaces::msg::Event>("events", 10);
    std::vector<bdd_ros2_interfaces::msg::Event> events;
    auto subscription = node->create_subscription<bdd_ros2_interfaces::msg::Event>(
      "events", 10, [&events](bdd_ros2_interfaces::msg::Event::ConstSharedPtr event) {
          events.push_back(*event);
      }
    );
    (void)subscription;
    unique_identifier_msgs::msg::UUID context_id;
    context_id.uuid[0] = 42;

    BT::NodeConfig config;
    config.input_ports["iri"] = "https://example.test/events/start";
    bdd_bt_executor_ros2::PublishBddEvent action("publish", config, node, publisher, context_id);
    EXPECT_EQ(action.executeTick(), BT::NodeStatus::SUCCESS);

    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (events.empty() && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
    }
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].uri, "https://example.test/events/start");
    EXPECT_EQ(events[0].scenario_context_id, context_id);
}
