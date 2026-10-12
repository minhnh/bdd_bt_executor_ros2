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
#include "bdd_bt_executor_ros2/nodes/pose_operations.hpp"
#include "bdd_bt_executor_ros2/nodes/resolve_behaviour_parameters.hpp"

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

TEST_F(BtNodesTest, SelectAndOffsetPosePreservesFrameAndSupportsLocalAndReferenceOffsets)
{
    using namespace bdd_bt_executor_ros2;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SelectEntityPose>("SelectEntityPose");
    factory.registerNodeType<OffsetPose>("OffsetPose");
    auto                            bb = BT::Blackboard::create();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "world";
    pose.header.stamp.sec   = 42;
    pose.pose.position.x    = 2;
    pose.pose.orientation.z = pose.pose.orientation.w = std::sqrt(0.5);
    bb->set("poses", EntityPoses{ { "urn:bottle", pose } });
    bb->set("uri", std::string("urn:bottle"));
    auto make = [&](const std::string &offset) {
        return factory.createTreeFromText(
          "<root BTCPP_format='4'><BehaviorTree ID='Test'><Sequence>"
          "<SelectEntityPose poses='{poses}' object_uri='{uri}' pose='{selected}'/>"
          "<OffsetPose pose='{selected}' target_pose='{target}' " + offset + "/>"
          "</Sequence></BehaviorTree></root>", bb);
    };
    auto local = make(
      "position_offset='1;0;0' orientation_offset='0;0;0.7071067811865476;0.7071067811865476'"
    );
    ASSERT_EQ(local.tickOnce(), BT::NodeStatus::SUCCESS);
    auto target = bb->get<geometry_msgs::msg::PoseStamped>("target");
    EXPECT_EQ(target.header, pose.header);
    EXPECT_NEAR(target.pose.position.x, 2, 1e-9);
    EXPECT_NEAR(target.pose.position.y, 1, 1e-9);
    EXPECT_NEAR(std::abs(target.pose.orientation.z), 1, 1e-9);
    auto reference = make("position_offset='1;0;0' offset_frame='reference'");
    ASSERT_EQ(reference.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(bb->get<geometry_msgs::msg::PoseStamped>("target").pose.position.x, 3);
    auto absolute = make("orientation='1;0;0;0' position_offset='0;0;-0.1'");
    ASSERT_EQ(absolute.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(bb->get<geometry_msgs::msg::PoseStamped>("target").pose.position.z, 0.1, 1e-9);
    for (const auto &input : { "position_offset='1;2'",
                               "position_offset='nan;0;0'",
                               "orientation_offset='0;0;0;0'",
                               "orientation='0;0;0;2'",
                               "offset_frame='unknown'" }) {
        auto invalid = make(input);
        EXPECT_EQ(invalid.tickOnce(), BT::NodeStatus::FAILURE);
    }
    bb->set("uri", std::string("urn:missing"));
    auto missing = make("");
    EXPECT_EQ(missing.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, BehaviourUriBindingsValidateVariablesWithoutFixingEntityValues)
{
    using namespace bdd_bt_executor_ros2;
    bdd_ros2_interfaces::action::Behaviour::Goal goal;
    bdd_ros2_interfaces::msg::ParamValue         parameter;
    parameter.variable_uri   = "urn:target-variable";
    parameter.param_val_uris = { "urn:cube" };
    goal.parameters.push_back(parameter);
    const auto bindings = YAML::Load("object_uri: urn:target-variable");
    EXPECT_EQ(resolveBehaviourParameters(goal, bindings).at("object_uri"), "urn:cube");
    goal.parameters[0].param_val_uris = { "urn:bottle" };
    EXPECT_EQ(resolveBehaviourParameters(goal, bindings).at("object_uri"), "urn:bottle");
    for (const auto &invalid : { "{}",
                                 "object_uri: urn:missing",
                                 "behaviour_goal: urn:target-variable",
                                 "a: urn:target-variable\nb: urn:target-variable",
                                 "a: urn:target-variable\na: urn:other" })
        EXPECT_THROW(resolveBehaviourParameters(goal, YAML::Load(invalid)), std::invalid_argument);
    goal.parameters.push_back(parameter);
    EXPECT_THROW(resolveBehaviourParameters(goal, bindings), std::invalid_argument);
    goal.parameters.pop_back();
    goal.parameters[0].param_val_uris.clear();
    EXPECT_THROW(resolveBehaviourParameters(goal, bindings), std::invalid_argument);
}
