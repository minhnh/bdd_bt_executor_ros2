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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "bdd_bt_executor_ros2/nodes/get_entity_pose.hpp"

using namespace std::chrono_literals;
using bdd_bt_executor_ros2::GetEntityPose;
using Service = GetEntityPose::Service;

class EntityPoseTest : public testing::Test
{
  protected:
    static void                                    SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void                                    TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr                        node;
    rclcpp::executors::SingleThreadedExecutor      executor;
    BT::BehaviorTreeFactory                        factory;
    rclcpp::Service<Service>::SharedPtr            service;
    std::vector<std::shared_ptr<rmw_request_id_t>> headers;
    std::vector<std::string>                       entities;

    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>(
          "entity_pose_test",
          rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter("use_sim_time", true) })
        );
        executor.add_node(node);
        factory.registerBuilder<GetEntityPose>(
          "GetEntityPose", [this](const auto &name, const auto &config) {
              return std::make_unique<GetEntityPose>(name, config, node);
          }
        );
    }
    void serve()
    {
        service = node->create_service<Service>(
          "/pose_test/get_entity_state",
          [this](std::shared_ptr<rmw_request_id_t> header, Service::Request::SharedPtr request) {
              headers.push_back(header);
              entities.push_back(request->entity);
          }
        );
    }
    BT::Tree tree(
      const std::string &attributes =
        "entity_id='{entity}' service_namespace='/pose_test/' timeout='1'"
    )
    {
        auto bb = BT::Blackboard::create();
        bb->set("entity", std::string("/spawned/cube"));
        return factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Test'>"
          "<GetEntityPose " + attributes + " pose='{pose}' status_message='{message}'/>"
          "</BehaviorTree></root>", bb);
    }
    bool wait(const std::function<bool()> &predicate)
    {
        const auto end = std::chrono::steady_clock::now() + 2s;
        do {
            executor.spin_some();
            if (predicate()) { return true; }
            std::this_thread::sleep_for(1ms);
        } while (std::chrono::steady_clock::now() < end);
        return false;
    }
    void respond(size_t index, uint8_t code = simulation_interfaces::msg::Result::RESULT_OK)
    {
        Service::Response response;
        response.result.result              = code;
        response.result.error_message       = "test service message";
        response.state.header.frame_id      = "simulation_world";
        response.state.header.stamp.sec     = 42;
        response.state.header.stamp.nanosec = 123;
        response.state.pose.position.x      = static_cast<double>(index + 1);
        response.state.pose.orientation.w   = 1.0;
        service->send_response(*headers.at(index), response);
    }
    BT::NodeStatus finish(BT::Tree &t)
    {
        auto status = BT::NodeStatus::RUNNING;
        EXPECT_TRUE(wait([&]() {
            status = t.tickOnce();
            return status != BT::NodeStatus::RUNNING;
        }));
        return status;
    }
};

TEST_F(EntityPoseTest, PreservesPoseFrameAndStampAndSubmitsOnce)
{
    serve();
    auto t = tree();
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return headers.size() == 1;
    }));
    EXPECT_EQ(entities.front(), "/spawned/cube");
    for (int i = 0; i < 5; ++i) { EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING); }
    executor.spin_some();
    EXPECT_EQ(headers.size(), 1u);
    respond(0);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    auto pose = t.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("pose");
    EXPECT_EQ(pose.header.frame_id, "simulation_world");
    EXPECT_EQ(pose.header.stamp.sec, 42);
    EXPECT_EQ(pose.header.stamp.nanosec, 123u);
    EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
    EXPECT_EQ(t.rootBlackboard()->get<std::string>("behaviour_status"), "GetEntityPose: succeeded");
}

TEST_F(EntityPoseTest, ServiceErrorsAndUnavailableServerFail)
{
    auto unavailable = tree("entity_id='cube' service_namespace='/absent' timeout='0.02'");
    EXPECT_EQ(unavailable.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(finish(unavailable), BT::NodeStatus::FAILURE);
    EXPECT_EQ(
      unavailable.rootBlackboard()->get<std::string>("message"),
      "service unavailable before timeout"
    );
    serve();
    for (uint8_t code : { simulation_interfaces::msg::Result::RESULT_NOT_FOUND,
                          simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED }) {
        auto       t     = tree();
        const auto index = headers.size();
        ASSERT_TRUE(wait([&]() {
            t.tickOnce();
            return headers.size() > index;
        }));
        respond(index, code);
        EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
        EXPECT_NE(
          t.rootBlackboard()->get<std::string>("message").find("test service message"),
          std::string::npos
        );
    }
}

TEST_F(EntityPoseTest, HaltTimeoutAndDestroyedTreeDiscardLateResponses)
{
    serve();
    auto t = tree();
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return headers.size() == 1;
    }));
    t.haltTree();
    t.rootBlackboard()->set("entity", std::string("/spawned/bin"));
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return headers.size() == 2;
    }));
    EXPECT_EQ(entities.back(), "/spawned/bin");
    respond(0);
    executor.spin_some();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    respond(1);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(
      t.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("pose").pose.position.x, 2.0
    );
    {
        auto timeout = tree("entity_id='cube' service_namespace='/pose_test' timeout='0.1'");
        ASSERT_TRUE(wait([&]() {
            timeout.tickOnce();
            return headers.size() == 3;
        }));
        EXPECT_EQ(finish(timeout), BT::NodeStatus::FAILURE);
        EXPECT_EQ(timeout.rootBlackboard()->get<std::string>("message"), "entity query timed out");
    }
    respond(2);
    executor.spin_some();
    {
        auto destroyed = tree();
        ASSERT_TRUE(wait([&]() {
            destroyed.tickOnce();
            return headers.size() == 4;
        }));
    }
    respond(3);
    executor.spin_some();
    auto fresh = tree();
    ASSERT_TRUE(wait([&]() {
        fresh.tickOnce();
        return headers.size() == 5;
    }));
    respond(4);
    EXPECT_EQ(finish(fresh), BT::NodeStatus::SUCCESS);
}

TEST_F(EntityPoseTest, InvalidInputsFailBeforeRequest)
{
    serve();
    for (const auto &attributes : { "entity_id=''",
                                    "",
                                    "entity_id='cube' timeout='0'",
                                    "entity_id='cube' timeout='-1'",
                                    "entity_id='cube' timeout='nan'",
                                    "entity_id='cube' service_namespace='invalid namespace'" }) {
        auto t = tree(attributes);
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    }
    EXPECT_TRUE(headers.empty());
}
