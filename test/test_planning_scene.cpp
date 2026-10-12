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

#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include "bdd_bt_executor_ros2/nodes/apply_planning_scene.hpp"

using namespace std::chrono_literals;
using bdd_bt_executor_ros2::ApplyPlanningScene;
using Service = ApplyPlanningScene::Service;

class PlanningSceneTest : public testing::Test
{
  protected:
    static void                                    SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void                                    TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr                        node;
    rclcpp::executors::SingleThreadedExecutor      executor;
    BT::BehaviorTreeFactory                        factory;
    rclcpp::Service<Service>::SharedPtr            service;
    std::vector<std::shared_ptr<rmw_request_id_t>> headers;
    std::vector<moveit_msgs::msg::PlanningScene>   requests;
    void                                           SetUp() override
    {
        node = std::make_shared<rclcpp::Node>(
          "planning_scene_test",
          rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter("use_sim_time", true) })
        );
        executor.add_node(node);
        factory.registerNodeType<ApplyPlanningScene>("ApplyPlanningScene", node);
    }
    void serve(const std::string &endpoint = "/scene_test/apply")
    {
        service = node->create_service<Service>(
          endpoint,
          [this](std::shared_ptr<rmw_request_id_t> header, Service::Request::SharedPtr request) {
              headers.push_back(header);
              requests.push_back(request->scene);
          }
        );
    }
    BT::Tree
      tree(const std::string &attributes = "scene='{scene}' service_name='{endpoint}' timeout='1'")
    {
        auto                            bb = BT::Blackboard::create();
        moveit_msgs::msg::PlanningScene scene;
        scene.name                = "native scene";
        scene.is_diff             = false;
        scene.robot_state.is_diff = true;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name   = "tool";
        attached.object.id   = "payload";
        attached.touch_links = { "finger" };
        scene.robot_state.attached_collision_objects.push_back(attached);
        bb->set("scene", scene);
        bb->set("endpoint", std::string("/scene_test/apply"));
        return factory.createTreeFromText(
          "<root BTCPP_format='4'><BehaviorTree ID='Test'><ApplyPlanningScene " + attributes
            + " status_message='{message}'/></BehaviorTree></root>",
          bb
        );
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
    void respond(size_t index, bool success = true)
    {
        Service::Response response;
        response.success = success;
        service->send_response(*headers.at(index), response);
    }
    BT::NodeStatus finish(BT::Tree &t)
    {
        auto status = BT::NodeStatus::RUNNING;
        EXPECT_TRUE(wait([&] {
            status = t.tickOnce();
            return status != BT::NodeStatus::RUNNING;
        }));
        return status;
    }
};
TEST_F(PlanningSceneTest, ForwardsNativeSceneOnceAndRequiresAcknowledgement)
{
    serve();
    auto t = tree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return requests.size() == 1;
    }));
    EXPECT_EQ(requests.front(), t.rootBlackboard()->get<moveit_msgs::msg::PlanningScene>("scene"));
    for (int i = 0; i < 5; ++i) { EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING); }
    executor.spin_some();
    EXPECT_EQ(requests.size(), 1u);
    respond(0);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(t.rootBlackboard()->get<std::string>("message"), "succeeded");
    EXPECT_EQ(
      t.rootBlackboard()->get<std::string>("behaviour_status"), "ApplyPlanningScene: succeeded"
    );
    t.haltTree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return requests.size() == 2;
    }));
    respond(1, false);
    EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
    EXPECT_EQ(t.rootBlackboard()->get<std::string>("message"), "scene update rejected");
    t.haltTree();
    t.rootBlackboard()->set("endpoint", std::string("/scene_test/other"));
    serve("/scene_test/other");
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return requests.size() == 3;
    }));
    respond(2);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
}
TEST_F(PlanningSceneTest, InvalidInputsAndUnavailableServiceFail)
{
    serve();
    for (const auto &attrs : { "service_name='/scene_test/apply'",
                               "scene='{scene}' service_name=''",
                               "scene='{scene}' timeout='0'",
                               "scene='{scene}' timeout='-1'",
                               "scene='{scene}' timeout='nan'",
                               "scene='{scene}' timeout='inf'",
                               "scene='{scene}' service_name='invalid name'" }) {
        auto t = tree(attrs);
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    }
    EXPECT_TRUE(requests.empty());
    auto absent = tree("scene='{scene}' service_name='/scene_test/absent' timeout='0.02'");
    EXPECT_EQ(absent.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(finish(absent), BT::NodeStatus::FAILURE);
    EXPECT_EQ(
      absent.rootBlackboard()->get<std::string>("message"), "service unavailable before timeout"
    );
}
TEST_F(PlanningSceneTest, HaltTimeoutAndDestructionDiscardLateReplies)
{
    serve();
    auto t = tree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return requests.size() == 1;
    }));
    t.haltTree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return requests.size() == 2;
    }));
    respond(0);
    executor.spin_some();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    respond(1);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    {
        auto timed = tree("scene='{scene}' service_name='/scene_test/apply' timeout='0.1'");
        ASSERT_TRUE(wait([&] {
            timed.tickOnce();
            return requests.size() == 3;
        }));
        EXPECT_EQ(finish(timed), BT::NodeStatus::FAILURE);
        EXPECT_EQ(
          timed.rootBlackboard()->get<std::string>("message"), "planning scene request timed out"
        );
    }
    respond(2);
    executor.spin_some();
    {
        auto destroyed = tree();
        ASSERT_TRUE(wait([&] {
            destroyed.tickOnce();
            return requests.size() == 4;
        }));
    }
    respond(3);
    executor.spin_some();
    auto fresh = tree();
    ASSERT_TRUE(wait([&] {
        fresh.tickOnce();
        return requests.size() == 5;
    }));
    respond(4);
    EXPECT_EQ(finish(fresh), BT::NodeStatus::SUCCESS);
}
