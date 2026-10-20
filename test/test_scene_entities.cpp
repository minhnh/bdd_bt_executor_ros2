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

#include <unistd.h>
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bdd_bt_executor_ros2/nodes/build_planning_scene.hpp"
#include "bdd_bt_executor_ros2/nodes/planning_scene_objects.hpp"

using namespace std::chrono_literals;
using bdd_bt_executor_ros2::GetEntityPoses;
using bdd_bt_executor_ros2::BuildPlanningScene;
using bdd_bt_executor_ros2::EntityPoses;
using bdd_bt_executor_ros2::GetEntityPose;
using bdd_bt_executor_ros2::readSceneEntities;
using Service = GetEntityPose::Service;
static const char *catalogue =
  R"(
entities:
  - uri: urn:cube
    id: /cube
    collision_id: test/cube
    shapes:
      - type: box
        dimensions: [0.05,0.05,0.05]
        position: [1,0,0]
  - uri: urn:can
    id: /can
    shapes:
      - type: cylinder
        dimensions: [0.12,0.03]
  - uri: urn:bin
    id: /bin
    shapes:
      - type: box
        dimensions: [0.3,0.2,0.01]
      - type: sphere
        dimensions: [0.02]
        position: [0,0,0.1]
)";
class SceneEntitiesTest : public testing::Test
{
  protected:
    static void                                    SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void                                    TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr                        node;
    rclcpp::executors::SingleThreadedExecutor      executor;
    std::shared_ptr<tf2_ros::Buffer>               tf;
    BT::BehaviorTreeFactory                        factory;
    rclcpp::Service<Service>::SharedPtr            service;
    std::vector<std::shared_ptr<rmw_request_id_t>> headers;
    std::vector<std::string>                       entities;
    std::string                                    file;
    void                                           SetUp() override
    {
        file   = "/tmp/bdd_scene_entities_XXXXXX";
        int fd = mkstemp(file.data());
        ASSERT_GE(fd, 0);
        close(fd);
        std::ofstream(file) << catalogue;
        node = std::make_shared<rclcpp::Node>(
          "scene_entities_test",
          rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter("use_sim_time", true) })
        );
        executor.add_node(node);
        tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id         = "world";
        transform.child_frame_id          = "sim";
        transform.transform.translation.x = 1;
        transform.transform.translation.y = 2;
        transform.transform.translation.z = 3;
        transform.transform.rotation.z    = std::sqrt(0.5);
        transform.transform.rotation.w    = std::sqrt(0.5);
        ASSERT_TRUE(tf->setTransform(transform, "test", true));
        factory.registerNodeType<GetEntityPoses>("GetEntityPoses", node, tf);
        factory.registerNodeType<BuildPlanningScene>("BuildPlanningScene");
        factory.registerNodeType<bdd_bt_executor_ros2::ClearPlanningSceneForUris>(
          "ClearPlanningSceneForUris", node
        );
        factory.registerNodeType<bdd_bt_executor_ros2::AttachObjectByUri>("AttachObjectByUri");
        factory.registerNodeType<bdd_bt_executor_ros2::DetachObjectByUri>("DetachObjectByUri");
        service = node->create_service<Service>(
          "/scene_entities_test/get_entity_state",
          [this](std::shared_ptr<rmw_request_id_t> header, Service::Request::SharedPtr request) {
              headers.push_back(header);
              entities.push_back(request->entity);
          }
        );
    }
    void     TearDown() override { std::remove(file.c_str()); }
    BT::Tree tree(const std::string &extra = "")
    {
        auto bb = BT::Blackboard::create();
        bb->set("file", file);
        return factory.createTreeFromText(
          "<root BTCPP_format='4'><BehaviorTree ID='Test'><Sequence><GetEntityPoses "
          "config_file='{file}' target_frame='world' service_namespace='/scene_entities_test' " +
              extra +
              " poses='{poses}' status_message='{message}'/>"
              "<BuildPlanningScene config_file='{file}' poses='{poses}' "
              "scene='{scene}'/></Sequence></BehaviorTree></root>",
          bb
    );
    }
    bool wait(const std::function<bool()> &predicate)
    {
        auto end = std::chrono::steady_clock::now() + 2s;
        do {
            executor.spin_some();
            if (predicate()) { return true; }
            std::this_thread::sleep_for(1ms);
        } while (std::chrono::steady_clock::now() < end);
        return false;
    }
    void respond(size_t index, uint8_t code = 1, const std::string &frame = "sim")
    {
        Service::Response response;
        response.result.result            = code;
        response.state.header.frame_id    = frame;
        response.state.header.stamp.sec   = 42;
        response.state.pose.position.x    = index + 1;
        response.state.pose.orientation.w = 1;
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
TEST_F(SceneEntitiesTest, BuildsAllPrimitivesAndSelectsDynamicIris)
{
    auto t = tree();
    for (size_t i = 0; i < 3; ++i) {
        ASSERT_TRUE(wait([&] {
            t.tickOnce();
            return headers.size() == i + 1;
        }));
        respond(i);
    }
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    auto scene = t.rootBlackboard()->get<moveit_msgs::msg::PlanningScene>("scene");
    EXPECT_TRUE(scene.is_diff);
    EXPECT_TRUE(scene.robot_state.is_diff);
    ASSERT_EQ(scene.world.collision_objects.size(), 3u);
    const auto &cube = scene.world.collision_objects[0];
    EXPECT_EQ(cube.id, "test/cube");
    EXPECT_EQ(cube.header.frame_id, "world");
    EXPECT_NEAR(cube.pose.position.x, 1, 1e-9);
    EXPECT_NEAR(cube.pose.position.y, 3, 1e-9);
    EXPECT_NEAR(cube.pose.position.z, 3, 1e-9);
    EXPECT_NEAR(cube.pose.orientation.z, std::sqrt(0.5), 1e-9);
    EXPECT_DOUBLE_EQ(cube.primitive_poses[0].position.x, 1); // Offset remains in the object frame.
    EXPECT_DOUBLE_EQ(cube.primitive_poses[0].orientation.w, 1);
    EXPECT_EQ(
      scene.world.collision_objects[1].primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER
    );
    EXPECT_DOUBLE_EQ(scene.world.collision_objects[1].primitives[0].dimensions[0], 0.12);
    EXPECT_EQ(scene.world.collision_objects[2].primitives.size(), 2u);
    EXPECT_EQ(
      scene.world.collision_objects[2].primitives[1].type, shape_msgs::msg::SolidPrimitive::SPHERE
    );
    EXPECT_EQ(t.rootBlackboard()->get<EntityPoses>("poses").size(), 3u);
    t.haltTree();
    auto selected = tree("uris='{selection}' override_frame_id='world'");
    selected.rootBlackboard()->set("selection", std::vector<std::string>{ "urn:can" });
    ASSERT_TRUE(wait([&] {
        selected.tickOnce();
        return headers.size() == 4;
    }));
    EXPECT_EQ(entities.back(), "/can");
    respond(3, 1, "incorrect_frame");
    EXPECT_EQ(finish(selected), BT::NodeStatus::SUCCESS);
    auto only = selected.rootBlackboard()->get<moveit_msgs::msg::PlanningScene>("scene");
    ASSERT_EQ(only.world.collision_objects.size(), 1u);
    EXPECT_EQ(only.world.collision_objects[0].id, "urn:can");
    EXPECT_EQ(only.world.collision_objects[0].header.frame_id, "world");
    EXPECT_DOUBLE_EQ(only.world.collision_objects[0].pose.position.x, 4);
}
TEST_F(SceneEntitiesTest, InvalidCatalogueSelectionAndTransformsFailWithoutPartialOutput)
{
    for (auto selection : { std::vector<std::string>{ "urn:missing" },
                            std::vector<std::string>{ "urn:cube", "urn:cube" } }) {
        auto t = tree("uris='{selection}'");
        t.rootBlackboard()->set("selection", selection);
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    }
    EXPECT_TRUE(headers.empty());
    for (const auto &patch :
         { "type: mesh",
           "type: box\n        dimensions: [-1,1,1]",
           "type: sphere\n        dimensions: [1,2]",
           "type: box\n        dimensions: [1,1,1]\n        orientation: [0,0,0,0]" }) {
        EXPECT_ANY_THROW(readSceneEntities(
          YAML::Load(
            std::string("entities:\n  - uri: urn:test\n    id: test\n    shapes:\n      - ") + patch
          )
        ));
    }
    auto duplicate = YAML::Load(catalogue);
    duplicate["entities"].push_back(YAML::Clone(duplicate["entities"][0]));
    EXPECT_THROW(readSceneEntities(duplicate), std::invalid_argument);
    auto t = tree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return headers.size() == 1;
    }));
    respond(0, 1, "unknown_frame");
    EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(t.rootBlackboard()->getEntry("scene")->value.empty());
    t.haltTree();
    ASSERT_TRUE(wait([&] {
        t.tickOnce();
        return headers.size() == 2;
    }));
    respond(1, 2);
    EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(t.rootBlackboard()->getEntry("scene")->value.empty());
    auto partial = tree();
    ASSERT_TRUE(wait([&] {
        partial.tickOnce();
        return headers.size() == 3;
    }));
    respond(2);
    ASSERT_TRUE(wait([&] {
        partial.tickOnce();
        return headers.size() == 4;
    }));
    respond(3, 2);
    EXPECT_EQ(finish(partial), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(partial.rootBlackboard()->getEntry("scene")->value.empty());
}
TEST_F(SceneEntitiesTest, BuildsFromBlackboardWithoutSimulationMappings)
{
    auto geometry = YAML::Load(catalogue);
    for (auto entry : geometry["entities"]) { entry.remove("id"); }
    std::ofstream(file) << geometry;
    auto bb = BT::Blackboard::create();
    bb->set("file", file);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "recorded_frame";
    pose.header.stamp.sec   = 17;
    pose.pose.orientation.w = 1;
    bb->set("poses", EntityPoses{ { "urn:can", pose } });
    auto t = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Test'><BuildPlanningScene "
      "config_file='{file}' poses='{poses}' scene='{scene}'/></BehaviorTree></root>",
      bb
    );
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
    const auto scene = bb->get<moveit_msgs::msg::PlanningScene>("scene");
    ASSERT_EQ(scene.world.collision_objects.size(), 1u);
    EXPECT_EQ(scene.world.collision_objects[0].header.stamp.sec, 17);
    EXPECT_EQ(scene.world.collision_objects[0].header.frame_id, "recorded_frame");
    EXPECT_TRUE(headers.empty());
    for (const auto &uri : { "urn:missing", "urn:can" }) {
        t.haltTree();
        pose.header.frame_id.clear();
        bb->set("poses", EntityPoses{ { uri, pose } });
        EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    }
    t.haltTree();
    bb->set("poses", EntityPoses{});
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(SceneEntitiesTest, SeparateAttachmentNodesValidateNativeInputs)
{
    auto bb = BT::Blackboard::create();
    bb->set("file", file);
    bb->set("touches", std::vector<std::string>{ "finger1", "finger2" });
    bb->set("uri", std::string("urn:can"));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "world";
    pose.pose.orientation.w = 1;
    pose.pose.position.x    = 0.2;
    bb->set("pose", pose);
    auto attach = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Test'><AttachObjectByUri "
      "config_file='{file}' object_uri='{uri}' link_name='tool' touch_links='{touches}' "
      "scene='{attached}'/></BehaviorTree></root>",
      bb
    );
    EXPECT_EQ(attach.tickOnce(), BT::NodeStatus::SUCCESS);
    const auto attached = bb->get<moveit_msgs::msg::PlanningScene>("attached");
    EXPECT_TRUE(attached.is_diff);
    EXPECT_TRUE(attached.robot_state.is_diff);
    ASSERT_EQ(attached.robot_state.attached_collision_objects.size(), 1u);
    const auto &held = attached.robot_state.attached_collision_objects[0];
    EXPECT_EQ(held.object.id, "urn:can");
    EXPECT_EQ(held.object.operation, held.object.ADD);
    EXPECT_EQ(held.touch_links.size(), 2u);
    auto detach = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Test'><DetachObjectByUri "
      "config_file='{file}' object_uri='{uri}' link_name='tool' planning_frame='world' "
      "object_pose='{pose}' scene='{detached}'/></BehaviorTree></root>",
      bb
    );
    EXPECT_EQ(detach.tickOnce(), BT::NodeStatus::SUCCESS);
    const auto detached = bb->get<moveit_msgs::msg::PlanningScene>("detached");
    ASSERT_EQ(detached.world.collision_objects.size(), 1u);
    EXPECT_EQ(detached.world.collision_objects[0].id, held.object.id);
    EXPECT_DOUBLE_EQ(detached.world.collision_objects[0].pose.position.x, 0.2);
    EXPECT_EQ(
      detached.robot_state.attached_collision_objects[0].object.operation, held.object.REMOVE
    );
    detach.haltTree();
    pose.header.frame_id = "wrong_frame";
    bb->set("pose", pose);
    EXPECT_EQ(detach.tickOnce(), BT::NodeStatus::FAILURE);
    detach.haltTree();
    pose.header.frame_id    = "world";
    pose.pose.orientation.w = 0;
    bb->set("pose", pose);
    EXPECT_EQ(detach.tickOnce(), BT::NodeStatus::FAILURE);
    attach.haltTree();
    bb->set("uri", std::string("urn:unknown"));
    EXPECT_EQ(attach.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(SceneEntitiesTest, CleanupSelectsUrisAndPreservesUnrelatedObjects)
{
    using Query = moveit_msgs::srv::GetPlanningScene;
    moveit_msgs::msg::PlanningScene current;
    auto                            query = node->create_service<Query>(
      "/scene_cleanup_test/get",
      [&](const Query::Request::SharedPtr request, Query::Response::SharedPtr response) {
          EXPECT_EQ(request->components.components, 12u);
          response->scene = current;
      }
    );
    auto bb = BT::Blackboard::create();
    bb->set("file", file);
    bb->set("selection", std::vector<std::string>{ "urn:cube" });
    auto t = factory.createTreeFromText(
      "<root BTCPP_format='4'><BehaviorTree ID='Test'><ClearPlanningSceneForUris "
      "config_file='{file}' uris='{selection}' service_name='/scene_cleanup_test/get' "
      "scene='{clean}'/></BehaviorTree></root>",
      bb
    );
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(bb->get<moveit_msgs::msg::PlanningScene>("clean").world.collision_objects.empty());
    moveit_msgs::msg::CollisionObject foreign;
    foreign.id                      = "foreign/object";
    current.world.collision_objects = { foreign,
                                        readSceneEntities(YAML::LoadFile(file))[1].object };
    moveit_msgs::msg::AttachedCollisionObject held;
    held.link_name                                 = "tool";
    held.object.id                                 = "test/cube";
    auto unrelated                                 = held;
    unrelated.object.id                            = "foreign/attached";
    current.robot_state.attached_collision_objects = { held, unrelated };
    t.haltTree();
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    auto clean = bb->get<moveit_msgs::msg::PlanningScene>("clean");
    ASSERT_EQ(clean.world.collision_objects.size(), 1u);
    EXPECT_EQ(clean.world.collision_objects[0].id, "test/cube");
    EXPECT_EQ(clean.world.collision_objects[0].operation, foreign.REMOVE);
    ASSERT_EQ(clean.robot_state.attached_collision_objects.size(), 1u);
    EXPECT_EQ(clean.robot_state.attached_collision_objects[0].link_name, "tool");
    t.haltTree();
    bb->set("selection", std::vector<std::string>{});
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(bb->get<moveit_msgs::msg::PlanningScene>("clean").world.collision_objects.size(), 2u);
    t.haltTree();
    bb->set("selection", std::vector<std::string>{ "urn:unknown" });
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(SceneCatalogue, ShippedExampleLoads)
{ EXPECT_NO_THROW(readSceneEntities(YAML::LoadFile(EXAMPLE_ENTITY_CONFIG))); }
