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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <bdd_ros2_interfaces/action/behaviour.hpp>
#include <bdd_ros2_interfaces/msg/trinary.hpp>

#include "bdd_bt_executor_ros2/bdd_bt_executor.hpp"
#include "bdd_bt_executor_ros2/nodes/motion_actions.hpp"

using namespace std::chrono_literals;
using Move    = moveit_msgs::action::MoveGroup;
using Gripper = control_msgs::action::GripperCommand;
using bdd_bt_executor_ros2::BddBtExecutor;
using bdd_bt_executor_ros2::MoveGroupAction;
using bdd_bt_executor_ros2::GripperCommandAction;

template<class Action> struct FakeServer
{
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    rclcpp::Node::SharedPtr                           node;
    typename rclcpp_action::Server<Action>::SharedPtr server;
    struct State
    {
        std::atomic<int>                     requests{ 0 }, cancellations{ 0 };
        std::atomic<int>                     acceptance_delay_ms{ 0 };
        std::atomic<bool>                    reject{ false }, reject_cancel{ false };
        std::mutex                           mutex;
        std::vector<std::shared_ptr<Handle>> handles;
    };
    std::shared_ptr<State> state = std::make_shared<State>();
    FakeServer(rclcpp::Node::SharedPtr n, const std::string &endpoint) : node(std::move(n))
    {
        server = rclcpp_action::create_server<Action>(
          node,
          endpoint,
          [data = state](const auto &, auto) {
              ++data->requests;
              std::this_thread::sleep_for(
                std::chrono::milliseconds(data->acceptance_delay_ms.load())
              );
              return data->reject ? rclcpp_action::GoalResponse::REJECT
                                  : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [data = state](auto) {
              ++data->cancellations;
              return data->reject_cancel ? rclcpp_action::CancelResponse::REJECT
                                         : rclcpp_action::CancelResponse::ACCEPT;
          },
          [data = state](auto handle) {
              std::lock_guard<std::mutex> lock(data->mutex);
              data->handles.push_back(handle);
          }
        );
    }
    size_t size()
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        return state->handles.size();
    }
    std::shared_ptr<Handle> handle(size_t i = 0)
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        return state->handles.at(i);
    }
};

class MotionActionsTest : public testing::Test
{
  protected:
    static void                              SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void                              TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr                  client_node, server_node;
    rclcpp::executors::MultiThreadedExecutor executor{ rclcpp::ExecutorOptions(), 2 };
    std::thread                              spinner;
    BT::BehaviorTreeFactory                  factory;
    void                                     SetUp() override
    {
        client_node = std::make_shared<rclcpp::Node>(
          "test_motion_client",
          rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter("use_sim_time", true) })
        );
        server_node = std::make_shared<rclcpp::Node>("test_motion_server");
        executor.add_node(client_node);
        executor.add_node(server_node);
        spinner = std::thread([this]() { executor.spin(); });
        factory.registerBuilder<MoveGroupAction>(
          "MoveGroupAction", [this](const auto &name, const auto &config) {
              return std::make_unique<MoveGroupAction>(name, config, client_node);
          }
        );
        factory.registerBuilder<GripperCommandAction>(
          "GripperCommandAction", [this](const auto &name, const auto &config) {
              return std::make_unique<GripperCommandAction>(name, config, client_node);
          }
        );
    }
    void TearDown() override
    {
        // Let bounded cancellation retention finish before shutting down ROS.
        EXPECT_TRUE(wait([&]() { return client_node.use_count() == 1; }));
        executor.cancel();
        spinner.join();
    }
    bool wait(const std::function<bool()> &predicate)
    {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) { return true; }
            std::this_thread::sleep_for(2ms);
        }
        return false;
    }
    BT::Tree tree(const std::string &xml, BT::Blackboard::Ptr bb = BT::Blackboard::create())
    {
        return factory.createTreeFromText(
          "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + xml + "</BehaviorTree></root>", bb
        );
    }
    BT::Tree moveTree(double timeout = 2.0)
    {
        auto       bb = BT::Blackboard::create();
        Move::Goal goal;
        goal.request.group_name            = "test_arm";
        goal.request.allowed_planning_time = 1.0;
        goal.request.goal_constraints.resize(1);
        bb->set("goal", goal);
        return tree("<MoveGroupAction goal=\"{goal}\" action_name=\"/test_move\" timeout=\"" +
      std::to_string(timeout) +
              "\" cancel_timeout=\"0.3\" feedback_state=\"{feedback}\" "
              "error_code=\"{code}\" status_message=\"{status}\"/>",
      bb);
    }
    BT::NodeStatus finish(BT::Tree &tree)
    {
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        EXPECT_TRUE(wait([&]() {
            status = tree.tickOnce();
            return status != BT::NodeStatus::RUNNING;
        }));
        return status;
    }
};

TEST_F(MotionActionsTest, MoveFeedbackAndResultAndSingleSubmission)
{
    FakeServer<Move> server(server_node, "/test_move");
    auto             t = moveTree();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return server.size() == 1;
    }));
    auto feedback   = std::make_shared<Move::Feedback>();
    feedback->state = "MONITORING";
    server.handle()->publish_feedback(feedback);
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        std::string text;
        return t.rootBlackboard()->get("feedback", text) && text == "MONITORING";
    }));
    EXPECT_NE(
      t.rootBlackboard()->get<std::string>("behaviour_status").find("MONITORING"), std::string::npos
    );
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i) { EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING); }
    EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
    EXPECT_EQ(server.state->requests, 1);
    auto result            = std::make_shared<Move::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    server.handle()->succeed(result);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(t.rootBlackboard()->get<int>("code"), 1);
}

TEST_F(MotionActionsTest, MoveRequiresBothRosAndMoveItSuccess)
{
    FakeServer<Move> server(server_node, "/test_move");
    for (int i = 0; i < 2; ++i) {
        auto t = moveTree();
        ASSERT_TRUE(wait([&]() {
            t.tickOnce();
            return server.size() == static_cast<size_t>(i + 1);
        }));
        auto result            = std::make_shared<Move::Result>();
        result->error_code.val = i == 0 ? moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED
                                        : moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        if (i == 0) {
            server.handle(i)->succeed(result);
        } else {
            server.handle(i)->abort(result);
        }
        EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
        EXPECT_EQ(t.rootBlackboard()->get<int>("code"), result->error_code.val);
    }
}

TEST_F(MotionActionsTest, MissingServerAndRejectedGoal)
{
    auto unavailable = moveTree(0.08);
    EXPECT_EQ(finish(unavailable), BT::NodeStatus::FAILURE);
    EXPECT_NE(
      unavailable.rootBlackboard()->get<std::string>("status").find("timeout"), std::string::npos
    );
    FakeServer<Move> server(server_node, "/test_move");
    server.state->reject = true;
    auto rejected        = moveTree();
    EXPECT_EQ(finish(rejected), BT::NodeStatus::FAILURE);
    EXPECT_EQ(rejected.rootBlackboard()->get<std::string>("status"), "goal rejected");
}

TEST_F(MotionActionsTest, HaltBeforeAcceptanceSurvivesTreeDestruction)
{
    FakeServer<Move> server(server_node, "/test_move");
    server.state->acceptance_delay_ms = 120;
    {
        auto t = moveTree();
        ASSERT_TRUE(wait([&]() {
            t.tickOnce();
            return server.state->requests == 1;
        }));
        const auto start = std::chrono::steady_clock::now();
        t.haltTree();
        EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
    }
    ASSERT_TRUE(wait([&]() { return server.state->cancellations == 1; }));
    ASSERT_TRUE(wait([&]() { return server.handle()->is_canceling(); }));
    server.handle()->canceled(std::make_shared<Move::Result>());
}

TEST_F(MotionActionsTest, TimeoutAndOldResultDoNotAffectNextInvocation)
{
    FakeServer<Move> server(server_node, "/test_move");
    auto             t = moveTree(0.3);
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return server.size() == 1;
    }));
    // No /clock publisher: timeout still progresses.
    EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
    auto canceled = [&]() {
        return server.state->cancellations == 1 && server.handle()->is_canceling();
    };
    ASSERT_TRUE(wait(canceled));
    t.haltTree();
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return server.size() == 2;
    }));
    auto old            = std::make_shared<Move::Result>();
    old->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    server.handle(0)->canceled(old);
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    auto current            = std::make_shared<Move::Result>();
    current->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    server.handle(1)->succeed(current);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
}

TEST_F(MotionActionsTest, CancellationRejectionIsReportedAndRetentionIsBounded)
{
    FakeServer<Move> server(server_node, "/test_move");
    server.state->reject_cancel = true;
    auto t                      = moveTree();
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return server.size() == 1;
    }));
    testing::internal::CaptureStderr();
    t.haltTree();
    // Destroy the tree so logging cannot rely on a BT tick or node lifetime.
    t = BT::Tree();
    EXPECT_TRUE(wait([&]() { return server.state->cancellations == 1; }));
    std::this_thread::sleep_for(350ms);
    const auto log = testing::internal::GetCapturedStderr();
    EXPECT_NE(log.find("cancellation rejected; stop unconfirmed"), std::string::npos);
    EXPECT_NE(log.find("Cancellation deadline expired; stop unconfirmed"), std::string::npos);
    server.handle()->abort(std::make_shared<Move::Result>());
}

TEST_F(MotionActionsTest, ActionEndpointDefaultsAndEmptyOverrides)
{
    auto valid        = moveTree();
    auto default_move = tree("<MoveGroupAction goal=\"{goal}\"/>", valid.rootBlackboard());
    EXPECT_EQ(
      default_move.rootNode()->getInput<std::string>("action_name").value(), "/move_action"
    );
    auto default_gripper = tree("<GripperCommandAction position=\"0.02\"/>");
    EXPECT_EQ(
      default_gripper.rootNode()->getInput<std::string>("action_name").value(), "/gripper_cmd"
    );
    for (const std::string endpoint : { " action_name=\"\"" }) {
        auto move =
          tree("<MoveGroupAction goal=\"{goal}\"" + endpoint + "/>", valid.rootBlackboard());
        EXPECT_EQ(move.tickOnce(), BT::NodeStatus::FAILURE);
        EXPECT_NE(
          move.rootBlackboard()->get<std::string>("behaviour_status").find("action_name"),
          std::string::npos
        );
        auto gripper = tree("<GripperCommandAction position=\"0.02\"" + endpoint + "/>");
        EXPECT_EQ(gripper.tickOnce(), BT::NodeStatus::FAILURE);
        EXPECT_NE(
          gripper.rootBlackboard()->get<std::string>("behaviour_status").find("action_name"),
          std::string::npos
        );
    }
}

TEST_F(MotionActionsTest, GripperOutputsAndInputValidation)
{
    FakeServer<Gripper> server(server_node, "/test_gripper");
    auto                t = tree(
      "<GripperCommandAction position=\"0.02\" max_effort=\"5\" action_name=\"/test_gripper\" "
      "result_position=\"{position}\" reached_goal=\"{reached}\" "
      "stalled=\"{stalled}\"/>"
    );
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return server.size() == 1;
    }));
    EXPECT_DOUBLE_EQ(server.handle()->get_goal()->command.position, 0.02);
    EXPECT_DOUBLE_EQ(server.handle()->get_goal()->command.max_effort, 5.0);
    auto result      = std::make_shared<Gripper::Result>();
    result->position = 0.015;
    result->stalled  = true;
    server.handle()->succeed(result);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(t.rootBlackboard()->get<double>("position"), 0.015);
    EXPECT_TRUE(t.rootBlackboard()->get<bool>("stalled"));
    EXPECT_FALSE(t.rootBlackboard()->get<bool>("reached"));
    for (const std::string position : { "-1", "nan", "inf" }) {
        auto invalid = tree(
          "<GripperCommandAction action_name=\"/test_gripper\" position=\"" + position + "\"/>"
        );
        EXPECT_EQ(invalid.tickOnce(), BT::NodeStatus::FAILURE);
    }
    auto invalid_timeout =
      tree("<GripperCommandAction action_name=\"/test_gripper\" position=\"0\" timeout=\"nan\"/>");
    EXPECT_EQ(invalid_timeout.tickOnce(), BT::NodeStatus::FAILURE);
    auto invalid_move = moveTree();
    invalid_move.rootBlackboard()->set("goal", Move::Goal());
    EXPECT_EQ(invalid_move.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(server.state->requests, 1);
}

// Application nodes receive the immutable Behaviour goal and prepare native goals.
class PrepareMove : public BT::SyncActionNode
{
  public:
    PrepareMove(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus       tick() override
    {
        auto goal =
          config().blackboard->get<std::shared_ptr<const BddBtExecutor::BehaviourAction::Goal>>(
            "behaviour_goal"
          );
        if (goal->scenario_context_id.uuid[0] != 42) { return BT::NodeStatus::FAILURE; }
        Move::Goal move;
        move.request.group_name            = "test_arm";
        move.request.allowed_planning_time = 1.0;
        move.request.goal_constraints.resize(1);
        config().blackboard->set("motion_goal", move);
        return BT::NodeStatus::SUCCESS;
    }
};

TEST_F(MotionActionsTest, BehaviourGoalAndFeedbackReachApplicationNodes)
{
    FakeServer<Move> server(server_node, "/test_move");
    const auto       path = "/tmp/bdd-bt-stage2-" + std::to_string(getpid()) + ".xml";
    {
        std::ofstream xml(path);
        xml << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><Sequence><PrepareMove/>"
               "<MoveGroupAction goal=\"{motion_goal}\" action_name=\"/test_move\"/>"
               "</Sequence></BehaviorTree></root>";
    }
    auto executor_node = std::make_shared<BddBtExecutor>(rclcpp::NodeOptions().parameter_overrides(
      { rclcpp::Parameter("tree_xml", path),
        rclcpp::Parameter("bhv_server_name", "/test_behaviour") }
    ));
    executor_node->set_node_registrar([](auto &f, const auto &) {
        f.template registerNodeType<PrepareMove>("PrepareMove");
    });
    executor.add_node(executor_node);
    using Behaviour = BddBtExecutor::BehaviourAction;
    auto client     = rclcpp_action::create_client<Behaviour>(client_node, "/test_behaviour");
    ASSERT_TRUE(client->wait_for_action_server(3s));
    std::mutex                                                 feedback_mutex;
    std::vector<Behaviour::Feedback>                           messages;
    typename rclcpp_action::Client<Behaviour>::SendGoalOptions options;
    options.feedback_callback = [&](auto, auto message) {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        messages.push_back(*message);
    };
    Behaviour::Goal goal;
    goal.scenario_context_id.uuid[0] = 42;
    auto accepted                    = client->async_send_goal(goal, options);
    ASSERT_EQ(accepted.wait_for(3s), std::future_status::ready);
    ASSERT_NE(accepted.get(), nullptr);
    auto result = client->async_get_result(accepted.get());
    ASSERT_TRUE(wait([&]() { return server.size() == 1; }));
    auto feedback   = std::make_shared<Move::Feedback>();
    feedback->state = "MONITORING";
    server.handle()->publish_feedback(feedback);
    EXPECT_TRUE(wait([&]() {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        for (const auto &message : messages) {
            if (
              message.status.find("MONITORING") != std::string::npos
              && message.scenario_context_id == goal.scenario_context_id
            ) {
                return true;
            }
        }
        return false;
    }));
    auto motion_result            = std::make_shared<Move::Result>();
    motion_result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    server.handle()->succeed(motion_result);
    ASSERT_EQ(result.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(result.get().result->result.trinary.value, bdd_ros2_interfaces::msg::Trinary::TRUE);
    EXPECT_EQ(result.get().result->result.scenario_context_id, goal.scenario_context_id);
    // A second invocation receives its own blackboard and preserves cancellation semantics.
    std::this_thread::sleep_for(50ms);
    auto second = client->async_send_goal(goal, options);
    ASSERT_EQ(second.wait_for(3s), std::future_status::ready);
    ASSERT_NE(second.get(), nullptr);
    auto canceled_result = client->async_get_result(second.get());
    ASSERT_TRUE(wait([&]() { return server.size() == 2; }));
    auto cancellation = client->async_cancel_goal(second.get());
    ASSERT_EQ(cancellation.wait_for(3s), std::future_status::ready);
    ASSERT_EQ(canceled_result.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(
      canceled_result.get().result->result.trinary.value, bdd_ros2_interfaces::msg::Trinary::UNKNOWN
    );
    EXPECT_EQ(canceled_result.get().result->result.scenario_context_id, goal.scenario_context_id);
    ASSERT_TRUE(wait([&]() { return server.handle(1)->is_canceling(); }));
    server.handle(1)->canceled(std::make_shared<Move::Result>());
    // Keep spinning until the worker and canceled child callbacks release the node.
    std::weak_ptr<BddBtExecutor> pending = executor_node;
    executor_node.reset();
    EXPECT_TRUE(wait([&]() { return pending.expired(); }));
    std::remove(path.c_str());
}

TEST_F(MotionActionsTest, ClientIsLazyReusedAndReplacedWhenEndpointChanges)
{
    FakeServer<Move> first(server_node, "/test_move");
    FakeServer<Move> second(server_node, "/other_move");
    auto             valid = moveTree();
    auto             bb    = valid.rootBlackboard();
    bb->set("endpoint", std::string("/test_move"));
    auto t           = tree("<MoveGroupAction goal=\"{goal}\" action_name=\"{endpoint}\"/>", bb);
    auto find_client = [&]() {
        return std::dynamic_pointer_cast<rclcpp_action::Client<Move>>(
          client_node->get_node_base_interface()
            ->get_default_callback_group()
            ->find_waitable_ptrs_if([&](auto waitable) {
                auto client = std::dynamic_pointer_cast<rclcpp_action::Client<Move>>(waitable);
                return client != nullptr;
            })
        );
    };
    EXPECT_EQ(find_client(), nullptr);
    std::weak_ptr<rclcpp_action::Client<Move>> original;
    for (size_t i = 0; i < 2; ++i) {
        ASSERT_TRUE(wait([&]() {
            t.tickOnce();
            return first.size() == i + 1;
        }));
        ASSERT_NE(find_client(), nullptr);
        if (i == 0) {
            original = find_client();
        } else {
            EXPECT_EQ(find_client(), original.lock());
        }
        auto result            = std::make_shared<Move::Result>();
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        first.handle(i)->succeed(result);
        EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
        EXPECT_FALSE(original.expired());
    }
    bb->set("endpoint", std::string("/other_move"));
    ASSERT_TRUE(wait([&]() {
        t.tickOnce();
        return second.size() == 1;
    }));
    EXPECT_TRUE(original.expired());
    EXPECT_NE(find_client(), nullptr);
    auto result            = std::make_shared<Move::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    second.handle()->succeed(result);
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
}

TEST_F(MotionActionsTest, NamedTargetResolvesAndRejectsInvalidInputs)
{
    client_node->declare_parameter<std::string>("robot_description_semantic", "");
    auto named = tree(
      "<MoveGroupAction group=\"test_arm\" named_target=\"ready\" action_name=\"/test_move\"/>"
    );
    EXPECT_EQ(named.tickOnce(), BT::NodeStatus::FAILURE);
    client_node->set_parameter(
      rclcpp::Parameter(
        "robot_description_semantic",
        "<robot name='test'><group_state group='test_arm' name='ready'>"
        "<joint name='joint1' value='-0.785'/></group_state></robot>"
      )
    );
    FakeServer<Move> server(server_node, "/test_move");
    named.haltTree();
    EXPECT_EQ(named.tickOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(wait([&]() { return server.size() == 1; }));
    const auto &goal = *server.handle()->get_goal();
    EXPECT_EQ(goal.request.group_name, "test_arm");
    ASSERT_EQ(goal.request.goal_constraints.size(), 1u);
    const auto &constraints = goal.request.goal_constraints.front();
    EXPECT_EQ(constraints.name, "ready");
    ASSERT_EQ(constraints.joint_constraints.size(), 1u);
    EXPECT_EQ(constraints.joint_constraints.front().joint_name, "joint1");
    EXPECT_DOUBLE_EQ(constraints.joint_constraints.front().position, -0.785);
    EXPECT_TRUE(goal.request.start_state.is_diff);
    EXPECT_FALSE(goal.planning_options.plan_only);
    auto result            = std::make_shared<Move::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    server.handle()->succeed(result);
    EXPECT_EQ(finish(named), BT::NodeStatus::SUCCESS);
    auto valid = moveTree();
    for (const auto &xml :
         { "<MoveGroupAction group='test_arm' named_target='missing'/>",
           "<MoveGroupAction group='test_arm'/>",
           "<MoveGroupAction goal='{goal}' group='test_arm' named_target='ready'/>",
           "<MoveGroupAction group='test_arm' named_target='ready' planning_time='0'/>",
           "<MoveGroupAction group='test_arm' named_target='ready' velocity_scaling='2'/>" }) {
        auto invalid = tree(xml, valid.rootBlackboard());
        EXPECT_EQ(invalid.tickOnce(), BT::NodeStatus::FAILURE);
    }
    for (const auto &srdf : { "not XML",
                              "<robot/>",
                              "<robot><group_state group='test_arm' name='ready'/></robot>",
                              "<robot><group_state group='test_arm' name='ready'><joint name='j' "
                              "value='0,5'/></group_state></robot>",
                              "<robot><group_state group='test_arm' name='ready'><joint name='j' "
                              "value='1 2'/></group_state></robot>",
                              "<robot><group_state group='test_arm' name='ready'><joint name='j' "
                              "value='nan'/></group_state></robot>",
                              "<robot><group_state group='test_arm' name='ready'><joint name='j' "
                              "value='1'/><joint name='j' value='2'/></group_state></robot>" }) {
        client_node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));
        auto invalid = tree("<MoveGroupAction group='test_arm' named_target='ready'/>");
        EXPECT_EQ(invalid.tickOnce(), BT::NodeStatus::FAILURE);
    }
    EXPECT_EQ(server.size(), 1);
}

TEST_F(MotionActionsTest, NamedGoalsExampleMovesThenClosesThenReturnsThenOpens)
{
    client_node->declare_parameter<std::string>(
      "robot_description_semantic",
      "<robot name='panda'><group_state group='panda_arm' name='extended'>"
      "<joint name='panda_joint1' value='0'/></group_state>"
      "<group_state group='panda_arm' name='ready'>"
      "<joint name='panda_joint1' value='0.5'/></group_state></robot>"
    );
    FakeServer<Move>    arm(server_node, "/move_action");
    FakeServer<Gripper> gripper(server_node, "/panda_hand_controller/gripper_cmd");
    auto                t          = factory.createTreeFromFile(NAMED_GOALS_XML);
    auto                tick_until = [&](const auto &predicate) {
        return wait([&]() {
            t.tickOnce();
            return predicate();
        });
    };
    ASSERT_TRUE(tick_until([&]() { return arm.size() == 1; }));
    EXPECT_EQ(arm.handle(0)->get_goal()->request.goal_constraints.front().name, "extended");
    EXPECT_EQ(gripper.size(), 0u);
    auto arm_result            = std::make_shared<Move::Result>();
    arm_result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    arm.handle(0)->succeed(arm_result);
    ASSERT_TRUE(tick_until([&]() { return gripper.size() == 1; }));
    EXPECT_DOUBLE_EQ(gripper.handle(0)->get_goal()->command.position, 0.0);
    EXPECT_EQ(arm.size(), 1u);
    gripper.handle(0)->succeed(std::make_shared<Gripper::Result>());
    ASSERT_TRUE(tick_until([&]() { return arm.size() == 2; }));
    EXPECT_EQ(arm.handle(1)->get_goal()->request.goal_constraints.front().name, "ready");
    EXPECT_EQ(gripper.size(), 1u);
    arm.handle(1)->succeed(arm_result);
    ASSERT_TRUE(tick_until([&]() { return gripper.size() == 2; }));
    EXPECT_DOUBLE_EQ(gripper.handle(1)->get_goal()->command.position, 0.04);
    gripper.handle(1)->succeed(std::make_shared<Gripper::Result>());
    EXPECT_EQ(finish(t), BT::NodeStatus::SUCCESS);
}
