// Copyright 2026 Minh Nguyen
// SPDX-License-Identifier: Apache-2.0
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <rcl/time.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include "bdd_bt_executor_ros2/nodes/wait_for_moveit_ready.hpp"

using namespace std::chrono_literals;
using bdd_bt_executor_ros2::WaitForMoveItReady;

class MoveItReadinessTest : public testing::Test
{
  protected:
    static void                               SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void                               TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr                   node;
    rclcpp::executors::SingleThreadedExecutor executor;
    BT::BehaviorTreeFactory                   factory;
    rclcpp_action::Server<WaitForMoveItReady::Action>::SharedPtr server;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joints;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr       transforms;

    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>(
          "readiness_test",
          rclcpp::NodeOptions().parameter_overrides({ rclcpp::Parameter("use_sim_time", true) })
        );
        node->declare_parameter<std::string>("robot_description", R"(
          <robot name="test"><link name="base"/><link name="middle"/><link name="tip"/>
          <joint name="joint1" type="continuous"><parent link="base"/><child link="middle"/></joint>
          <joint name="joint2" type="continuous"><parent link="middle"/><child link="tip"/></joint>
          </robot>)");
        node->declare_parameter<std::string>(
          "robot_description_semantic", R"(
          <robot name="test"><group name="arm"><chain base_link="base" tip_link="tip"/></group></robot>)"
        );
        executor.add_node(node);
        setTime(100.0);
        factory.registerBuilder<WaitForMoveItReady>(
          "WaitForMoveItReady", [this](const auto &name, const auto &config) {
              return std::make_unique<WaitForMoveItReady>(name, config, node);
          }
        );
        joints = node->create_publisher<sensor_msgs::msg::JointState>("/readiness_test/joints", 10);
        transforms = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }
    void setTime(double seconds)
    {
        ASSERT_EQ(
          rcl_set_ros_time_override(
            node->get_clock()->get_clock_handle(), static_cast<int64_t>(seconds * 1e9)
          ),
          RCL_RET_OK
        );
    }
    BT::Tree tree(double timeout = 1.0)
    {
        return factory.createTreeFromText(
          "<root BTCPP_format='4'><BehaviorTree ID='Test'><WaitForMoveItReady "
          "action_name='/readiness_test/move' joint_states_topic='/readiness_test/joints' "
          "group='arm' target_frame='readiness_world' "
          "end_effector_frame='readiness_tool' timeout='"
          + std::to_string(timeout) + "' status_message='{message}'/></BehaviorTree></root>"
        );
    }
    void pump()
    {
        const auto end = std::chrono::steady_clock::now() + 50ms;
        do {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        } while (std::chrono::steady_clock::now() < end);
    }
    void publishJoints(const std::vector<std::string> &names, double stamp)
    {
        sensor_msgs::msg::JointState msg;
        msg.name = names;
        msg.position.resize(names.size(), 0.0);
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
        joints->publish(msg);
        pump();
    }
    void publishTransform(double stamp)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id      = "readiness_world";
        tf.child_frame_id       = "readiness_tool";
        tf.header.stamp         = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
        tf.transform.rotation.w = 1.0;
        tf2_msgs::msg::TFMessage msg;
        msg.transforms.push_back(tf);
        transforms->publish(msg);
        pump();
    }
};

TEST_F(MoveItReadinessTest, RequiresServerAndEveryFreshJointAndTransformOnEachActivation)
{
    auto t = tree();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    using Action = WaitForMoveItReady::Action;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    server       = rclcpp_action::create_server<Action>(
      node,
      "/readiness_test/move",
      [](const auto &, std::shared_ptr<const Action::Goal>) {
          return rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](std::shared_ptr<Handle>) {}
    );
    pump();
    publishJoints({ "joint1", "joint2" }, 99.0);
    publishTransform(99.0);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    setTime(100.1);
    publishJoints({ "joint1" }, 100.1);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    publishJoints({ "joint2" }, 100.1);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    publishTransform(100.1);
    auto status = t.tickOnce();
    for (int i = 0; status == BT::NodeStatus::RUNNING && i < 20; ++i) {
        pump();
        status = t.tickOnce();
    }
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    t.haltTree();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    t.haltTree();
}

TEST_F(MoveItReadinessTest, TimeoutPausesWithNodeClockAndRebasesAfterBackwardJump)
{
    auto t = tree(0.1);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    pump();
    pump();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    setTime(101.0);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_NE(t.rootBlackboard()->get<std::string>("message").find("timed out"), std::string::npos);
    t.haltTree();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    setTime(50.0);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
    setTime(50.2);
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(MoveItReadinessTest, RejectsUnknownGroup)
{
    node->set_parameter(rclcpp::Parameter("robot_description_semantic", "<robot name='test'/>"));
    auto t = tree();
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}
