// Copyright 2026 Minh Nguyen
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "bdd_bt_executor_ros2/nodes/get_entity_poses.hpp"

namespace bdd_bt_executor_ros2 {
class SelectEntityPose : public BT::SyncActionNode
{
  public:
    using BT::SyncActionNode::SyncActionNode;
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<EntityPoses>("poses"),
                 BT::InputPort<std::string>("object_uri"),
                 BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus tick() override
    {
        try {
            EntityPoses poses;
            std::string uri;
            if (!getInput("poses", poses) || !getInput("object_uri", uri) || uri.empty())
                throw std::invalid_argument("require poses and object_uri");
            const auto &pose = poses.at(uri);
            validatePose(pose.pose);
            if (pose.header.frame_id.empty()) throw std::invalid_argument("empty pose frame");
            setOutput("pose", pose);
            setOutput("status_message", std::string());
            return BT::NodeStatus::SUCCESS;
        } catch (const std::exception &error) {
            setOutput("status_message", std::string(error.what()));
            config().blackboard->set("behaviour_status", name() + ": " + error.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};

class OffsetPose : public BT::SyncActionNode
{
  public:
    using BT::SyncActionNode::SyncActionNode;
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
            BT::InputPort<std::vector<double>>("position_offset", "0;0;0", "Translation in metres"),
            BT::InputPort<std::vector<double>>(
              "orientation_offset", "0;0;0;1", "Relative quaternion xyzw"
            ),
            BT::InputPort<std::vector<double>>("orientation"),
            BT::InputPort<std::string>("offset_frame", "local", "local or reference"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            BT::OutputPort<std::string>("status_message")
        };
    }
    BT::NodeStatus tick() override
    {
        try {
            geometry_msgs::msg::PoseStamped pose;
            std::vector<double>             translation, rotation;
            std::string                     frame;
            if (
              !getInput("pose", pose) || !getInput("position_offset", translation)
              || !getInput("orientation_offset", rotation) || !getInput("offset_frame", frame)
            )
                throw std::invalid_argument("missing pose or invalid offset input");
            validatePose(pose.pose);
            if (pose.header.frame_id.empty()) throw std::invalid_argument("empty pose frame");
            if (translation.size() != 3 || (frame != "local" && frame != "reference"))
                throw std::invalid_argument(
                  "require xyz position_offset and local or reference offset_frame"
                );
            auto quaternion = [](const std::vector<double> &values) {
                if (values.size() != 4) throw std::invalid_argument("require xyzw quaternion");
                geometry_msgs::msg::Pose check;
                check.orientation.x = values[0];
                check.orientation.y = values[1];
                check.orientation.z = values[2];
                check.orientation.w = values[3];
                validatePose(check);
                tf2::Quaternion q;
                tf2::fromMsg(check.orientation, q);
                q.normalize();
                return q;
            };
            tf2::Quaternion base;
            tf2::fromMsg(pose.pose.orientation, base);
            base.normalize();
            if (config().input_ports.count("orientation")) {
                std::vector<double> absolute;
                if (!getInput("orientation", absolute))
                    throw std::invalid_argument("invalid orientation");
                base = quaternion(absolute);
            }
            for (double value : translation)
                if (!std::isfinite(value)) throw std::invalid_argument("nonfinite position_offset");
            tf2::Vector3 delta(translation[0], translation[1], translation[2]);
            if (frame == "local") delta = tf2::quatRotate(base, delta);
            pose.pose.position.x += delta.x();
            pose.pose.position.y += delta.y();
            pose.pose.position.z += delta.z();
            const auto relative = quaternion(rotation);
            auto       result   = frame == "local" ? base * relative : relative * base;
            result.normalize();
            pose.pose.orientation = tf2::toMsg(result);
            validatePose(pose.pose);
            setOutput("target_pose", pose);
            setOutput("status_message", std::string());
            return BT::NodeStatus::SUCCESS;
        } catch (const std::exception &error) {
            setOutput("status_message", std::string(error.what()));
            config().blackboard->set("behaviour_status", name() + ": " + error.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};
} // namespace bdd_bt_executor_ros2
