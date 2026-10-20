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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bdd_bt_executor_ros2/nodes/get_entity_pose.hpp"

namespace bdd_bt_executor_ros2 {
using EntityPoses = std::map<std::string, geometry_msgs::msg::PoseStamped>;
struct EntityMapping
{
    std::string uri;
    std::string id;
};
inline std::vector<EntityMapping> readEntityMappings(const YAML::Node &root)
{
    const auto entries = root["entities"];
    if (!entries.IsSequence() || entries.size() == 0) {
        throw std::invalid_argument("entities must be a nonempty list");
    }
    std::vector<EntityMapping> result;
    std::set<std::string>      seen;
    for (const auto &entry : entries) {
        EntityMapping entity{ entry["uri"].as<std::string>(), entry["id"].as<std::string>() };
        if (entity.uri.empty() || entity.id.empty() || !seen.insert(entity.uri).second) {
            throw std::invalid_argument("empty or duplicate entity URI or empty simulator ID");
        }
        result.push_back(std::move(entity));
    }
    return result;
}
inline void validatePose(const geometry_msgs::msg::Pose &pose)
{
    const auto &p = pose.position;
    const auto &q = pose.orientation;
    for (double v : { p.x, p.y, p.z, q.x, q.y, q.z, q.w }) {
        if (!std::isfinite(v)) { throw std::invalid_argument("nonfinite pose"); }
    }
    if (std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 1e-3) {
        throw std::invalid_argument("pose orientation must be a unit quaternion");
    }
}
inline geometry_msgs::msg::PoseStamped toTargetFrame(
  geometry_msgs::msg::PoseStamped pose,
  const std::string              &target_frame,
  const std::string              &world_frame,
  tf2_ros::Buffer                &tf
)
{
    validatePose(pose.pose);
    if (pose.header.frame_id.empty()) { pose.header.frame_id = world_frame; }
    if (pose.header.frame_id != target_frame) {
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(
          pose,
          transformed,
          tf.lookupTransform(
            target_frame,
            pose.header.frame_id,
            rclcpp::Time(pose.header.stamp),
            rclcpp::Duration::from_seconds(0)
          )
        );
        pose = transformed;
    }
    validatePose(pose.pose);
    return pose;
}
class GetEntityPoses : public BT::StatefulActionNode
{
    using Clock = std::chrono::steady_clock;
    std::shared_ptr<tf2_ros::Buffer>            tf_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<GetEntityPose>              query_;
    BT::Blackboard::Ptr                         query_bb_;
    std::vector<EntityMapping>                  selected_;
    EntityPoses                                 poses_;
    std::string                                 target_frame_, world_frame_;
    Clock::time_point                           deadline_;
    size_t                                      index_    = 0;
    bool                                        querying_ = false;
    void                                        cleanup()
    {
        if (query_) { query_->haltNode(); }
        querying_ = false;
    }
    void statusMessage(const std::string &message)
    {
        setOutput("status_message", message);
        config().blackboard->set("behaviour_status", name() + ": " + message);
    }
    BT::NodeStatus fail(const std::string &message)
    {
        cleanup();
        statusMessage(message);
        return BT::NodeStatus::FAILURE;
    }

  public:
    GetEntityPoses(
      const std::string               &name,
      const BT::NodeConfig            &config,
      rclcpp::Node::SharedPtr          node,
      std::shared_ptr<tf2_ros::Buffer> tf = nullptr
    )
      : BT::StatefulActionNode(name, config), tf_(std::move(tf))
    {
        if (!tf_) {
            tf_       = std::make_shared<tf2_ros::Buffer>(node->get_clock());
            listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, node, false);
        }
        BT::NodeConfig child;
        query_bb_          = BT::Blackboard::create();
        child.blackboard   = query_bb_;
        child.input_ports  = { { "entity_id", "{entity}" },
                               { "service_namespace", "{namespace}" },
                               { "override_frame_id", "{override_frame_id}" },
                               { "timeout", "{timeout}" } };
        child.output_ports = { { "pose", "{pose}" }, { "status_message", "{message}" } };
        query_ = std::make_unique<GetEntityPose>(name + "/pose_query", child, std::move(node));
    }
    ~GetEntityPoses() override { cleanup(); }
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("config_file"),
            BT::InputPort<std::vector<std::string>>("uris"),
            BT::InputPort<std::string>("target_frame"),
            BT::InputPort<std::string>(
              "override_frame_id", "", "Replace response frame before TF; coordinates unchanged"
            ),
            BT::InputPort<std::string>("world_frame", "world", "Frame for empty service headers"),
            BT::InputPort<std::string>("service_namespace", "/", "Simulation service namespace"),
            BT::InputPort<double>("timeout", 5.0, "Total steady-clock timeout"),
            BT::OutputPort<EntityPoses>("poses"),
            BT::OutputPort<std::string>("status_message")
        };
    }
    BT::NodeStatus onStart() override
    {
        cleanup();
        selected_.clear();
        poses_.clear();
        index_ = 0;
        try {
            double      timeout;
            std::string file, ns, override_frame_id;
            auto        now = Clock::now();
            if (
              !getInput("config_file", file) || file.empty()
              || !getInput("target_frame", target_frame_) || target_frame_.empty()
              || !getInput("world_frame", world_frame_) || world_frame_.empty()
              || !getInput("override_frame_id", override_frame_id)
              || !getInput("service_namespace", ns) || !getInput("timeout", timeout)
              || !std::isfinite(timeout) || timeout <= 0
              || timeout >= std::chrono::duration<double>(Clock::time_point::max() - now).count()
            ) {
                throw std::invalid_argument(
                  "require config_file, frames and finite positive timeout"
                );
            }
            deadline_ =
              now
              + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
            auto                     catalogue = readEntityMappings(YAML::LoadFile(file));
            std::vector<std::string> uris;
            if (config().input_ports.count("uris")) {
                auto input = getInput("uris", uris);
                if (!input) { throw std::invalid_argument(input.error()); }
            }
            if (uris.empty()) {
                selected_ = std::move(catalogue);
            } else {
                std::set<std::string> seen;
                for (const auto &uri : uris) {
                    auto entry =
                      std::find_if(catalogue.begin(), catalogue.end(), [&](const auto &e) {
                          return e.uri == uri;
                      });
                    if (!seen.insert(uri).second || entry == catalogue.end()) {
                        throw std::invalid_argument("duplicate or unmapped IRI: " + uri);
                    }
                    selected_.push_back(*entry);
                }
            }
            query_bb_->set("namespace", ns);
            query_bb_->set("override_frame_id", override_frame_id);
            return onRunning();
        } catch (const std::exception &e) {
            return fail(e.what());
        }
    }
    BT::NodeStatus onRunning() override
    {
        try {
            auto now = Clock::now();
            if (now >= deadline_) { return fail("pose collection timed out"); }
            const auto &entity = selected_.at(index_);
            if (!querying_) {
                query_bb_->set("entity", entity.id);
                query_bb_->set("timeout", std::chrono::duration<double>(deadline_ - now).count());
                querying_ = true;
            }
            auto status = query_->executeTick();
            if (status == BT::NodeStatus::FAILURE) {
                return fail(entity.uri + ": " + query_bb_->get<std::string>("message"));
            }
            if (status == BT::NodeStatus::RUNNING) {
                statusMessage("querying " + entity.uri);
                return status;
            }
            auto pose = toTargetFrame(
              query_bb_->get<geometry_msgs::msg::PoseStamped>("pose"),
              target_frame_,
              world_frame_,
              *tf_
            );
            poses_.emplace(entity.uri, pose);
            cleanup();
            if (++index_ == selected_.size()) {
                setOutput("poses", poses_);
                statusMessage("succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        } catch (const std::exception &e) {
            return fail(e.what());
        }
    }
    void onHalted() override
    {
        cleanup();
        statusMessage("halted; pending pose response discarded");
    }
};
} // namespace bdd_bt_executor_ros2
