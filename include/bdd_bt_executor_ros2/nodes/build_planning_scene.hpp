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

#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bdd_bt_executor_ros2/nodes/get_entity_poses.hpp"

namespace bdd_bt_executor_ros2 {
struct SceneEntity
{
    std::string                       uri;
    std::string                       id;
    moveit_msgs::msg::CollisionObject object;
};
inline std::vector<SceneEntity> readSceneEntities(const YAML::Node &root)
{
    auto entries = root["entities"];
    if (!entries.IsSequence() || entries.size() == 0) {
        throw std::invalid_argument("entities must be a nonempty list");
    }
    std::vector<SceneEntity> entities;
    std::set<std::string>    uris, collision_ids;
    for (const auto &entry : entries) {
        SceneEntity entity;
        entity.uri = entry["uri"].as<std::string>();
        entity.id  = entry["id"] ? entry["id"].as<std::string>() : "";
        entity.object.id =
          entry["collision_id"] ? entry["collision_id"].as<std::string>() : entity.uri;
        if (
          entity.uri.empty() || entity.object.id.empty() || !uris.insert(entity.uri).second
          || !collision_ids.insert(entity.object.id).second
        ) {
            throw std::invalid_argument("empty or duplicate entity URI/collision ID");
        }
        const auto shapes = entry["shapes"];
        if (!shapes.IsSequence() || shapes.size() == 0) {
            throw std::invalid_argument("require shapes for " + entity.uri);
        }
        for (const auto &shape : shapes) {
            shape_msgs::msg::SolidPrimitive primitive;
            const auto                      type = shape["type"].as<std::string>();
            size_t                          count;
            if (type == "box") {
                primitive.type = primitive.BOX;
                count          = 3;
            } else if (type == "sphere") {
                primitive.type = primitive.SPHERE;
                count          = 1;
            } else if (type == "cylinder" || type == "cone") {
                primitive.type = type == "cylinder" ? primitive.CYLINDER : primitive.CONE;
                count          = 2;
            } else {
                throw std::invalid_argument("unsupported primitive: " + type);
            }
            const auto dims = shape["dimensions"].as<std::vector<double>>();
            if (dims.size() != count) {
                throw std::invalid_argument("incorrect primitive dimension count");
            }
            for (double v : dims) {
                if (!std::isfinite(v) || v <= 0) {
                    throw std::invalid_argument("dimensions must be finite and positive");
                }
            }
            primitive.dimensions.assign(dims.begin(), dims.end());
            geometry_msgs::msg::Pose pose;
            pose.orientation.w = 1;
            if (shape["position"]) {
                auto v = shape["position"].as<std::vector<double>>();
                if (v.size() != 3) {
                    throw std::invalid_argument("position requires three values");
                }
                pose.position.x = v[0];
                pose.position.y = v[1];
                pose.position.z = v[2];
            }
            if (shape["orientation"]) {
                auto q = shape["orientation"].as<std::vector<double>>();
                if (q.size() != 4) { throw std::invalid_argument("orientation requires x,y,z,w"); }
                pose.orientation.x = q[0];
                pose.orientation.y = q[1];
                pose.orientation.z = q[2];
                pose.orientation.w = q[3];
            }
            validatePose(pose);
            entity.object.primitives.push_back(primitive);
            entity.object.primitive_poses.push_back(pose);
        }
        entity.object.operation = entity.object.ADD;
        entities.push_back(std::move(entity));
    }
    return entities;
}
inline moveit_msgs::msg::CollisionObject
  entityCollisionObject(const SceneEntity &entity, const geometry_msgs::msg::PoseStamped &pose)
{
    validatePose(pose.pose);
    auto object   = entity.object;
    object.header = pose.header;
    object.pose   = pose.pose;
    return object;
}
class BuildPlanningScene : public BT::SyncActionNode
{
  public:
    BuildPlanningScene(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("config_file"),
                 BT::InputPort<EntityPoses>("poses"),
                 BT::OutputPort<moveit_msgs::msg::PlanningScene>("scene"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus tick() override
    {
        try {
            std::string file;
            EntityPoses poses;
            if (
              !getInput("config_file", file) || file.empty() || !getInput("poses", poses)
              || poses.empty()
            ) {
                throw std::invalid_argument("require config_file and nonempty URI-pose map");
            }
            auto                            catalogue = readSceneEntities(YAML::LoadFile(file));
            moveit_msgs::msg::PlanningScene scene;
            scene.is_diff             = true;
            scene.robot_state.is_diff = true;
            for (const auto &[uri, pose] : poses) {
                const auto entry =
                  std::find_if(catalogue.begin(), catalogue.end(), [&](const auto &entity) {
                      return entity.uri == uri;
                  });
                if (entry == catalogue.end() || pose.header.frame_id.empty()) {
                    throw std::invalid_argument("unmapped URI or empty pose frame: " + uri);
                }
                validatePose(pose.pose);
            }
            for (const auto &entity : catalogue) {
                const auto pose = poses.find(entity.uri);
                if (pose != poses.end()) {
                    scene.world.collision_objects.push_back(
                      entityCollisionObject(entity, pose->second)
                    );
                }
            }
            setOutput("scene", scene);
            setOutput("status_message", std::string("succeeded"));
            return BT::NodeStatus::SUCCESS;
        } catch (const std::exception &e) {
            setOutput("status_message", std::string(e.what()));
            config().blackboard->set("behaviour_status", name() + ": " + e.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};
} // namespace bdd_bt_executor_ros2
