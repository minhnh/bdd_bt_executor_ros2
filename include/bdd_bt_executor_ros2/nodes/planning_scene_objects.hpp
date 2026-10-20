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
#include <future>
#include <optional>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include "bdd_bt_executor_ros2/nodes/build_planning_scene.hpp"

namespace bdd_bt_executor_ros2 {
inline void sceneNodeStatus(BT::TreeNode &node, const std::string &message)
{
    node.setOutput("status_message", message);
    const BT::TreeNode &view = node;
    view.config().blackboard->set("behaviour_status", node.name() + ": " + message);
}
inline std::vector<SceneEntity> loadSceneCatalogue(BT::TreeNode &node)
{
    std::string file;
    if (!node.getInput("config_file", file) || file.empty())
        throw std::invalid_argument("require config_file");
    return readSceneEntities(YAML::LoadFile(file));
}
inline moveit_msgs::msg::PlanningScene clearScene(
  const std::vector<SceneEntity>        &catalogue,
  const moveit_msgs::msg::PlanningScene &current
)
{
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff             = true;
    scene.robot_state.is_diff = true;
    std::set<std::string> owned, present;
    for (const auto &entity : catalogue) owned.insert(entity.object.id);
    for (const auto &object : current.world.collision_objects)
        if (owned.count(object.id)) present.insert(object.id);
    for (const auto &object : current.robot_state.attached_collision_objects) {
        if (!owned.count(object.object.id)) continue;
        moveit_msgs::msg::AttachedCollisionObject remove;
        remove.link_name        = object.link_name;
        remove.object.id        = object.object.id;
        remove.object.operation = remove.object.REMOVE;
        scene.robot_state.attached_collision_objects.push_back(remove);
        // Detaching restores the object to the world; remove it there too.
        present.insert(object.object.id);
    }
    for (const auto &id : present) {
        moveit_msgs::msg::CollisionObject remove;
        remove.id        = id;
        remove.operation = remove.REMOVE;
        scene.world.collision_objects.push_back(remove);
    }
    return scene;
}
inline const SceneEntity &
  sceneEntity(const std::vector<SceneEntity> &catalogue, const std::string &uri)
{
    const auto entry = std::find_if(catalogue.begin(), catalogue.end(), [&](const auto &entity) {
        return entity.uri == uri;
    });
    if (uri.empty() || entry == catalogue.end()) throw std::invalid_argument("unmapped object URI");
    return *entry;
}
inline moveit_msgs::msg::AttachedCollisionObject
  attachedObject(const std::string &link, const SceneEntity &entity, int operation)
{
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name        = link;
    object.object.id        = entity.object.id;
    object.object.operation = operation;
    return object;
}
inline moveit_msgs::msg::PlanningScene attachScene(
  const std::vector<SceneEntity> &catalogue,
  const std::string              &uri,
  const std::string              &link,
  const std::vector<std::string> &touch_links
)
{
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff             = true;
    scene.robot_state.is_diff = true;
    auto object =
      attachedObject(link, sceneEntity(catalogue, uri), moveit_msgs::msg::CollisionObject::ADD);
    object.touch_links = touch_links;
    scene.robot_state.attached_collision_objects.push_back(object);
    return scene;
}
inline moveit_msgs::msg::PlanningScene detachScene(
  const std::vector<SceneEntity>        &catalogue,
  const std::string                     &uri,
  const std::string                     &link,
  const geometry_msgs::msg::PoseStamped &pose,
  const std::string                     &planning_frame
)
{
    if (pose.header.frame_id != planning_frame)
        throw std::invalid_argument("scene pose frame mismatch");
    const auto                     &entity = sceneEntity(catalogue, uri);
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff             = true;
    scene.robot_state.is_diff = true;
    scene.robot_state.attached_collision_objects.push_back(
      attachedObject(link, entity, moveit_msgs::msg::CollisionObject::REMOVE)
    );
    scene.world.collision_objects.push_back(
      bdd_bt_executor_ros2::entityCollisionObject(entity, pose)
    );
    return scene;
}
class ClearPlanningSceneForUris : public BT::StatefulActionNode
{
    using Service = moveit_msgs::srv::GetPlanningScene;
    using Clock   = std::chrono::steady_clock;
    std::vector<SceneEntity>                  catalogue_;
    rclcpp::Node::SharedPtr                   node_;
    rclcpp::Client<Service>::SharedPtr        client_;
    std::future<Service::Response::SharedPtr> future_;
    std::optional<int64_t>                    request_id_;
    Clock::time_point                         deadline_;
    void                                      cleanup()
    {
        if (client_ && request_id_) client_->remove_pending_request(*request_id_);
        request_id_.reset();
        future_ = {};
    }
    BT::NodeStatus fail(const std::string &message)
    {
        cleanup();
        sceneNodeStatus(*this, message);
        return BT::NodeStatus::FAILURE;
    }

  public:
    ClearPlanningSceneForUris(
      const std::string      &name,
      const BT::NodeConfig   &cfg,
      rclcpp::Node::SharedPtr node
    )
      : BT::StatefulActionNode(name, cfg), node_(std::move(node))
    {}
    ~ClearPlanningSceneForUris() override { cleanup(); }
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("config_file"),
                 BT::InputPort<std::vector<std::string>>("uris"),
                 BT::InputPort<std::string>(
                   "service_name", "/get_planning_scene", "Cleanup query endpoint"
                 ),
                 BT::InputPort<double>("timeout", 5.0, "Total steady-clock timeout"),
                 BT::OutputPort<moveit_msgs::msg::PlanningScene>("scene"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus onStart() override
    {
        cleanup();
        try {
            std::string endpoint;
            double      timeout;
            const auto  now = Clock::now();
            if (
              !getInput("service_name", endpoint) || endpoint.empty()
              || !getInput("timeout", timeout) || !std::isfinite(timeout) || timeout <= 0
              || timeout >= std::chrono::duration<double>(Clock::time_point::max() - now).count()
            )
                throw std::invalid_argument("require service_name and finite positive timeout");
            deadline_ =
              now
              + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
            catalogue_ = loadSceneCatalogue(*this);
            std::vector<std::string> uris;
            if (config().input_ports.count("uris") && !getInput("uris", uris))
                throw std::invalid_argument("invalid uris input");
            if (!uris.empty()) {
                std::vector<SceneEntity> selected;
                std::set<std::string>    seen;
                for (const auto &uri : uris) {
                    if (!seen.insert(uri).second)
                        throw std::invalid_argument("duplicate URI: " + uri);
                    selected.push_back(sceneEntity(catalogue_, uri));
                }
                catalogue_ = std::move(selected);
            }
            if (!client_ || client_->get_service_name() != endpoint)
                client_ = node_->create_client<Service>(endpoint);
            return onRunning();
        } catch (const std::exception &e) {
            return fail(e.what());
        }
    }
    BT::NodeStatus onRunning() override
    {
        try {
            if (
              future_.valid()
              && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready
            ) {
                auto response = future_.get();
                cleanup();
                setOutput("scene", clearScene(catalogue_, response->scene));
                sceneNodeStatus(*this, "succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            if (Clock::now() >= deadline_) return fail("cleanup scene query timed out");
            if (!request_id_ && client_->service_is_ready()) {
                auto request                   = std::make_shared<Service::Request>();
                request->components.components = request->components.WORLD_OBJECT_NAMES
                                                 | request->components.ROBOT_STATE_ATTACHED_OBJECTS;
                auto pending                   = client_->async_send_request(request);
                request_id_                    = pending.request_id;
                future_                        = std::move(pending.future);
            }
            sceneNodeStatus(*this, "querying current scene for cleanup");
            return BT::NodeStatus::RUNNING;
        } catch (const std::exception &e) {
            return fail(e.what());
        }
    }
    void onHalted() override { cleanup(); }
};
class AttachObjectByUri : public BT::SyncActionNode
{
  public:
    AttachObjectByUri(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("config_file"),
                 BT::InputPort<std::string>("object_uri"),
                 BT::InputPort<std::string>("link_name"),
                 BT::InputPort<std::vector<std::string>>("touch_links"),
                 BT::OutputPort<moveit_msgs::msg::PlanningScene>("scene"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus tick() override
    {
        try {
            std::string uri, link;
            if (
              !getInput("object_uri", uri) || uri.empty() || !getInput("link_name", link)
              || link.empty()
            )
                throw std::invalid_argument("require object_uri and link_name");
            auto                     catalogue = loadSceneCatalogue(*this);
            std::vector<std::string> touches;
            if (
              !getInput("touch_links", touches) || touches.empty()
              || std::any_of(touches.begin(), touches.end(), [](const auto &v) {
                     return v.empty();
                 })
            )
                throw std::invalid_argument("require nonempty touch_links");
            setOutput("scene", attachScene(catalogue, uri, link, touches));
            sceneNodeStatus(*this, "succeeded");
            return BT::NodeStatus::SUCCESS;
        } catch (const std::exception &e) {
            sceneNodeStatus(*this, e.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};
class DetachObjectByUri : public BT::SyncActionNode
{
  public:
    DetachObjectByUri(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("config_file"),
                 BT::InputPort<std::string>("object_uri"),
                 BT::InputPort<std::string>("link_name"),
                 BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose"),
                 BT::InputPort<std::string>("planning_frame"),
                 BT::OutputPort<moveit_msgs::msg::PlanningScene>("scene"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus tick() override
    {
        try {
            std::string uri, link;
            if (
              !getInput("object_uri", uri) || uri.empty() || !getInput("link_name", link)
              || link.empty()
            )
                throw std::invalid_argument("require object_uri and link_name");
            auto                            catalogue = loadSceneCatalogue(*this);
            geometry_msgs::msg::PoseStamped pose;
            std::string                     frame;
            if (
              !getInput("object_pose", pose) || !getInput("planning_frame", frame) || frame.empty()
            )
                throw std::invalid_argument("require object_pose and planning_frame");
            setOutput("scene", detachScene(catalogue, uri, link, pose, frame));
            sceneNodeStatus(*this, "succeeded");
            return BT::NodeStatus::SUCCESS;
        } catch (const std::exception &e) {
            sceneNodeStatus(*this, e.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};
} // namespace bdd_bt_executor_ros2
