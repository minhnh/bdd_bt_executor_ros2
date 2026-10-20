// Copyright 2026 Minh Nguyen
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <bdd_ros2_interfaces/action/behaviour.hpp>
#include <behaviortree_cpp/action_node.h>
#include <map>
#include <memory>
#include <stdexcept>
#include <set>
#include <string>
#include <yaml-cpp/yaml.h>

namespace bdd_bt_executor_ros2 {
using UriBindings = std::map<std::string, std::string>;

inline UriBindings resolveBehaviourParameters(
  const bdd_ros2_interfaces::action::Behaviour::Goal &goal,
  const YAML::Node                                   &bindings
)
{
    if (!goal.configs.empty())
        throw std::invalid_argument("Behaviour configurations are not supported");
    if (!bindings.IsMap() || bindings.size() == 0)
        throw std::invalid_argument("variable_bindings must be a nonempty map");
    UriBindings values, result;
    for (const auto &parameter : goal.parameters) {
        if (
          parameter.variable_uri.empty() || parameter.param_val_uris.size() != 1
          || parameter.param_val_uris.front().empty()
          || !values.emplace(parameter.variable_uri, parameter.param_val_uris.front()).second
        )
            throw std::invalid_argument("require one nonempty URI per unique variable URI");
    }
    std::set<std::string> variables;
    for (const auto &binding : bindings) {
        const auto alias    = binding.first.as<std::string>();
        const auto variable = binding.second.as<std::string>();
        if (
          alias.empty() || alias == "behaviour_goal" || alias == "behaviour_status"
          || variable.empty() || !variables.insert(variable).second
        )
            throw std::invalid_argument("invalid or duplicate variable binding");
        if (!values.count(variable))
            throw std::invalid_argument("missing Behaviour parameter: " + variable);
        if (!result.emplace(alias, values.at(variable)).second)
            throw std::invalid_argument("duplicate binding alias: " + alias);
    }
    return result;
}

class ResolveBehaviourParameters : public BT::SyncActionNode
{
  public:
    using BT::SyncActionNode::SyncActionNode;
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("config_file"),
                 BT::OutputPort<UriBindings>("uri_bindings"),
                 BT::OutputPort<std::string>("status_message") };
    }
    BT::NodeStatus tick() override
    {
        try {
            std::string file;
            auto        input = getInput("config_file", file);
            if (!input || file.empty()) throw std::invalid_argument("missing config_file");
            auto goal =
              config()
                .blackboard
                ->get<std::shared_ptr<const bdd_ros2_interfaces::action::Behaviour::Goal>>(
                  "behaviour_goal"
                );
            if (!goal) throw std::invalid_argument("missing Behaviour goal");
            auto values =
              resolveBehaviourParameters(*goal, YAML::LoadFile(file)["variable_bindings"]);
            setOutput("uri_bindings", values);
            for (const auto &[alias, uri] : values) config().blackboard->set(alias, uri);
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
