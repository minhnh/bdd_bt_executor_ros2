# bdd_bt_executor_ros2

A reusable ROS 2 `bdd_ros2_interfaces/action/Behaviour` server backed by
[BehaviorTree.CPP](https://www.behaviortree.dev/).

The package provides the `bdd_bt_action_server` executable and two generic tree nodes:

- `PublishBddEvent` publishes the current scenario context on the configured event topic.
- `MockTimedAction` is a timed diagnostic action for integration tests and tree debugging.

## Run

```sh
ros2 run bdd_bt_executor_ros2 bdd_bt_action_server --ros-args \
  -p tree_xml:=/absolute/path/to/tree.xml \
  -p bhv_server_name:=bhv_server \
  -p event_topic:=/bdd/events
```

Parameters:

- `tree_xml` (required): behavior-tree XML file loaded for each accepted goal.
- `tick_rate_hz` (default `30`): tree tick frequency.
- `bhv_server_name` (default `bhv_server`): ROS action name.
- `event_topic` (default `/bdd/events`): BDD event topic.

## Coordinator smoke test

Launch the executor and the `bdd_exec_ros2` coordinator together:

```sh
ros2 launch bdd_bt_executor_ros2 launch_mockup.yaml
ros2 topic pub /bdd/start std_msgs/msg/Empty "{}" -1
```

The default BDD graph is the local RobBDD model set. Override `graph_models` to use another model set.

## Add application-specific tree nodes

Link to the exported `bdd_bt_executor_ros2` library, construct `BddBtExecutor`, and set its
node-registration callback before spinning it:

```cpp
auto executor = std::make_shared<bdd_bt_executor_ros2::BddBtExecutor>();
executor->set_node_registrar(
    [](BT::BehaviorTreeFactory& factory, const rclcpp::Node::SharedPtr& node) {
      // Register application-specific BehaviorTree.CPP nodes here.
    });
rclcpp::spin(executor);
```

This keeps robot-specific dependencies such as MoveIt out of this package.
