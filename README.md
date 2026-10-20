# bdd_bt_executor_ros2

A reusable ROS 2 `bdd_ros2_interfaces/action/Behaviour` server backed by
[BehaviorTree.CPP](https://www.behaviortree.dev/).

The package provides the `bdd_bt_action_server` executable and these tree nodes:

- `PublishBddEvent` publishes the current scenario context on the configured event topic.
- `MockTimedAction` is a timed diagnostic action for integration tests and tree debugging.
- `WaitForMoveItReady` waits for the MoveGroup action servers to be ready. This is needed to
  handle [clock reset issue](https://github.com/ros-controls/ros2_control/issues/3697)
  when loading scene variations into simulation.
- `MoveGroupAction` sends native goals, SRDF named targets, or Cartesian pose targets to MoveIt.
- `GripperCommandAction` sends a position/effort command to a gripper action server.
- `ApplyPlanningScene` forwards a `moveit_msgs/PlanningScene` message to MoveIt and waits for acknowledgement.
- `ClearPlanningSceneForUris`, `AttachObjectByUri`, and `DetachObjectByUri` construct
  URI-based cleanup, attachment, and detachment scene diffs.
- `ResolveBehaviourParameters` maps incoming URI-valued goal parameters to configured tree roles.
- `GetEntityPoses` collects URI-to-pose mappings; `BuildPlanningScene` builds primitive collision geometry.
- `SelectEntityPose` selects a pose by URI; `OffsetPose` applies translation and rotation offsets.
- `GetEntityPose` queries an entity pose through
  [ROS simulation interfaces](https://github.com/ros-simulation/simulation_interfaces).

## Contents

<!-- mtoc-start -->

1. [Behaviour action server](#behaviour-action-server)
    1. [Extend `BddBtExecutor` with custom nodes](#extend-bddbtexecutor-with-custom-nodes)
1. [Examples and tests](#examples-and-tests)
    1. [BDD coordinator example](#bdd-coordinator-example)
    1. [Isaac Sim named-goals smoke test](#isaac-sim-named-goals-smoke-test)
    1. [Automated node tests](#automated-node-tests)
1. [BT node reference](#bt-node-reference)

<!-- mtoc-end -->

## Behaviour action server

`BddBtExecutor` connects a ROS Behaviour action goal to an XML behavior tree.
The `bdd_bt_action_server` executable runs this class with the built-in nodes;
applications can also embed the exported C++ class and register custom nodes.

- **Trigger:** a Behaviour goal starts execution; launching the server alone does
  not tick the tree. Goals may come from a BDD coordinator or any ROS action client.
- **Execution:** each goal gets a new tree and blackboard. The executor runs one
  tree at a time and ticks it on a worker thread while ROS callbacks keep spinning.
- **Context:** the immutable goal exposes scenario parameters/configurations to
  application nodes. Feedback, results, and published BDD events preserve the
  goal's scenario context UUID.
- **Feedback:** each tick reports node-supplied `behaviour_status`, or the tree
  status when no node supplies a message.
- **Results:** tree success returns ROS `SUCCEEDED` with trinary `TRUE`; tree
  failure or an execution exception returns `ABORTED` with `FALSE`.
- **Cancellation:** halts the tree and returns `CANCELED` with `UNKNOWN`. Child
  actions request cancellation; physical motion stoppage is not guaranteed by this result.
- **Events:** trees explicitly publish BDD boundaries with `PublishBddEvent`;
  the executor does not infer boundary events from motion completion.
- **Dependencies:** MoveIt and simulation servers are needed only by trees that
  invoke their respective nodes. URDF and SRDF are required for
  named-target goals and readiness checks.

Run the server:

```sh
ros2 run bdd_bt_executor_ros2 bdd_bt_action_server --ros-args \
  -p tree_xml:=/absolute/path/to/tree.xml \
  -p bhv_server_name:=bhv_server \
  -p event_topic:=/bdd/events
```

Parameters:

- `tree_xml` (required): behavior-tree XML file loaded for each accepted goal.
- `tree_inputs` (default `"{}"`): YAML map of scalar values copied to each new
  root blackboard as strings. BT ports convert numbers and semicolon-separated
  vectors to their native types. `behaviour_goal` and `behaviour_status` are reserved.
- `tick_rate_hz` (default `30`): tree tick frequency.
- `bhv_server_name` (default `bhv_server`): ROS action name.
- `event_topic` (default `/bdd/events`): BDD event topic.
- `robot_description` (default empty): URDF XML for named targets and readiness.
- `robot_description_semantic` (default empty): SRDF XML for named targets and readiness.

### Extend `BddBtExecutor` with custom nodes

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

- `behaviour_goal` on the root blackboard is an immutable
  `std::shared_ptr<const bdd_ros2_interfaces::action::Behaviour::Goal>`.
- Application nodes can resolve its parameters/configurations and write native motion goals.
- Nodes can set `behaviour_status` during ticks to supply Behaviour feedback.
  The executor clears it before each tick and falls back to tree status when no node writes a message.

## Examples and tests

### BDD coordinator example

Launch the executor and the `bdd_exec_ros2` coordinator together:

```sh
ros2 launch bdd_bt_executor_ros2 launch_mockup.yaml
ros2 topic pub /bdd/start std_msgs/msg/Empty "{}" -1
```

The default BDD graph is the local RobBDD model set. Override `graph_models` to use another model set.

### Isaac Sim named-goals smoke test

This optional example moves Panda to `extended`, closes the gripper, returns
to `ready`, then opens the gripper.
The executor's motion nodes work with other MoveIt setups as well.

Setup:

- Follow [NVIDIA's Isaac Sim MoveIt tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_moveit.html)
  to prepare the Panda simulation and `isaac_moveit` workspace.
- A workspace setup script is also available in
  [robbdd_tutorials](https://github.com/minhnh/robbdd_tutorials), using its `isaacsim-bt` context.
- Source the ROS, Isaac MoveIt, and executor workspace environments, or the
  generated tutorial environment, in both terminals.
- Start Isaac Sim with the Panda scene playing.

With MoveIt already running:

```sh
ros2 launch bdd_bt_executor_ros2 isaac_named_goals.launch.yaml
```

To start MoveIt and the Behaviour server together:

```sh
ros2 launch bdd_bt_executor_ros2 isaac_named_goals.launch.yaml start_moveit:=true
```

- `start_moveit:=true` includes NVIDIA's launch, including RViz and controllers.
- Use the default `start_moveit:=false` when MoveIt is already running.

> [!WARNING]
> You may have to prepend above command with `LC_NUMERIC=en_US.UTF-8` per
> [moveit/moveit2#1782](https://github.com/moveit/moveit2/issues/1782),
> if you want to use MoveIt via the RViz planning interface.

In another terminal, trigger one round trip:

```sh
ros2 action send_goal /bdd/named_goals bdd_ros2_interfaces/action/Behaviour '{}' --feedback
```

Execution and configuration:

- Launching alone does not move the arm; a Behaviour goal triggers the sequence.
- Success returns action status `SUCCEEDED` and `result.trinary.value: 1`.
- Named targets come from the installed Panda SRDF.
- Each move starts from the current state, with 20% velocity/acceleration scaling.
- [behavior_trees/named_goals.xml](behavior_trees/named_goals.xml) specifies the
  sequence, planning group, targets, action endpoint, and 60-second timeout per move.
- Override `tree_xml`, `urdf`, `srdf`, or `behaviour_action` in the launch command.
- Each timeout covers discovery, planning, and execution using steady time.
- Gripper commands use `/panda_hand_controller/gripper_cmd`: `0.0` closes
  each finger and `0.04` opens each finger (meters).
- Failure stops the sequence; each step runs only after the preceding step succeeds.
- The Isaac gripper bridge reports completion after a fixed delay; verify physical
  finger movement in the simulator.

### Automated node tests

Run without MoveIt or a simulator:

```sh
colcon build --packages-up-to bdd_bt_executor_ros2
colcon test --packages-select bdd_bt_executor_ros2
colcon test-result --verbose
```

- Fake action servers exercise goals, feedback, results, timeouts, cancellation,
  client reuse, and SRDF named-target validation.
- Fake entity-state services check pose/header preservation, errors, timeouts,
  and late responses after halt/restart or tree destruction.
- Fake planning-scene services check native-message forwarding, acknowledgement,
  endpoint changes, timeouts, and late replies after halt/restart or destruction.
- Executor checks cover Behaviour feedback, scenario UUIDs, and trinary results.

## BT node reference

See [docs/bt_nodes.md](docs/bt_nodes.md) for node ports, examples, timing,
client lifetime, and cancellation behavior.
