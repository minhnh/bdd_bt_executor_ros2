# bdd_bt_executor_ros2

A reusable ROS 2 `bdd_ros2_interfaces/action/Behaviour` server backed by
[BehaviorTree.CPP](https://www.behaviortree.dev/).

The package provides the `bdd_bt_action_server` executable and these tree nodes:

- `PublishBddEvent` publishes the current scenario context on the configured event topic.
- `MockTimedAction` is a timed diagnostic action for integration tests and tree debugging.
- `MoveGroupAction` sends a native MoveIt goal or resolves an SRDF named target.
- `GripperCommandAction` sends a position/effort command to a gripper action server.
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
    1. [BDD event publication](#bdd-event-publication)
    1. [Timed mock actions](#timed-mock-actions)
    1. [MoveIt and gripper actions](#moveit-and-gripper-actions)
    1. [Entity pose queries](#entity-pose-queries)

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
  invoke their respective nodes. SRDF is required only for named-target goals.

Run the server:

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
- `robot_description_semantic` (default empty): SRDF XML required for named targets.

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
- Override `tree_xml`, `srdf`, or `behaviour_action` in the launch command.
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
- Executor checks cover Behaviour feedback, scenario UUIDs, and trinary results.


## BT node reference

### BDD event publication

```xml
<PublishBddEvent iri="https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-pick-start"/>
```

- `iri` is required and must be nonempty; use the exact event IRI from the BDD model.
- Publishes a `bdd_ros2_interfaces/msg/Event` on the executor's `event_topic`.
- Includes the current Behaviour goal's scenario UUID and the ROS clock timestamp.
- Returns `SUCCESS` after publishing; a missing/empty IRI returns `FAILURE`.
- Publication does not wait for coordinator acknowledgment.

### Timed mock actions

```xml
<MockTimedAction duration="0.5" heartbeat_duration="0.1" message="Mock picking"/>
```

- All three inputs are required: `duration` in seconds (nonnegative),
  `heartbeat_duration` in seconds (positive), and `message` as a string.
- Returns `RUNNING` until the steady-clock duration expires, then `SUCCESS`.
  Zero duration succeeds immediately.
- Logs `message` at the heartbeat interval while running; it has no output ports.
- Halt ends the wait without additional work; it sends no robot commands.
- Used by [behavior_trees/mock_pick_place.xml](behavior_trees/mock_pick_place.xml)
  to exercise event ordering and coordinator integration without motion servers.

### MoveIt and gripper actions

```xml
<MoveGroupAction action_name="{move_action_name}"
                 goal="{motion_goal}" feedback_state="{move_state}"
                 error_code="{move_code}" status_message="{move_status}"/>
<GripperCommandAction action_name="{gripper_action_name}"
                      position="0.04" max_effort="5"
                      result_position="{finger_position}"
                      reached_goal="{reached}" stalled="{stalled}"/>
```

Alternatively, resolve a named target from the executor's `robot_description_semantic` parameter:

```xml
<MoveGroupAction group="arm" named_target="ready" timeout="60"/>
```

Goal inputs:

- **Native goal:** `motion_goal` must be a `moveit_msgs::action::MoveGroup::Goal`
  prepared by an application node. It requires a group, goal constraints, positive
  planning time, and finite scaling factors in `[0, 1]` (zero preserves MoveIt defaults).
- **Named target:** specify `group` + `named_target` and supply SRDF XML through
  the executor's `robot_description_semantic` parameter. Other goal forms and
  non-motion nodes do not require SRDF on the executor.
- **Exclusive inputs:** combining `goal` with named-target inputs is rejected.
- **Named-target validation:** unknown/empty targets, duplicate joints, malformed
  XML, and invalid positions fail before sending a goal. Only scalar joint
  positions are supported; parsing uses the classic dot-decimal locale.
- **Named-target execution:** starts from MoveIt's current robot state and executes
  the planned trajectory. Options are `planning_time` (5 seconds),
  `velocity_scaling` (0.2), and `acceleration_scaling` (0.2).
- **MoveGroup success:** requires both ROS action success and a successful MoveIt error code.
- **Gripper success:** reports command completion; reached/stalled outputs do not certify physical grasp.

Endpoints and timing:

- `action_name` defaults to `/move_action` for MoveGroup and `/gripper_cmd` for
  the gripper. Override through application configuration and blackboard entries
  when endpoints differ. Empty endpoints fail.
- `timeout` defaults to 30 seconds; `cancel_timeout` defaults to 2 seconds.
- Timeouts use steady time even with a paused simulation clock. Ticks and halt
  never wait for ROS futures; unavailable servers are polled until the timeout.
- Action feedback updates output ports and Behaviour status on ticks.
- Named and native goals use the same endpoints, timing, feedback, and cancellation handling.

Client lifetime:

- Each BT node creates a client lazily on its first valid invocation and reuses
  it while `action_name` stays the same.
- Changing the name replaces that node's retained client; there is no shared client cache.
- Callback state is separate for each invocation. Pending cancellation retains
  an old client only for the bounded cancellation interval.
- Client retention lasts for the BT node's lifetime. The executor creates a new
  tree for each Behaviour goal; unused motion nodes create no clients.

Cancellation:

- Halt/timeout requests cancellation, including after delayed goal acceptance.
- Keep the ROS executor spinning for `cancel_timeout` after tree destruction to
  deliver late callbacks. After that interval, late callbacks are discarded.
- Cancellation failures and expired confirmation windows are logged as
  **stop unconfirmed**. Behaviour cancellation does not prove the controller stopped.
- Verify controller stoppage/reset before another run.


### Entity pose queries

```xml
<GetEntityPose entity_id="{simulator_entity_id}" service_namespace="/"
               timeout="5" pose="{entity_pose}" status_message="{pose_status}"/>
```

- `entity_id` is the simulator's entity name/ID, such as a prim path. URI mappings
  belong to the application; the node accepts simulator IDs only.
- `service_namespace` defaults to `/`, selecting `/get_entity_state` of type
  `simulation_interfaces/srv/GetEntityState`. An empty namespace uses the node's
  namespace; relative and absolute namespaces are supported.
- `timeout` defaults to 5 seconds and covers discovery and the request together,
  using steady time even if simulation time is paused.
- Discovery and response polling are asynchronous; each invocation submits once.
- `pose` is a native `geometry_msgs::msg::PoseStamped` blackboard value, valid only
  after success. The service's header and pose are copied without modification.
- An empty response frame means world according to the interface; it remains
  empty in the output. Zero/unspecified stamps are preserved, not replaced with
  the current time. The query does not establish observation freshness.
- Empty IDs, invalid namespaces/timeouts, unavailable services, service errors,
  and expired requests fail with `status_message` and Behaviour feedback.
- Halt, timeout, and node destruction remove the local pending request. Service
  execution cannot be canceled; late replies are discarded and cannot affect a
  later invocation. The client is reused while the service endpoint stays the same.
- Fake-service tests cover pose/header preservation, errors, timeouts, and late
  responses after halt/restart or tree destruction; no simulator is needed.
