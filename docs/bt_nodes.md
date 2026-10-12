# BT node reference

[Package overview](../README.md)

Use `{key}` to read/write native blackboard values. Scene and pose outputs are
valid after `SUCCESS`. Timeouts use steady time; keep ROS callbacks spinning.

## Contents

<!-- mtoc-start -->

1. [Entity catalogue](#entity-catalogue)
1. [PublishBddEvent](#publishbddevent)
1. [MockTimedAction](#mocktimedaction)
1. [Simulation interfaces](#simulation-interfaces)
    1. [GetEntityPose](#getentitypose)
    1. [GetEntityPoses](#getentityposes)
1. [MoveIt Motion actions](#moveit-motion-actions)
    1. [MoveGroupAction](#movegroupaction)
    1. [GripperCommandAction](#grippercommandaction)
1. [MoveIt planning scenes](#moveit-planning-scenes)
    1. [BuildPlanningScene](#buildplanningscene)
    1. [ApplyPlanningScene](#applyplanningscene)
    1. [ClearPlanningSceneForUris](#clearplanningsceneforuris)
    1. [AttachObjectByUri](#attachobjectbyuri)
    1. [DetachObjectByUri](#detachobjectbyuri)

<!-- mtoc-end -->

## Entity catalogue

`GetEntityPoses` and the MoveIt scene construction nodes share `config_file`.

Use the shipped [config/example_entities.yaml](../config/example_entities.yaml)
for the catalogue schema. It is installed under the package share directory:

```sh
ENTITY_CONFIG="$(ros2 pkg prefix --share bdd_bt_executor_ros2)/config/example_entities.yaml"
```

Replace its placeholder simulator IDs and geometry for your scene. For a runnable
integration, see [robbdd_tutorials](https://github.com/minhnh/robbdd_tutorials):
`config/isaac_pick_place.yaml` and `launch/isaac_prepare.launch.yaml`.

- `uri` identifies the object in both node groups.
- `id` is required by `GetEntityPoses`; MoveIt construction nodes do not require it.
- `collision_id` defaults to `uri`. `shapes` are required by MoveIt construction
  nodes and ignored by pose collection.
- Shapes: `box` `[x,y,z]`, `sphere` `[radius]`, `cylinder`/`cone` `[height,radius]`;
  finite positive dimensions in metres. Multiple shapes are allowed.
- Local `position` defaults to zero; `orientation` is a unit quaternion `[x,y,z,w]`,
  default identity. Both are relative to the entity pose.

## PublishBddEvent

Publishes a BDD event on the executor's event topic, with the current scenario
UUID and timestamp.

```xml
<PublishBddEvent iri="{event_iri}"/>
```

- Required: nonempty `iri` from the BDD model.

## MockTimedAction

Runs a timed logging action for testing a tree without behaviour integration.

```xml
<MockTimedAction duration="0.5" heartbeat_duration="0.1" message="Mock picking"/>
```

- Required: `duration` >= 0, `heartbeat_duration` > 0 (seconds), and `message`.
- Logs at the heartbeat interval; succeeds after the duration. No output ports.

## Simulation interfaces

Both nodes query `simulation_interfaces/srv/GetEntityState`:

- Defaults: `service_namespace="/"`, `timeout="5"`, `override_frame_id=""`.
- A nonempty override replaces the response frame label; it does not transform coordinates.

### GetEntityPose

Fetches the ground-truth pose of one simulated entity.

```xml
<GetEntityPose entity_id="{entity_id}" pose="{pose}"/>
```

- Required: `entity_id` (simulator ID, such as a prim path).
- Output: `pose` (`geometry_msgs::msg::PoseStamped`), preserving the response timestamp.
- Optional output: `status_message`.

### GetEntityPoses

Collects poses for selected object URIs into a reusable blackboard map,
transforming them into the requested target frame.

```xml
<GetEntityPoses config_file="{config}" target_frame="world"
    uris="{uris}" poses="{poses}"/>
```

- Required: `config_file` with `entities` containing `uri`/`id`, and `target_frame`.
- Optional: native string vector `uris` (omitted/empty = all), `world_frame="world"`
  for empty response frames, and the shared simulation options above.
- Output: `poses` (`bdd_bt_executor_ros2::EntityPoses`, URI -> PoseStamped map);
  optional `status_message`. Queries sequentially and converts through TF; missing TF fails.

## MoveIt Motion actions

These nodes send ROS action goals for motion or gripper control.

- Defaults: `timeout="30"` and `cancel_timeout="2"` (seconds).
- Halt/timeout requests cancellation; physical stoppage is not guaranteed.

### MoveGroupAction

Sends a motion goal to MoveIt using either a native goal or a named robot
configuration from the SRDF.

```xml
<MoveGroupAction goal="{goal}" error_code="{code}" feedback_state="{state}"/>
<MoveGroupAction group="arm" named_target="ready"/>
```

- Required: native `moveit_msgs::action::MoveGroup::Goal` `goal`, OR
  `group` + `named_target`. Named targets require the executor's SRDF parameter
  `robot_description_semantic`; the two input forms cannot be combined.
- Optional: `action_name="/move_action"` and shared timing options.
- Named-target options: `planning_time="5"`, `velocity_scaling="0.2"`,
  `acceleration_scaling="0.2"`.
- Outputs: `error_code`, `feedback_state`, `status_message`.
  Success requires ROS action success and a successful MoveIt error code.

### GripperCommandAction

Commands a gripper position through a ROS GripperCommand action server.

```xml
<GripperCommandAction position="0.04" max_effort="5"
    result_position="{position}" reached_goal="{reached}" stalled="{stalled}"/>
```

- Required: `position` (metres).
- Optional: `max_effort="0"`, `action_name="/gripper_cmd"`, and shared timing options.
- Outputs: `result_position`, `reached_goal`, `stalled`, `status_message`.
  Completion does not certify a physical grasp.

## MoveIt planning scenes

These nodes construct or apply MoveIt collision-scene updates.

- All construction nodes output native `moveit_msgs::msg::PlanningScene` diffs and
  optional `status_message`. Apply each diff with `ApplyPlanningScene`.
- Attachment/detachment updates collision planning only, not the physical simulator.

### BuildPlanningScene

Combines catalogue geometry and observed poses into a world-collision scene diff.

```xml
<BuildPlanningScene config_file="{config}" poses="{poses}" scene="{scene}"/>
```

- Required: `config_file` and nonempty native URI -> PoseStamped map `poses`.
- Adds catalogue geometry for the map's keys. Preserves each pose's frame and timestamp;
  it does not perform TF conversion or remove omitted objects.

### ApplyPlanningScene

Submits a planning-scene message to MoveIt and waits for acknowledgement.

```xml
<ApplyPlanningScene scene="{scene}" service_name="/apply_planning_scene" timeout="5"/>
```

- Required: native `scene`; forwarded unchanged. Set `is_diff=true` for incremental updates.
- Defaults: `service_name="/apply_planning_scene"`, `timeout="5"`.
- Output: optional `status_message`. Succeeds only when MoveIt accepts the update.

### ClearPlanningSceneForUris

Queries MoveIt and prepares a cleanup diff for selected catalogue URIs.
Objects outside the selection are preserved.

```xml
<ClearPlanningSceneForUris config_file="{config}" uris="{uris}" scene="{scene}"/>
```

- Required: `config_file`. Optional native string vector `uris` (omitted/empty = all).
- Defaults: `service_name="/get_planning_scene"`, `timeout="5"`.
- Queries MoveIt and builds removals for existing selected world/attached objects,
  preserving unrelated objects. The query and subsequent application are not atomic.

### AttachObjectByUri

Prepares an attachment diff so MoveIt plans with the object carried by the
specified robot link.

```xml
<AttachObjectByUri config_file="{config}" object_uri="{uri}"
    link_name="{link}" touch_links="{touch_links}" scene="{scene}"/>
```

- Required: `config_file`, mapped `object_uri`, nonempty `link_name`, and native
  nonempty string vector `touch_links`.
- Builds an attachment ADD using the collision ID. Its geometry must already exist
  in MoveIt's world scene when applied.

### DetachObjectByUri

Prepares a diff that detaches an object and returns its collision geometry to
the world at the supplied pose.

```xml
<DetachObjectByUri config_file="{config}" object_uri="{uri}" link_name="{link}"
    object_pose="{pose}" planning_frame="world" scene="{scene}"/>
```

- Required: `config_file`, mapped `object_uri`, nonempty `link_name`, native
  PoseStamped `object_pose`, and `planning_frame` matching the pose's frame.
- Builds attachment REMOVE and world ADD at that pose using catalogue geometry.
  No TF conversion is performed.
