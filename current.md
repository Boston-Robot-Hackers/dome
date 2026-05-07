# dome — Current Launch Handoff

Use this file as the first context document when working on Dome launch
organization. It is not a replacement for `README.md`; it explains how the
current launch files relate and proposes a cleaner structure.

## Snapshot

**Branch:** `main`

`dome` is the robot-base package. It owns the physical platform launch and local
navigation configuration. The perception, behavior, speech, and voice code live
in sibling packages:

- `oak_roboflow_ros` — OAK camera, detections, semantic map, `/describe_scene`
- `control` — behavior manager, `/intent` handling, `/announcement`, speech output
- `robot_voice` — wake word, STT, voice-to-intent publishing

## Current Working Launch Relationship

Full robot operation currently needs the base stack plus the robot brain stack:

```bash
bl dome mini_bringup_bl.launch.py
bl dome robot_brain.launch.py
```

`dome/launch/robot_brain.launch.py` is an aggregate launch file added for this
session. It includes:

```text
oak_roboflow_ros/launch/robot.launch.py
control/launch/robot.launch.py
robot_voice/launch/robot.launch.py
```

That gives the full semantic behavior path:

```text
voice_input or CLI
  -> /intent
  -> behavior_manager
  -> /describe_scene
  -> semantic_map
  -> /announcement
  -> speech_output
```

The base stack and brain stack are intentionally separate right now:

- Base stack: robot body, motors, lidar, robot_state_publisher, TF, Foxglove, base sensors
- Brain stack: OAK perception, semantic map, behavior manager, speech, voice input

This split is useful during debugging because the robot can be brought up without
camera/voice, and the brain stack can be restarted without disturbing the base.

## Important Existing Launch Files

- `mini_bringup_bl.launch.py` — includes `linorobot2_bringup/bringup.launch.py`
  and Foxglove. This is the closest thing to the current robot base launch.
- `robot_brain.launch.py` — includes the three sibling robot brain launches.
- `dome_navigation_bl.launch.py` — Nav2 bringup using `exp4` map and merged
  Nav2 params.
- `dome_slam_bl.launch.py` — slam_toolbox launch with local param patch.
- `navigation_bl.launch.py` and `slam_bl.launch.py` — older direct wrappers
  around `linorobot2_navigation`.
- `copied_navigation.launch.py`, `copied_slam.launch.py` — copied upstream files;
  treat as references unless there is a specific reason to run them.
- `oak_tf.launch.py`, `imu_apply.launch.py`, `depthai_example_stereo.launch.py` —
  focused experiments/utilities, not main robot entry points.

## Current Semantic Map Status

The semantic map update path was verified after fixing `semantic_map_node`.
Targets appear on the 3D map and annotated image. Direct service call works:

```bash
ros2 service call /describe_scene std_srvs/srv/Trigger {}
```

The spoken `scene describe` path requires `control/launch/robot.launch.py`,
because that starts `behavior_manager` and `speech_output`. Without those nodes,
`/describe_scene` can work while speech stays silent.

## Problems With Current Organization

- There is no single obvious top-level launch name for "run the robot."
- `robot.launch.py` means different things in different packages:
  - in `dome`, it does not currently exist
  - in `control`, it launches behavior/speech
  - in `robot_voice`, it launches voice input
  - in `oak_roboflow_ros`, it launches camera/semantic map
- Some launch files are current operational files, while others are experiments,
  copied references, or older wrappers.
- `dome_navigation_bl.launch.py` and `dome_slam_bl.launch.py` duplicate helper
  logic for YAML override merging.
- Launch names mix function, package, and implementation details (`*_bl`,
  `copied_*`, `mini_*`) instead of user-facing robot modes.

## Proposed Better Organization

Keep package ownership, but make `dome` the operator-facing composition layer.

Recommended `dome/launch` entry points:

```text
robot_base.launch.py       # physical base only
robot_brain.launch.py      # perception + behavior + voice only
robot.launch.py            # base + brain, normal full robot
navigation.launch.py       # base + Nav2 localization/navigation
slam.launch.py             # base + SLAM
diagnostics.launch.py      # Foxglove and optional diagnostics
```

Recommended support layout:

```text
dome/launch/include/       # copied/reference/low-level includes if needed
dome/launch/experiments/   # oak_tf, depthai examples, one-off launch tests
dome/dome/launch_utils.py  # shared yaml_override and path helpers
```

Each sibling package should keep a package-local `robot.launch.py` for the nodes
it owns. `dome` should then compose those package-local launches into whole-robot
modes.

## Suggested Migration Steps

1. Rename `mini_bringup_bl.launch.py` to `robot_base.launch.py`.
2. Add `dome/launch/robot.launch.py` that includes `robot_base.launch.py` and
   `robot_brain.launch.py`.
3. Rename `dome_navigation_bl.launch.py` to `navigation.launch.py`.
4. Rename `dome_slam_bl.launch.py` to `slam.launch.py`.
5. Move copied and experimental launch files out of the top-level launch list.
6. Extract repeated `yaml_override()` into a shared helper module.
7. Update `README.md` with the small set of supported launch commands.

## Target Operator Commands

After cleanup, the main commands should be:

```bash
# Full robot: base + OAK + semantic map + behavior + speech + voice
bl dome robot.launch.py

# Base only
bl dome robot_base.launch.py

# Brain only, useful after base is already running
bl dome robot_brain.launch.py

# Navigation mode
bl dome navigation.launch.py

# Mapping mode
bl dome slam.launch.py
```

Until that cleanup is done, use:

```bash
bl dome mini_bringup_bl.launch.py
bl dome robot_brain.launch.py
```
