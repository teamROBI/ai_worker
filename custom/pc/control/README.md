# Robot Control Scripts

Standalone scripts for controlling the FFW robot: joint presets, keyboard teleop, and init pose from LeRobot training data.

## Setup

All scripts require ROS2 packages (`rclpy`, `sensor_msgs`, `trajectory_msgs`, `cv_bridge`) from the sourced ROS2 workspace.

`init_pose_from_data.py` additionally needs `pyarrow`, `scikit-image`, `lerobot`, and `opencv-python`. These are managed with [uv](https://docs.astral.sh/uv/):

```bash
cd custom/pc/control
uv sync
```

## Scripts

### `keyboard_control_full_robot.py`

All-in-one GUI controller with three tabs: **Joints** (sliders + presets), **Cartesian** (MoveIt FK/IK jog), **Base** (swerve keyboard drive).

```bash
./run_keyboard_control.sh
```

**Do not run it directly with `python3` from the host shell** — since ai_worker `2.0.0`
the robot's RMW is `rmw_zenoh_cpp`, and the host's ROS 2 Humble only has FastDDS, so
the ROS graph comes up empty (no `/joint_states`, no `/cmd_vel`, nothing moves and every
arm send is refused by the freshness gate). `run_keyboard_control.sh` runs the script
inside the jazzy `ai_worker` container as a zenoh **client of the router on the robot**.

Connecting as a zenoh *peer* is not enough: the robot's nodes listen only on their own
loopback, so a peer has no route to them. A client routes everything through the robot's
router, which does. See [Zenoh networking](#zenoh-networking) below.

Already inside the container (e.g. after the `DAW` alias)? The same script runs there
directly — but the container has no mDNS, so give it the robot's IP:

```bash
cd /root/ros2_ws/src/ai_worker/custom/pc/control
ROBOT_IP=192.168.6.2 ./run_keyboard_control.sh
```

No rebuild or `colcon build` is ever needed for these scripts: the repo is bind-mounted
at `/root/ros2_ws/src/ai_worker`, and `custom/pc/` is not a colcon package, so edits
on the host take effect on the next run.

- Arrow keys / A / D for base driving, Shift for boost
- Sliders for joint control, Spacebar to send all
- Cartesian tab requires MoveIt `move_group` running
- Live "Actual" column shows real-time joint state from `/joint_states`

### `apply_joint_preset.py`

Move the robot to a saved joint preset (from `preset_joint_positions/`).

```bash
./run_keyboard_control.sh apply_joint_preset.py <preset_name>
# e.g.
./run_keyboard_control.sh apply_joint_preset.py action_ready
```

(Same reason as above: it needs the zenoh client env, so go through the launcher rather
than `python3` on the host.)

### `init_pose_from_data.py`

Read a LeRobot v2 dataset, move the robot to an episode's initial pose, and visually compare the live head camera against the dataset's reference frame. Requires the uv venv.

```bash
# Random episode init pose (default)
uv run python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot

# Average of all episodes
uv run python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --mode average

# Specific episode
uv run python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --episode 5

# Joints only, skip visual comparison
uv run python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --no-visual
```

The dataset name is resolved under `<repo>/lerobot/ACS_ROBI/<name>` by default.
The dataset directory or symlink is device-local and is not included in this fork.
Use `--data-root /data/datasets/lerobot/ACS_ROBI` or set `LEROBOT_DATA_ROOT` to
choose another location; an absolute dataset path also works. When launching via
Docker, `--data-root` must refer to a location mounted inside the container.
The keyboard launcher also forwards `LEROBOT_DATA_ROOT` from the host; its value
must likewise use the container's dataset path.
If the name is wrong it will list available datasets.

Visual comparison window shows **Reference | Live | Diff heatmap** with SSIM score. Press `s` to save snapshot, `q` to quit.

### `cartesian_servo_teleop.py`

Helper module (not run directly). Provides Cartesian FK/IK jogging via MoveIt services. Imported by `keyboard_control_full_robot.py`.

## Directory

```
control/
  keyboard_control_full_robot.py   # GUI teleop (joints + cartesian + base)
  apply_joint_preset.py            # Apply saved joint preset
  init_pose_from_data.py           # Init pose from LeRobot v2 data + visual compare
  cartesian_servo_teleop.py        # Cartesian teleop helper (imported, not run)
  preset_joint_positions/          # Saved joint preset JSON files
  pyproject.toml                   # uv project config (for init_pose_from_data.py)
```

## Zenoh networking

> This affects **everything**, not just these scripts. Most importantly the **leader**:
> if `ffw_lg2_leader_ai` runs on the PC while the follower runs on the robot, they land
> on two different zenoh routers and teleoperation silently does nothing — the leader
> publishes `/leader/.../joint_trajectory` into the PC's router while the follower waits
> on the robot's. Nothing errors; the arms just never move. Source
> `custom/pc/network/zenoh_robot_env.sh` in the shell **before** launching the leader:
>
> ```bash
> DAW
> ROBOT_IP=192.168.6.2 source /root/ros2_ws/src/ai_worker/custom/pc/network/zenoh_robot_env.sh
> ffw_lg2_leader_ai
> ```
>
> To check: from a shell that sourced it, `ros2 topic info -v \
> /leader/joint_trajectory_command_broadcaster_left/joint_trajectory` should show
> **both** a publisher (leader) and a subscriber (follower). One without the other
> means the two ends are still on separate routers.


Since ai_worker `2.0.0` the RMW is `rmw_zenoh_cpp`. Each host runs a router (`zenohd`,
i.e. `ros2 run rmw_zenoh_cpp rmw_zenohd`) on port `7447`, and routers on different hosts
are **not** linked automatically — so the PC's local `zenoh_daemon` container never sees
the robot's topics.

`run_keyboard_control.sh` sidesteps that by making the node a zenoh client of the robot's
router, which needs no change to either router:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ROS_DOMAIN_ID=30
ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<ROBOT_IP>:7447"]'
```

Robot side (over `ssh ai_worker`, inside its container), in this order:

```bash
zenohd                                            # must be up BEFORE the bringup
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

Debugging, from the PC:

```bash
# is the robot's router reachable?
nc -z <ROBOT_IP> 7447 && echo open

# does the graph arrive? (empty output = only /rosout, /parameter_events)
ROBOT_IP=<ROBOT_IP> docker exec \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp -e ROS_DOMAIN_ID=30 \
  -e ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$ROBOT_IP:7447\"]" \
  ai_worker bash -c 'source /opt/ros/jazzy/setup.bash; ros2 topic list --no-daemon'
```

`ros2 topic list` without `--no-daemon` can report a stale graph from a daemon started
under a different RMW — `ros2 daemon stop` first, or always pass `--no-daemon`.

The **Cartesian tab stays disabled unless MoveIt `move_group` is running**; the follower
bringup alone does not start it.
