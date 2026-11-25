# FFW Teleop VR — SG2 Robot Control with Meta Quest 3

VR teleoperation package for controlling the SG2 robot using a Meta Quest 3 headset with motion controllers. Built on [Vuer](https://github.com/vuer-ai/vuer) (WebXR) and ROS2.

Supports two network scenarios — same nodes, different machines:
- **Scenario 1 (Direct):** Quest 3 connects to the robot's WiFi AP. All nodes run on the robot.
- **Scenario 2 (Relay):** Quest 3 connects via an external router through a server PC. Publisher runs on the server, consumer + IK run on the robot.

---

## Setup (Both Scenarios)

### 0. Install Dependencies (inside Docker container)

The default Docker image does not include `ffw_teleop_vr` or its Python dependencies. Install them first:

```bash
# Install Python dependencies
pip install vuer nest_asyncio

# IMPORTANT: vuer's dependency (params-proto) installs a PyPI 'argparse' package
# that shadows Python 3.12's built-in argparse and breaks ROS2 controller spawning.
# You MUST uninstall it:
pip uninstall argparse -y

# scipy may be pre-installed via apt (debian), which blocks pip upgrade.
# Use --ignore-installed to force the pip version:
pip install --ignore-installed scipy

# For IK solver (arm control via PyRoKi):
# IMPORTANT: Use editable install (-e) to avoid breaking ROS2 CLI dependencies.
# Do NOT use `pip install git+...` (non-editable) — it upgrades packages that break ros2cli.
pip install -e git+https://github.com/chungmin99/pyroki.git#egg=pyroki
echo 'export PYTHONPATH=/opt/venv/src/pyroki/src:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc

# Build the package
cd ~/ros2_ws
colcon build --packages-select ffw_teleop_vr
# colcon build --packages-up-to ffw_teleop_vr # If the upper command fails.
source install/setup.bash
```

### 1. SSL Certificates

The Quest 3 browser requires HTTPS for WebXR. Pre-generated self-signed certs are included in `custom/ffw_teleop_vr/cert/`. To regenerate from the repository root:

```bash
cd custom/ffw_teleop_vr/cert
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=localhost'
```

The certs are automatically installed to the package share directory during `colcon build`. All launch files find them automatically — no manual path needed. You can override with `cert_file:=` and `key_file:=` if needed.

---

## Scenario 1 — Direct Connection

Quest 3 connects directly to the robot's WiFi access point. All nodes run on the robot.

```
                    Robot AP WiFi
Quest 3  ──────────────────────────>  Robot PC
                                      vr_raw_sg2_publisher
                                      (Vuer server → raw topics)
                                            |
                                     /vr/raw/* topics
                                            |
                                            v
                                      vr_raw_sg2_consumer
                                      (raw topics → goal poses)
                                            |
                                     /l_goal_pose, /r_goal_pose
                                            |
                                            v
                                      bimanual_ik_solver (PyRoKi)
                                            |
                                      joint trajectories
                                            |
                                            v
                                      Robot Arm Controllers
```

### How to Run

```bash
# Terminal 1 — Robot bringup (if not already running):
ffw_sg2_follower_ai

# Terminal 2 — All VR nodes (publisher + consumer + IK solver):
ros2 launch ffw_teleop_vr vr_direct_with_ik.launch.py

# Terminal 3 (optional) — Monitor VR topics:
ros2 run ffw_teleop_vr vr_topic_monitor
```

Then on the Quest 3 browser:
1. First go to `https://<robot-ip>:8012` and **accept the certificate warning**
2. Then go to `https://vuer.ai?ws=wss://<robot-ip>:8012`
3. Tap **Enter AR** (or Enter VR)

---

## Scenario 2 — Relay via Server PC

Quest 3 and a server PC are on an external router network. The server PC has a second network interface connected to the robot's AP. The server bridges VR data to the robot over ROS2 DDS.

```
                  External Router                     Robot AP
Quest 3  ──────────────────────>  Server PC  ──────────────────>  Robot PC
                                  vr_raw_sg2_publisher            vr_raw_sg2_consumer
                                  (Vuer server → raw topics)      (raw topics → goal poses)
                                        |                               |
                                        └── /vr/raw/* ROS2 topics ──────┘
                                            (via DDS over Robot AP)
                                                                        |
                                                                  /l_goal_pose, /r_goal_pose
                                                                        |
                                                                        v
                                                                  bimanual_ik_solver (PyRoKi)
                                                                        |
                                                                  joint trajectories
                                                                        |
                                                                        v
                                                                  Robot Arm Controllers
```

### How to Run

```bash
# On the server PC:
ros2 launch ffw_teleop_vr vr_raw_sg2_publisher.launch.py

# On the robot — Terminal 1 (bringup, if not already running):
ffw_sg2_follower_ai

# On the robot — Terminal 2 (VR consumer + IK solver):
ros2 launch ffw_teleop_vr vr_raw_sg2_with_ik.launch.py

# On the robot — Terminal 3 (optional) — Monitor VR topics:
ros2 run ffw_teleop_vr vr_topic_monitor
```

Then on the Quest 3 browser:
1. First go to `https://<server-pc-ip>:8012` and **accept the certificate warning**
2. Then go to `https://vuer.ai?ws=wss://<server-pc-ip>:8012`
3. Tap **Enter AR** (or Enter VR)

### Server PC Network Setup

The server PC needs two network connections:
1. **External router** (e.g. `eth0` or `wlan0`) — same network as Quest 3
2. **Robot AP** (e.g. `wlan1` or `eth1`) — same network as the robot

For ROS2 DDS discovery to work across the robot AP interface, ensure both the server and robot use the same `ROS_DOMAIN_ID` and that multicast or discovery-server is properly configured on the robot AP subnet.

If DDS multicast doesn't work across subnets, you can use Cyclone DDS with explicit peer addresses:

```xml
<!-- cyclone_dds.xml on both server and robot -->
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>ROBOT_AP_SUBNET_INTERFACE</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <Peers>
        <Peer address="ROBOT_IP"/>
        <Peer address="SERVER_IP_ON_ROBOT_AP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///path/to/cyclone_dds.xml
```

---

## Controller Mapping

The SG2 uses **Meta Quest 3 motion controllers** (not hand tracking).

### Arm Control (Deadman Switch)

| Input | Action |
|-------|--------|
| Both grip squeeze > 0.8 | **Enable** arm goal pose publishing |
| Controller pose (while squeezing) | Wrist target pose sent to robot arms |
| Release either grip | **Freeze** arms in current position |

Squeezing both grips acts as a deadman switch — the robot arms only track your controller movement while both grips are firmly held.

### Thumbstick Controls

The thumbsticks have two modes, toggled by pressing **both thumbsticks simultaneously**:

#### Mode 1: LIFT + HEAD (default)

| Input | Action |
|-------|--------|
| Left thumbstick X/Y | Head pan / tilt (`head_joint1`, `head_joint2`) |
| Right thumbstick X | Lift joint up / down |

#### Mode 2: LIFT + CMD_VEL (navigation)

| Input | Action |
|-------|--------|
| Left thumbstick X | Base linear X velocity (forward/back) |
| Left thumbstick Y | Base linear Y velocity (strafe left/right) |
| Right thumbstick X | Lift joint up / down |
| Right thumbstick Y | Base angular Z rotation (turn) |

### Buttons

| Input | Action |
|-------|--------|
| Trigger (analog) | Gripper squeeze (0.0 = open, 1.0 = closed) |
| Both A/X buttons | Call `/reactivate` service (error recovery) |
| Both thumbstick press | Toggle between HEAD and CMD_VEL mode |

---

## ROS2 Topics

### Raw VR Topics (publisher → consumer)

Published by `vr_raw_sg2_publisher`, consumed by `vr_raw_sg2_consumer`.
In Scenario 1 these are local topics; in Scenario 2 they travel over the network.

| Topic | Type | Description |
|-------|------|-------------|
| `/vr/raw/controller/head_pose_ros` | PoseStamped | Head/camera pose in `ros_world` |
| `/vr/raw/controller/left_pose_ros` | PoseStamped | Left controller pose in `ros_world` |
| `/vr/raw/controller/right_pose_ros` | PoseStamped | Right controller pose in `ros_world` |
| `/vr/raw/controller/left_state` | Joy | Left controller buttons/axes |
| `/vr/raw/controller/right_state` | Joy | Right controller buttons/axes |
| `/vr/raw/controller/left_elbow_pose_ros` | PoseStamped | Left elbow pose in `ros_world` |
| `/vr/raw/controller/right_elbow_pose_ros` | PoseStamped | Right elbow pose in `ros_world` |

### Robot Command Topics (consumer → robot)

Published by `vr_raw_sg2_consumer`.

| Topic | Type | Description |
|-------|------|-------------|
| `/l_goal_pose` | PoseStamped | Left arm wrist target pose |
| `/r_goal_pose` | PoseStamped | Right arm wrist target pose |
| `/l_elbow_pose` | PoseStamped | Left elbow target pose |
| `/r_elbow_pose` | PoseStamped | Right elbow target pose |
| `/leader/joystick_controller_left/joint_trajectory` | JointTrajectory | Head joint commands |
| `/leader/joystick_controller_right/joint_trajectory` | JointTrajectory | Lift joint commands |
| `/cmd_vel` | Twist | Base velocity commands |
| `/vr_controller/left_trigger` | Float32 | Left gripper value (0-1) |
| `/vr_controller/right_trigger` | Float32 | Right gripper value (0-1) |
| `/vr_controller/left_squeeze` | Float32 | Left grip squeeze value |
| `/vr_controller/right_squeeze` | Float32 | Right grip squeeze value |

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vr_control/toggle` | Bool | Enable/disable VR control |
| `/joint_states` | JointState | Current robot joint positions (subscribed) |

---

## Configurable Parameters

### vr_raw_sg2_consumer

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_camera_z` | -1.577 | Robot camera height above base_link (meters, at home lift position). Lower this value to move arm targets down. |
| `goal_pose_position_scale` | 1.1 | Arm reach scaling factor |
| `pose_publish_hz` | 30.0 | Goal pose publish rate |
| `apply_lift_to_arm_z` | true | Compensate arm Z for lift position |
| `lift_to_arm_z_scale` | 1.0 | Lift-to-arm-Z compensation scale |
| `left_wrist_offset_x/y/z` | 0.0 | Left wrist position offset (meters) |
| `right_wrist_offset_x/y/z` | 0.0 | Right wrist position offset (meters) |
| `left_wrist_roll/pitch/yaw_offset_deg` | 90/0/0 | Left wrist rotation offset (degrees) |
| `right_wrist_roll/pitch/yaw_offset_deg` | 90/0/0 | Right wrist rotation offset (degrees) |
| `left_trigger_offset` / `right_trigger_offset` | 0.0 | Trigger calibration offset |
| `left_trigger_scale` / `right_trigger_scale` | 1.0 | Trigger calibration scale |
| `left_elbow_offset_x/y/z` | 0.0 | Left elbow position offset |
| `right_elbow_offset_x/y/z` | 0.0 | Right elbow position offset |

### vr_raw_sg2_publisher

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stream_fps` | 30 | Vuer body/controller stream rate |
| `vuer_port` | 8012 | Vuer WebSocket server port |
| `cert_file` | (auto) | Path to SSL cert.pem |
| `key_file` | (auto) | Path to SSL key.pem |

### bimanual_ik_solver

| Parameter | Default | Description |
|-----------|---------|-------------|
| `left_target_pose_topic` | `/l_goal_pose` | Left arm target pose topic |
| `right_target_pose_topic` | `/r_goal_pose` | Right arm target pose topic |
| `left_end_effector_link` | `arm_l_link7` | Left arm end effector link name |
| `right_end_effector_link` | `arm_r_link7` | Right arm end effector link name |
| `left_ik_solution_topic` | `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory` | Left arm joint trajectory output |
| `right_ik_solution_topic` | `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory` | Right arm joint trajectory output |
| `max_joint_step_rad` | 0.15 | Normal step size per cycle per joint |
| `max_joint_jump_rad` | 0.2 | Threshold above which slow crawl step is used instead |
| `large_jump_step_rad` | 0.02 | Slow crawl step when far from target (e.g. on startup) |

---

## Coordinate System

VR and ROS use different coordinate conventions:

```
VR world frame:    +X = right, +Y = up,      +Z = backward
ROS world frame:   +X = forward, +Y = left,  +Z = up

VR_WORLD_TO_ROS_WORLD = [[0, 0, -1],
                          [-1, 0, 0],
                          [0, 1, 0]]
```

The publisher converts all poses from VR world to ROS world frame before publishing.
The consumer then computes each controller pose relative to the head, and maps it to the robot's `base_link` frame using a fixed `camera_to_base_offset`.

---

## Development Workflow

### Sync code to the robot

After editing on your dev machine, copy `custom/ffw_teleop_vr/` to the robot with your preferred file sync tool.

```bash
# From the repo root, for example:
rsync -av custom/ffw_teleop_vr/ user@robot-ip:/path/to/ai_worker/custom/ffw_teleop_vr/
```

### Build on the robot

After syncing, rebuild the package on the robot:

```bash
cd ~/ros2_ws && colcon build --packages-select ffw_teleop_vr && source install/setup.bash
```

---

## Testing / Debugging

### Verify Quest 3 connectivity

Before running the full teleop, confirm the VR connection and ROS2 topics are working:

```bash
# 1. Start the publisher (Scenario 1: on robot, Scenario 2: on server PC)
ros2 launch ffw_teleop_vr vr_raw_sg2_publisher.launch.py

# 2. Connect the Quest 3 browser:
#    - Go to https://<ip>:8012 and accept the certificate warning
#    - Go to https://vuer.ai?ws=wss://<ip>:8012 and tap Enter AR

# 3. Verify topics are flowing:
ros2 run ffw_teleop_vr vr_topic_monitor
```

---

## File Descriptions

| File | Description |
|------|-------------|
| `ffw_teleop/vr_raw_sg2_publisher.py` | Vuer WebXR server — streams Quest 3 data to `/vr/raw/*` ROS topics |
| `ffw_teleop/vr_raw_sg2_consumer.py` | Converts raw VR topics to robot goal poses and joint commands |
| `ffw_teleop/pyroki_ik/bimanual_ik_solver.py` | PyRoKi bimanual IK solver (goal poses → joint trajectories) |
| `ffw_teleop/pyroki_ik/pyroki_snippets/` | PyRoKi IK core algorithm |
| `ffw_teleop/vr_topic_monitor.py` | GUI topic monitor for VR data |
| `cert/cert.pem` / `cert/key.pem` | SSL certificate and key for WebXR (runs on publisher host) |
| `launch/vr_direct_with_ik.launch.py` | **Scenario 1**: All nodes on the robot |
| `launch/vr_raw_sg2_publisher.launch.py` | **Scenario 2**: Publisher node (run on server PC) |
| `launch/vr_raw_sg2_with_ik.launch.py` | **Scenario 2**: Consumer + IK solver (run on robot) |
| `launch/bimanual_ik_solver.launch.py` | IK solver standalone |

---

## Quick Reference

| Scenario | Where | Command |
|----------|-------|---------|
| 1 (Direct) | Robot | `ros2 launch ffw_teleop_vr vr_direct_with_ik.launch.py` |
| 2 (Relay) | Server PC | `ros2 launch ffw_teleop_vr vr_raw_sg2_publisher.launch.py` |
| 2 (Relay) | Robot | `ros2 launch ffw_teleop_vr vr_raw_sg2_with_ik.launch.py` |
| IK only | Robot | `ros2 launch ffw_teleop_vr bimanual_ik_solver.launch.py` |
