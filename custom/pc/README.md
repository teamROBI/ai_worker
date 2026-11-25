# PC tools — desktop, laptop and UMPC

These tools run on an Ubuntu PC connected to the robot. The desktop and UMPC
helper trees were identical except for the Ally Foxglove extension, so there
is one implementation of each function here.

| Purpose | Entry point / documentation |
| --- | --- |
| GUI robot control, presets and dataset initialization | [control/README.md](control/README.md) |
| Optional joystick/gamepad control | [joystick/README.md](joystick/README.md) |
| Camera recording (PC or robot) | [shared recording guide](../shared/record/README.md) |
| Robot Zenoh environment | `network/zenoh_robot_env.sh` |
| Historical direct cable network notes | [network/README.md](network/README.md) |
| Camera topic inspection | `topic_check/rgb_check.py` |

## Keyboard/GUI control

From the repository root on the PC host:

```bash
ROBOT_IP=192.168.6.2 custom/pc/control/run_keyboard_control.sh
```

The launcher runs in the existing `ai_worker` container and joins the robot's
Zenoh router. Configure the robot IP and domain for the connected robot.

## Other ROS helpers

Enter the existing ROS container and source its workspace. From the mounted
repository root, join the robot's router before launching tools:

```bash
ROBOT_IP=192.168.6.2 source custom/pc/network/zenoh_robot_env.sh
python3 custom/shared/record/record_cameras_headless.py my_episode
# Or: python3 custom/pc/topic_check/rgb_check.py
```

Joint presets and reference images are retained as editable source assets.
The single recording implementation runs on either PC or robot. Generated videos
are ignored by Git. The PDF manuals and TF diagram remain as historical references;
use the Markdown recording manual for current paths.

## Optional PC features

Joystick support is a function, independent of whether the PC is a desktop or
UMPC. Follow the [joystick guide](joystick/README.md) to install the Foxglove panel
on the PC running Foxglove Desktop. The panel uses Cyclo Manager rather than the
keyboard launcher's ROS connection.

Use official ROBOTIS support for VR and navigation. The former custom VR package,
map/navigation tools, maps, waypoints, and VR sync helper are retained in
[archive](../archive/README.md) for reference.
