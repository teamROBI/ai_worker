# Camera recording — PC or robot

This is the single recording implementation for the PC and robot. Their former
headless recorder copies were identical and have been consolidated here.

- `record_cameras_headless.py`: records without a display.
- `record_cameras.py`: GUI recorder for a machine with a display.
- [MANUAL_KR.md](MANUAL_KR.md): setup and usage instructions with current paths.
- `MANUAL_KR.pdf`: historical PDF manual.

From the repository root in a sourced ROS/container shell on either machine:

```bash
python3 custom/shared/record/record_cameras_headless.py my_episode
```

When running on a PC connected to the robot, join the robot's network first:

```bash
ROBOT_IP=192.168.6.2 source custom/pc/network/zenoh_robot_env.sh
```

Onboard the robot, use the robot's existing ROS network environment. The GUI
version needs a display; both scripts require their existing ROS/Python and video
encoding dependencies, described in the manual.

Recordings are written to `custom/shared/record/videos/` on the executing machine
and ignored by Git. Each machine keeps its own recordings.
