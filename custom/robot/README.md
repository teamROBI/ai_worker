# Robot — Jetson Orin

The active onboard customization is grouped by purpose:

- `bringup/ffw_custom_robot/`: independent mixed-gripper robot package.

Build from the ROS workspace root in the robot's existing ROS environment:

```bash
colcon build --packages-up-to ffw_custom_robot
source install/setup.bash
ros2 launch ffw_custom_robot ffw_sg2l_sh5r_follower_ai.launch.py
```

Rebuild after moving the packages; old build/install artifacts can retain their
previous source paths. Package names and ROS launch commands are unchanged.
See the [bringup package](bringup/ffw_custom_robot/README.md) for setup and launch
details. Use official ROBOTIS VR support; the previous custom package is retained
in [archive](../archive/README.md) and excluded from normal builds.

Recording uses the same implementation on either machine. From the repository
root in a sourced ROS shell on the robot:

```bash
python3 custom/shared/record/record_cameras_headless.py my_episode
```

Recordings go to `custom/shared/record/videos/` on the machine running the script.
See the [recording guide](../shared/record/README.md) for dependencies and network setup.
