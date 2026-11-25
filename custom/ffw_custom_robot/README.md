# Custom mixed-gripper robot package

This package keeps the local SG2 left 2F and SH5 right hand configuration
separate from the ROBOTIS packages. It depends on the official `ffw_bringup`
and `ffw_description` packages for shared nodes, launches, meshes, and URDF
components.

Build it from the ROS workspace root:

```bash
colcon build --packages-up-to ffw_custom_robot
source install/setup.bash
```

Launch the robot with:

```bash
ros2 launch ffw_custom_robot ffw_sg2l_sh5r_follower_ai.launch.py
```
