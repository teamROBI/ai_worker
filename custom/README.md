# Local customizations

This directory contains locally maintained code and configuration that is not
provided by ROBOTIS.

- `ffw_custom_robot/` is an independent ROS package for the mixed SG2 left 2F
  and SH5 right hand setup. It installs its own launch, config, URDF, and
  ros2_control files while depending on the standard ROBOTIS packages.
- `ffw_teleop_vr/` is the earlier custom ROS 2 VR teleoperation package.
- `scripts/` contains local helper scripts and recordings.

The custom packages and tools stay within this directory, so they can be
committed to a fork without adding files to the ROBOTIS package directories.
