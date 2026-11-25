# Archived VR package sync

This helper belongs to the retired custom VR implementation. It is retained for
reference and copies only into the destination checkout's `custom/archive/`.

Prefer checking out this fork on each device. If you need to copy just the custom
VR package, this helper resolves it from its new source location and sends it to
the same `custom/archive/robot/teleop/ffw_teleop_vr/` path in the destination checkout.
The target argument is the **repository root**, not a ROS package directory.

```bash
# From the repository root; preview before copying:
custom/archive/pc/deployment/sync_teleop_vr.sh ai_worker:/home/robotis/ai_worker --dry-run

# Copy when ready:
custom/archive/pc/deployment/sync_teleop_vr.sh ai_worker:/home/robotis/ai_worker
```

Requires rsync with `--mkpath` support on both ends. Remote files are not deleted;
the robot's private TLS key is excluded. Copying archived sources does not enable
the package: `custom/archive/COLCON_IGNORE` keeps it out of normal builds. This
helper does not remove older checkouts or duplicate packages.
