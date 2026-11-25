# Retired custom code

These implementations are no longer used in this setup because official ROBOTIS
support now covers VR teleoperation and navigation. Sources and supporting assets
are preserved here for reference, grouped by their original device and purpose.

| Location | Contents |
| --- | --- |
| [robot/teleop/ffw_teleop_vr](robot/teleop/ffw_teleop_vr/README.md) | Custom VR package, including publisher, consumer, IK and launch files |
| [pc/map_nav](pc/map_nav/README.md) | Custom navigation scripts, parameters, maps and waypoints |
| [pc/deployment](pc/deployment/README.md) | Sync helper for the retired VR package |

`COLCON_IGNORE` excludes this entire tree from normal Colcon package discovery
and builds. Keep that marker tracked. Archived documentation contains historical
setup and launch instructions; active device guides use official support instead.
The private VR TLS key remains ignored.
