# Map / Nav Helpers

> Archived custom helpers, superseded in this setup by official ROBOTIS
> navigation support. Scripts, maps, parameters, and waypoints are retained for
> reference; the instructions below describe the retired workflow.

These scripts wrap the current `ffw_navigation` flow in this repository.
They assume your Docker terminal already has ROS 2 and the workspace sourced.

## Scripts

`mapping.sh`
- Starts `ffw_navigation` in SLAM mode with `mapping.rviz`.

`save_map.sh [map_prefix]`
- Saves the current SLAM map under `custom/archive/pc/map_nav/maps`.
- Example: `./save_map.sh office_a`

`navigation.sh [map_yaml]`
- Starts `ffw_navigation` in localization + navigation mode.
- Relative map names are resolved from `custom/archive/pc/map_nav/maps` when present.
- If no map is passed, it uses the package default from `ffw_navigation`.

## Typical flow

Terminal 1:
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

Terminal 2:
```bash
./mapping.sh
```

Drive the robot, then save the map:
```bash
./save_map.sh office_a
```

Later, navigate on that map:
```bash
./navigation.sh office_a.yaml
```

## Useful overrides

Simulation:
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py use_sim:=true
USE_SIM_TIME=true ./mapping.sh
USE_SIM_TIME=true ./navigation.sh
```
