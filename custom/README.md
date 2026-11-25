# Local AI Worker customizations

Keep locally maintained code, configuration, launch files, and documentation
inside `custom/`. The rest of the repository belongs to ROBOTIS, so upstream
updates can be merged without mixing local changes into official packages.

## Layout

```text
custom/
  robot/                            # Onboard Jetson Orin
    bringup/ffw_custom_robot/        # Mixed SG2 left 2F + SH5 right hand
  pc/                               # Desktop, laptop, or UMPC running Ubuntu
    control/                        # Keyboard/GUI teleop, presets, dataset init
    joystick/allyx-sg2-controller/   # Optional Foxglove gamepad extension
    network/                        # Zenoh environment helper and network notes
    topic_check/                    # Camera checks and reference TF diagrams
  shared/
    record/                         # Single GUI/headless implementation for PC and robot
  archive/                          # Retired custom code; excluded from Colcon
    robot/teleop/ffw_teleop_vr/     # Legacy custom VR implementation
    pc/map_nav/                     # Legacy navigation helpers, maps and waypoints
    pc/deployment/                  # Legacy VR sync helper
```

Choose the [robot guide](robot/README.md) or [PC guide](pc/README.md).
Desktop and UMPC use the same PC code. Joystick support is an optional function,
so a new device does not need a separate directory or copy of the helpers.
Put future additions under the appropriate device and purpose.

Use official ROBOTIS support for VR and navigation. The earlier custom versions
are preserved in [archive](archive/README.md), with `COLCON_IGNORE` to exclude
them from normal workspace discovery and builds. Recording has one implementation
in [shared/record](shared/record/README.md), usable on either the PC or the robot.

## Start on a new device

1. Clone this fork and follow the official setup for the device's architecture.
   Build and run ROS code in its corresponding ROS/container environment.
2. Follow the device guide above. PC helpers are scripts, not ROS packages;
   editing them does not require `colcon build`.
3. Set `ROBOT_IP` and, if needed, `ROS_DOMAIN_ID` for the connected robot. The
   existing default domain is `30`; the keyboard launcher retains the existing
   fallback robot IP `192.168.6.2`. Configure these for a new setup.
4. Recreate dependencies from `pyproject.toml` / `uv.lock` or `package-lock.json`.
   Environments, build outputs, recordings, and private TLS keys are not in Git.
   Dataset initialization accepts `--data-root` or `LEROBOT_DATA_ROOT`; datasets
   and the old checkout's `lerobot` symlink are not copied.

The official Docker Compose file mounts the repository at
`/root/ros2_ws/src/ai_worker/`, even when the host checkout is named
`ai_worker_custom`. The PC keyboard launcher uses that mount by default;
override `CTR_REPO` if your container uses another path.

## Updating from ROBOTIS

`origin` is the existing `teamROBI/ai_worker` fork. Configure a separate upstream
remote once, if it is not already configured:

```bash
git remote add upstream https://github.com/ROBOTIS-GIT/ai_worker.git
```

After reviewing and committing your local work, update the fork using a merge
to preserve its existing history:

```bash
git switch main
git status --short                 # commit or otherwise save pending work first
git fetch upstream
git merge upstream/main
```

Review the result and validate on the relevant device before pushing the fork.
Upstream may change package APIs or launch arguments; isolation avoids overlapping
edits, but compatibility still needs checking after updates. If ROBOTIS introduces
its own `custom/` directory, inspect that overlap before merging.

For future local work, review `git diff --name-only` and
`git ls-files --others --exclude-standard` before committing: your paths should
start with `custom/`. Use custom launch wrappers, parameter files, and separate
ROS packages to extend upstream behavior. Keep ignore rules in `custom/.gitignore`.

See [migration notes](MIGRATION.md) for the source-to-destination mapping and the
small path changes made during this migration.
