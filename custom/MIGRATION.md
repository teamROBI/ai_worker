# Migration record

Source checkouts: `/home/robi/projects/ai_worker/` and the existing
`/home/robi/projects/ai_worker_custom/custom/` tree. The UMPC source directory is
actually named `help_scripts_umpc/` (plural `scripts`). The original checkout was
left untouched, including its uncommitted official-package changes.

| Previous location | New location under `custom/` |
| --- | --- |
| `custom/ffw_custom_robot/` | `robot/bringup/ffw_custom_robot/` |
| `custom/ffw_teleop_vr/` | `archive/robot/teleop/ffw_teleop_vr/` |
| `custom/scripts/record/` | `shared/record/` (identical headless recorder consolidated) |
| Both helper trees: `control/` | `pc/control/` |
| Both helper trees: `map_nav/` | `archive/pc/map_nav/` |
| Both helper trees: `record/` | `shared/record/` |
| Both helper trees: `topic_check/` | `pc/topic_check/` |
| Both helper trees: `network/` | `pc/network/` |
| Both helper trees: `zenoh_robot_env.sh` | `pc/network/zenoh_robot_env.sh` |
| Both helper trees: `sync_teleop_vr.sh` | `archive/pc/deployment/sync_teleop_vr.sh` |
| `help_scripts_umpc/control/allyx-sg2-controller/` | `pc/joystick/allyx-sg2-controller/` |

The two helper trees had 81 byte-identical files after excluding generated data
and dependencies. The Ally Foxglove extension is the only additional source
project in the UMPC tree. Desktop, laptop and UMPC use one PC implementation;
joystick support is organized as a function. There are no duplicate ROS
packages or additional Git repositories.

## Migration adjustments

- Updated executable/documentation paths for the deeper layout.
- Fixed host-to-container repository lookup in the keyboard launcher; forwarded
  arguments now retain spaces and literal shell characters through Docker.
- Kept the dataset default at `<repo>/lerobot/ACS_ROBI` and added `--data-root` /
  `LEROBOT_DATA_ROOT` so each device can choose its dataset location. No dataset
  symlink is installed automatically.
- Added a Bash shebang to `mapping.sh` so direct execution uses its intended shell.
- Updated VR sync to write within the destination checkout's `custom/`, exclude
  the private key, preserve remote-only files, and support rsync dry runs.
- Corrected the legacy map YAML's image reference to the preserved
  `market_417_old.pgm` filename.
- Retained source code, configs, lockfiles, presets, reference images, map images,
  manuals and reference TF diagrams. The headless recorder remains byte-identical
  to both previous copies; there is one implementation in `shared/record/`, usable
  on the PC or robot. Its output directory is now `shared/record/videos/` on either.
- Archived the retired custom VR package, related sync helper, and map/navigation
  tools because this setup now uses official ROBOTIS support. The archive retains
  device/function organization and is excluded from Colcon via a tracked marker.

Not copied: `.venv/`, `node_modules/`, Python caches, ROS `build/`, `install/`,
`log/`, recorded `videos/`, extension `dist/`, and the generated `.foxe` bundle.
These remain in the original checkout and can be recreated or copied separately
as local data. The included PDF manuals/TF diagrams retain their historical text;
use the Markdown guides for current paths.

No official files, remote configuration, branches or history were changed. No
commit, push, robot deployment, or robot control command was performed.
