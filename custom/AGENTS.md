# Customization boundary

- Keep local changes within `custom/`. Files outside it are maintained by ROBOTIS;
  extend them through custom packages, launch wrappers, or configuration here.
- Organize by execution device (`robot` or `pc`), then by function. Desktop,
  laptop, and UMPC use the same `pc` code; joystick support is a PC function.
- Put implementations used on both the robot and PC under `shared/<function>/`.
- Keep retired custom code under `archive/`, preserving its device/function
  layout. Track `archive/COLCON_IGNORE` so it stays out of normal ROS builds.
- Maintain one recording implementation in `shared/record/`, usable on either
  computer. Do not duplicate it under `robot/`.
- Preserve ROS package names and existing control behavior when reorganizing code.
- Keep recordings, datasets, virtual environments, dependency directories, build
  outputs, and private TLS keys out of Git. Use `custom/.gitignore` for local rules.
- Never exercise robot motion as a migration or static validation check.
- Leave source checkouts, Git remotes, branches, and history alone unless the user
  requests a change to them. Document upstream update commands in `README.md`.
