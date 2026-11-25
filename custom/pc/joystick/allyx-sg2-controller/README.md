# Ally SG2 Controller for Foxglove

A local Foxglove custom panel for controlling the AI Worker SG2 with the Ally's
built-in Xbox-style gamepad. It sends bounded jog intents to Cyclo Manager's
existing `/ws/jog` endpoint. Cyclo Manager remains responsible for ROS 2
publishers, robot readiness checks, limits, cadence, and stop handling; this
extension does not publish ROS topics directly.

## Controller mapping

- Connect to Cyclo Manager, then explicitly press **Enable Jog**.
- **Hold LB** as the deadman while moving.
- **Base mode:** left stick drives forward/lateral; right stick X turns.
- **Joint mode:** choose a joint in the panel, then use left stick up/down.
  Stick deflection selects fine, normal, or coarse jog increments.
- Releasing LB, centering the stick, losing focus, disconnecting the controller,
  or losing the manager connection requests a stop. Focus loss, controller
  disconnect, feedback timeout, and connection loss disarm Jog.

The defaults assume Foxglove Desktop is running on the Ally while connected to
the robot Wi-Fi:

- Cyclo Manager: `ws://192.168.6.2:8081`
- Container: `ai_worker`

Both fields are editable in the panel. Confirm the Manager address from the
Cyclo Manager UI if it is hosted at a different robot address.

If the Ally is not on the robot Wi-Fi but can SSH through the `ai_worker_ip`
alias, keep this tunnel running in a terminal:

```sh
ssh -N -L 18081:127.0.0.1:8081 ai_worker_ip
```

Then set the panel's Manager address to `ws://127.0.0.1:18081`.

## Install into Foxglove Desktop

From this directory:

```sh
npm install
npm run local-install
```

Open or refresh Foxglove Desktop, then add **Ally SG2 Controller** as a panel.
Foxglove's current extension docs require a developer seat to install local
extensions.

Use Foxglove Desktop for the default `ws://` Manager connection. A browser-hosted
Foxglove page served over HTTPS may block an insecure `ws://` connection; that
setup needs a `wss://` endpoint.

## Telemetry and 3D visualization

This panel receives live Jog state from Cyclo Manager. Foxglove's normal topic,
camera, and 3D panels need a separate Foxglove-compatible ROS 2 data source, such
as `foxglove_bridge`, connected to the robot's ROS 2 graph. That telemetry link
is independent of the command path through `/ws/jog`.

Cyclo Manager's API is unauthenticated by default. Keep its port on the trusted
robot network; do not expose it to the public internet.
