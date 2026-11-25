# Joystick and gamepad control

The [Ally SG2 Controller](allyx-sg2-controller/README.md) is the Foxglove panel
originally developed on the ROG Xbox Ally X. It uses the gamepad interface;
there is no separate UMPC code tree.

From the repository root on the PC running Foxglove Desktop:

```bash
cd custom/pc/joystick/allyx-sg2-controller
npm ci
npm run build
npm run local-install
```

Open or refresh Foxglove Desktop and add the **Ally SG2 Controller** panel.
Configure the Cyclo Manager address and container in the panel for your setup.
See the extension guide for its controller mapping, enable/deadman behavior,
network connection, and telemetry setup.

Source and `package-lock.json` are tracked. `node_modules/`, `dist/`, and
packaged `.foxe` files are generated locally and ignored.
