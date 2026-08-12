# NYU Gripper

A tendon-driven parallel gripper for Stretch 4, designed at NYU. A single Feetech smart servo winds a Kevlar tendon to close the two fingers; a return spring opens them. Gripper position is commanded as a unitless percentage: `0` = fully closed, `100` = fully open.

Project page: https://nyu-gripper.pages.dev

## Hardware requirements

- Stretch 4 with the DexWrist v4 (`tool_params.yaml` sets `wrist: eoaw_dw4`)
- Feetech smart servo configured to **ID 25**, connected to the wrist bus (`/dev/hello-feetech-wrist`)

> **Warning:** the `eeprom_cfg` block in `tool_params.yaml` (protection limits, PID gains, multi-turn `phase: 61`, encoder polarity) is written to the servo's EEPROM at startup. Only connect a servo you intend to configure this way, and make sure its ID matches `id: 25` first — a wrong ID causes a failed ping and a server restart loop.

## Installation

On your robot:

```bash
cp -r stretch4_tool_share/nyu_gripper ~/stretch_user/user_tools/
stretch_add_user_tool nyu_gripper
stretch_configure_tool          # select nyu_gripper
stretch_body_server --restart
```

Verify with:

```bash
stretch_add_user_tool nyu_gripper --check
stretch_gripper_jog             # 'x' / 'y' to open / close
stretch_collision_viz
```

## Homing

The tendon only transmits force in the closing direction, so this tool overrides the stock `home()`: it drives the spool closed under PWM and detects the hardstop by position settling (the default velocity-stall detection does not work for tendon-driven mechanisms), then finishes fully open at 100%.

Before homing, confirm the tendon is anchored to the spool — a slipping tendon prevents the stall from being detected and homing times out. Home with `stretch_gripper_home` (or `robot.end_of_arm.home()`); the gripper requires homing after startup before it will accept motion commands (`req_calibration: 1`).

## Usage

- Position commands in percent via the end-of-arm interface: `robot.end_of_arm.move_to('nyu_gripper', 50.0)` / `move_by(...)`
- Named servo poses: `zero`, `open`, `close`
- Named robot poses (`pose_models.yaml`): `stow`, `zero`, usable with the pose tools
- Gamepad teleop: standard gripper open/close buttons via `stretch_gamepad_teleop`

## Credits

Gripper design and drivers by NYU — https://nyu-gripper.pages.dev. Shared under the repository's [Apache License 2.0](../LICENSE).
