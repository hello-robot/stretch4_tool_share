# Bradito

A tendon-driven parallel gripper for Stretch 4 — mechanically the NYU gripper
(https://nyu-gripper.pages.dev), packaged as a **custom user end-of-arm tool** against the
`feature/eoa-tooling` architecture (`ToolMetadata`, generic `GripperCommandGroup`,
metadata-driven clients).

A single Feetech servo winds a Kevlar tendon to close the two fingers; a return spring opens
them. Position is commanded as a unitless percentage: `0` = fully closed, `100` = fully open.

## Hardware requirements

- Stretch 4 with the DexWrist v4 (`tool_params.yaml` sets `wrist: eoaw_dw4`)
- Feetech smart servo at **ID 25** on the wrist bus (`/dev/hello-feetech-wrist`)

> **Warning:** the `eeprom_cfg` block in `tool_params.yaml` (protection limits, PID gains,
> multi-turn `phase: 61`, encoder polarity) is written to the servo's EEPROM at startup. Only
> connect a servo you intend to configure this way, and make sure its ID matches `id: 25`
> first — a wrong ID causes a failed ping and a server restart loop.

## Files

| File | Role |
|---|---|
| `tool_params.yaml` | Wires up the three components and holds every hardware parameter. Merged onto the `eoa_wrist_dw4_tool_nil` baseline. |
| `bradito_metadata.py` | `BraditoMetadata(ToolMetadata)` — Path B (nonlinear) unit conversions between `urdf`/`command`/`actuator`/`aperture`/`normalized`, plus the analytic Jacobian for velocities. |
| `tool.py` | `BraditoGripper(FeetechSMHello)` — the servo driver, including tendon-aware homing. |
| `end_of_arm.py` | `Bradito(EndOfArm)` — the end-of-arm chain: stow order and the homing sequence. |
| `client.py` | `BraditoClient(ToolJointClient)` — the joint client `robot.end_of_arm.gripper` resolves to. |
| `gamepad.py` | `BraditoGamepadTeleop` / `CommandBraditoPosition` — required by name for any user tool. |
| `collision.py` | `BraditoCollision` — maps status onto the URDF finger joints for the self-collision model. |
| `tool.urdf`, `meshes/`, `collision_mesh_config.yaml` | Kinematics and geometry. Visual and collision meshes are already generated. |
| `pose_models.yaml` | Named robot poses (`stow`, `zero`). |

## Units

| Unit | Bradito |
|---|---|
| `command` | pct, `0`..`100`. What `move_to()`/`move_by()` take. |
| `actuator` | servo angle, `0`..`3.264` rad (`range_deg: [0, 187]`). Linear in pct. |
| `aperture` | fingertip opening, `0`..`0.145` m. Linear in pct. |
| `urdf` | `brd_finger_*_joint` angle, `0`..`0.704` rad. `aperture = 2·L·sin(urdf)`, so this edge is nonlinear — hence a `ToolMetadata` subclass rather than `LinearToolMetadata`. |
| `normalized` | `0.0`..`1.0`, derived from `actuator_range`. |

## Install

The directory already lives in `~/stretch_user/user_tools/bradito`. Activate it with:

```bash
stretch_configure_tool          # pick "Bradito" from the list
stretch_body_server --restart
stretch_robot_home              # or stretch_gripper_home for the tool alone
```

## Homing

The tendon only transmits force in the closing direction, so `BraditoGripper.home()`
overrides the stock velocity-stall detection: it drives the spool closed under PWM
(`homing_pwm: -80`) and detects the hardstop by position settling
(`homing_stall_pos_window_t`), then finishes fully open at 100%.

Before homing, confirm the tendon is anchored to the spool — a slipping tendon prevents the
stall from being detected and homing times out after 15 s. The gripper requires homing after
startup before it accepts motion commands (`req_calibration: 1`).

## Verify

```bash
stretch_gripper_jog        # x / y to close / open
stretch_collision_viz      # fingers should track the hardware
stretch_gamepad_teleop     # standard gripper open/close buttons
```

## Credits

Gripper design and original drivers by NYU — https://nyu-gripper.pages.dev.
