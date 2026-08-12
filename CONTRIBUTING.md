# Contributing a tool

Thank you for sharing your tool with the Stretch community! Contributions must follow the guidelines below so that every tool in this repository installs and works the same way.

A contribution is one complete, robot-tested tool: a single folder at the repository root, in exactly the format used by `~/stretch_user/user_tools/<tool_name>/` on the robot. Installing it should require nothing more than copying the folder onto a robot and running `stretch_add_user_tool <tool_name>`.

## What to share

Share every file needed to make your tool work on a Stretch 4 — the meshes, the configuration YAMLs, and the Python drivers — plus documentation that tells others how to build and use it:

- `meshes/` — visual STLs plus the generated `*_collision_link.STL` files, so installs work without regeneration
- `tool.urdf` — root link must be `quick_connect_interface_link`; mesh paths use `$(arg tool_mesh_dir)`, no absolute paths
- `tool_params.yaml` — declares the tool/driver classes, devices, stow pose, and collision management
- `collision_mesh_config.yaml`
- `pose_models.yaml` — if your tool defines named poses
- Python drivers: `tool.py`, `end_of_arm.py`, `client.py` (plus `command_group.py`, `gamepad.py`, `collision.py` as applicable)
- `README.md` — how others can use your tool: description and photos, hardware requirements, bill of materials, assembly notes, install steps, homing caveats, usage (API and teleop), and author credit

Encouraged: `CAD/` (STEP preferred) and `images/`.

## Do not commit

- `__pycache__/` or `*.pyc` (gitignored)
- Absolute paths in `tool.urdf` or configs
- Robot-specific calibration values

## Validate before opening a PR

On your robot, from a fresh copy of your folder in `~/stretch_user/user_tools/`:

1. `stretch_add_user_tool <tool_name>` completes cleanly
2. `stretch_add_user_tool <tool_name> --check` passes
3. Activate with `stretch_configure_tool`, restart with `stretch_body_server --restart`, home the tool, and exercise it (`stretch_gripper_jog`, `stretch_collision_viz`, `stretch_gamepad_teleop`)
4. `python3 -m py_compile *.py` succeeds

## Submitting your tool

1. Fork this repository and create a branch for your tool (e.g. `feature/add-<tool_name>`). One tool per branch and per pull request.
2. Add your tool folder at the repository root and add your tool's row to the gallery table in the root README, including a photo of your tool placed in `assets/` (e.g. `assets/<tool_name>.jpeg`).
3. Open a pull request containing all the details a reviewer needs to understand your tool **before** testing it:
   - What the tool is and what it does, with photos (or a short video link)
   - Hardware required to reproduce it (wrist version, servos and their IDs, printed parts, BOM)
   - The robot model and software versions it was tested on
   - How you validated it (output of `stretch_add_user_tool <tool_name> --check`, homing and teleop behavior)
   - Any safety notes or known limitations
4. A reviewer will install, test, and validate the tool before approving and merging the PR. Incomplete PRs will be sent back for details, so include everything up front.

## Licensing

Contributions are accepted under the repository's [Apache License 2.0](LICENSE). Put attribution and credits in your tool's README; you may keep copyright headers in your own source files.
