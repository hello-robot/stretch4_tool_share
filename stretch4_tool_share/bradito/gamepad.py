#!/usr/bin/env python3
"""
Gamepad teleop mapping for the Bradito gripper.

GamePadTeleop loads `<ToolName>GamepadTeleop` and `Command<ToolName>Position` from this
module by name for any user-defined tool, and raises if either is missing -- so both
classes are required even though the generic CommandToolPosition already handles this tool.
"""

from stretch4_body.core.gamepad_joints import CommandToolPosition

TOOL_NAME = 'bradito'


class BraditoGamepadTeleop:
    """
    Per-iteration hook for gamepad teleop. The standard open/close buttons are handled by
    CommandBraditoPosition below, so there is nothing tool-specific to add here.
    """

    def __init__(self, robot):
        self.robot = robot

    def update_teleop(self, gamepad_state):
        """Called at every iteration of the gamepad teleop loop."""
        pass


class CommandBraditoPosition(CommandToolPosition):
    """
    Bradito motion command class. Steps by 10% of command_range per button press, with the
    velocity/acceleration of the requested motion profile, exactly like the built-in tools.
    """

    def __init__(self, motion_profile: str = 'max'):
        CommandToolPosition.__init__(self, name=TOOL_NAME, motion_profile=motion_profile)
