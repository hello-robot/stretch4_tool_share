#!/usr/bin/env python3
from stretch4_body.core.gamepad_joints import CommandStretchGripperPosition


class NyuGripperGamepadTeleop:
    """
    Gamepad teleoperation control mapping for nyu_gripper.
    Maps joystick button and axis events to joint commands.
    """
    def __init__(self, robot):
        self.robot = robot

    def update_teleop(self, gamepad_state):
        """
        This function is called at every iteration of the gamepad teleop loop.

        """
        pass

class CommandNyuGripperPosition(CommandStretchGripperPosition):
    """NYU tendon gripper motion command class.
    Pct-based with the same open/close behavior as the Stretch Gripper.
    CommandStretchGripperPosition auto-resolves the joint name to
    'nyu_gripper' via RobotJoints.gripper, so the base init is used as-is.
    """
    def __init__(self, motion_profile:str = 'max'):
        CommandStretchGripperPosition.__init__(self, motion_profile)


# gamepad_teleop.set_joint_command() loads a custom tool's gripper command
# class by this exact name.
CommandCustomToolPosition = CommandNyuGripperPosition
