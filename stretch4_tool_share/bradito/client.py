#!/usr/bin/env python3
"""
Client-side interface to the Bradito gripper joint.

This is the class BraditoMetadata.client_class hands back, so it is what
`robot.end_of_arm.gripper` is, and what `stretch_gripper_jog` / `stretch_gripper_home`
drive. It is a thin subclass of the generic ToolJointClient: the tool needs no bespoke
remote behavior beyond the unit conversions its metadata already provides.
"""

from stretch4_body.robot.robot_client import ToolJointClient

TOOL_NAME = 'bradito'


class BraditoClient(ToolJointClient):
    """ Client for the Bradito tendon gripper. Positions are in pct (0 closed, 100 open). """

    def __init__(self, metadata=None, parent=None, ip_address=None):
        if metadata is None:
            # Constructed without its metadata (e.g. by a tool that only knows the class).
            from stretch4_body.utils.tool_metadata import get_tool_metadata
            metadata = get_tool_metadata(TOOL_NAME)
        ToolJointClient.__init__(self, metadata, parent=parent, ip_address=ip_address)

    def home(self, end_pos=100.0, wait_on_completion=True, timeout=20):
        """Finish homing fully open (pct units), matching the direct driver's default."""
        return ToolJointClient.home(
            self, end_pos=end_pos, wait_on_completion=wait_on_completion, timeout=timeout
        )

    # stretch_gripper_jog rebinds move_to/move_by for a custom tool as
    # `lambda x, v, a: move_to(<tool name>, x, v, a)`, on the assumption that a custom
    # tool's client is an EndOfArmClient. This client is a joint client, so swallow a
    # leading joint-name argument to stay usable from that tool.
    def move_to(self, *args, **kwargs):
        return ToolJointClient.move_to(self, *self._strip_joint_name(args), **kwargs)

    def move_by(self, *args, **kwargs):
        return ToolJointClient.move_by(self, *self._strip_joint_name(args), **kwargs)

    @staticmethod
    def _strip_joint_name(args):
        return args[1:] if args and isinstance(args[0], str) else args
