#!/usr/bin/env python3
"""
Collision joint mapping for the Bradito gripper.

The self-collision sentry routes every user-defined tool through this file (see
`get_custom_tool_joints` in stretch4_body/core/mujoco_urdf.py), so the tool's finger joints
are only animated in the collision model if `BraditoCollision.get_mujoco_joints` returns
them. Unknown joint names are skipped silently, so these must match the URDF exactly.
"""

from stretch4_body.utils.tool_metadata import get_tool_metadata

TOOL_NAME = 'bradito'


class BraditoCollision:
    """Maps the tool's hardware status onto its URDF finger joints."""

    def __init__(self, robot=None):
        self.robot = robot
        self._metadata = None

    @property
    def metadata(self):
        if self._metadata is None:
            self._metadata = get_tool_metadata(TOOL_NAME)
        return self._metadata

    def get_mujoco_joints(self, state):
        """
        Given the raw robot status dictionary, return {URDF joint name: position}.

        Both fingers are driven off the one tendon, so they share a single angle.
        """
        tool_status = state.get('end_of_arm', {}).get(TOOL_NAME) or {}
        if not tool_status:
            return {}

        conversion = tool_status.get('gripper_conversion') or {}
        if 'finger_rad' in conversion:
            joint_val = conversion['finger_rad']
        else:
            # Status without the driver's conversion block (e.g. before the first
            # pull_status): derive it from the raw servo angle.
            joint_val = self.metadata.actuator_to_urdf(tool_status.get('pos', 0.0))

        return {joint: joint_val for joint in self.metadata.tool_joints}
