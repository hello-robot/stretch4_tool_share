#!/usr/bin/env python3
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.subsystem.end_of_arm.gripper_conversion import get_finger_joint_limits

class NyuGripperCollision:
    """
    Collision joint state mapping for nyu_gripper.
    Maps custom joint state values to Mujoco visualizer joints.
    """
    def __init__(self, robot=None):
        self.robot = robot
        self._finger_limits = None

    def get_mujoco_joints(self, state):
        """
        Given the raw robot status dictionary,
        return a dictionary mapping Mujoco joint names to their target positions.
        """
        eoa = state.get('end_of_arm', {})

        # Look for the custom tool within the end of arm status
        tool_status = eoa.get('nyu_gripper') or {}

        # Device params for the gripper device named after the tool are merged
        # into the tool's top-level params block
        _, robot_params = RobotParams.get_params()
        tool_params = robot_params.get('nyu_gripper', {})

        # Drivers that publish a gripper_conversion status report the finger
        # angle directly; parallel-jaw drivers report an aperture in mm
        conversion = tool_status.get('gripper_conversion') or {}
        if 'finger_rad' in conversion:
            joint_val = conversion['finger_rad']
        else:
            if self._finger_limits is None:
                # Cached: this regenerates the robot URDF, too slow to call per cycle
                self._finger_limits = get_finger_joint_limits()
            lower, upper = self._finger_limits
            range_mm = tool_params.get('range_mm', 80.0)
            pct = tool_status.get('pos_mm', 0.0) / range_mm if range_mm else 0.0
            joint_val = upper + pct * (lower - upper)

        return {
            'ng_finger_left_joint': joint_val,
            'ng_finger_right_joint': joint_val
        }
