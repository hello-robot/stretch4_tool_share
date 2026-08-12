#!/usr/bin/env python3
from stretch4_body.robot.robot_client import EndOfArmClient, WristJointClient

class NYUGripperClient(WristJointClient):
    """ Client for the NYU tendon gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='nyu_gripper', parent=parent, ip_address=ip_address)
        self.pct_max_open = 100.0
        self.poses = {'zero': 0.0,
                      'open': self.pct_max_open,
                      'close': 0.0}
        self.status['gripper_conversion'] = {'aperture_m': 0.0,
                                             'finger_rad': 0.0,
                                             'finger_effort': 0.0,
                                             'finger_vel': 0.0}

    def home(self, end_pos=100.0, wait_on_completion=True, timeout=20):
        # Match the direct API: finish homing at fully open (pct units).
        return WristJointClient.home(self, end_pos=end_pos, wait_on_completion=wait_on_completion, timeout=timeout)

class NyuGripper_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll / NYU tendon gripper for version 4 of DexWrist
    """
    def __init__(self,parent=None):
        EndOfArmClient.__init__(self,name='nyu_gripper',parent=parent)

