#!/usr/bin/env python3
import time
import threading
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
from stretch4_body.subsystem.end_of_arm.end_of_arm_tools import home_dw4_joints

class NyuGripper(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll / NYU tendon gripper for version 4 of DexWrist
    """
    def __init__(self, name='nyu_gripper'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt.
        self.urdf_map={
            'wrist_yaw_joint':'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint':'wrist_roll'}

    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

        self.move_to('nyu_gripper', self.params['stow']['nyu_gripper'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            success = home_dw4_joints(self)
            success = success and self.motors['nyu_gripper'].home()
            self.status['is_homing'] = False
            return success

        if wait_on_completion:
            return _do_home()

        thread = threading.Thread(target=_do_home)
        thread.start()
        return None

    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])