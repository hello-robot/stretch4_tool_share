#!/usr/bin/env python3
"""
The EndOfArm subclass for the Bradito tool: the 3-DOF DexWrist v4 plus the gripper servo.

`py_class_name`/`py_module_name` in tool_params.yaml point here. Stretch Body instantiates
this class as the `end_of_arm` subsystem (in the server's EndOfArmLoop, or directly in
Robot), so it is the place for stow ordering and the homing sequence -- not for the servo
driver itself, which lives in tool.py under the `devices` entry.
"""

import threading
import time

from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
from stretch4_body.subsystem.end_of_arm.end_of_arm_tools import home_dw4_joints

TOOL_NAME = 'bradito'


class Bradito(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll + Bradito tendon gripper, for version 4 of the DexWrist.
    """

    def __init__(self, name=TOOL_NAME):
        EndOfArm.__init__(self, name)

        # Maps URDF joint names to Stretch Body joint names, for CollisionMgmt.
        self.urdf_map = {
            'wrist_yaw_joint': 'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint': 'wrist_roll',
        }

    def stow(self):
        # Fold in wrist, then close the gripper.
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

        self.move_to(TOOL_NAME, self.params['stow'][TOOL_NAME])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            success = home_dw4_joints(self)
            # The gripper's own home() drives the tendon closed and finishes fully open.
            success = success and self.motors[TOOL_NAME].home()
            self.status['is_homing'] = False
            return success

        if wait_on_completion:
            return _do_home()

        thread = threading.Thread(target=_do_home)
        thread.start()
        return None

    def pre_stow(self, robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
