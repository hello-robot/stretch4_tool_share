#!/usr/bin/env python3
class NyuGripperCommandGroup:
    """
    ROS Command Group template for nyu_gripper.
    Maps ROS JointState and trajectory commands to the custom python driver.
    """
    def __init__(self, robot, node=None):
        self.robot = robot
        self.node = node
        self.joint_name = "nyu_gripper_joint"

    def get_joint_state(self):
        """
        Returns current state of the joint to publish on ROS joint_states.
        """
        status = self.robot.end_of_arm.status.get('nyu_gripper', {})
        return {
            'name': self.joint_name,
            'pos': status.get('pos', 0.0),
            'vel': status.get('vel', 0.0),
            'effort': status.get('effort', 0.0)
        }

    def command_joint(self, position, velocity=None, acceleration=None):
        """
        Applies target command to the joint driver.
        """
        self.robot.end_of_arm.move_to('nyu_gripper', position)
