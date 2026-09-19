import math
from functools import partial
from typing import Callable

from stretch4_body.core.hello_utils import deg_to_rad
from stretch4_body.utils.tool_metadata import ToolMetadata


class NyuGripperMetadata(ToolMetadata):

    @property
    def tool_name(self) -> str:
        return "nyu_gripper"


    @property
    def tool_joints(self) -> list[str]:
        return ['ng_finger_left_joint', 'ng_finger_right_joint']

    @property
    def tool_links(self) -> list[str]:
        return ['ng_finger_left_link', 'ng_finger_right_link', 'ng_body_link']

    @property
    def client_class(self) -> Callable:
        from stretch4_body.robot.robot_client import ToolJointClient

        return partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        from nyu_gripper_driver import NyuGripper

        return NyuGripper

    @property
    def actuator_range(self) -> tuple[float, float]:
        return (0.0, deg_to_rad(187.0))  # Servo travel: ~0.0 to 3.2638 rad

    @property
    def command_range(self) -> tuple[float, float]:
        return (0.0, 100.0)  # Commanded percentage

    # Kinematics: finger_length = 0.11205 m, aperture_open = 0.145 m
    def urdf_to_command(self, urdf: float) -> float:
        aperture_m = 2.0 * 0.11205 * math.sin(urdf)
        return 100.0 * (aperture_m / 0.145)

    def command_to_urdf(self, command: float) -> float:
        pct = min(max(command, 0.0), 100.0)
        aperture_m = 0.145 * (pct / 100.0)
        val = aperture_m / (2.0 * 0.11205)
        return math.asin(min(max(val, -1.0), 1.0))

    def command_to_actuator(self, command: float) -> float:
        return deg_to_rad(187.0) * (command / 100.0)

    def actuator_to_command(self, actuator: float) -> float:
        return 100.0 * actuator / deg_to_rad(187.0)

    def aperture_to_actuator(self, aperture: float) -> float:
        cmd = 100.0 * (aperture / 0.145)
        return self.command_to_actuator(cmd)

    def actuator_to_aperture(self, actuator: float) -> float:
        cmd = self.actuator_to_command(actuator)
        return 0.145 * (cmd / 100.0)

    def status_to_metadata(self, status: dict) -> dict:
        return status.get('gripper_conversion', {
            'aperture_m': 0.0,
            'finger_rad': 0.0,
            'finger_vel': 0.0,
            'finger_effort': 0.0,
        })
