#!/usr/bin/env python3
"""
ToolMetadata for the Bradito gripper (a tendon-driven parallel gripper; mechanically
identical to the NYU gripper, https://nyu-gripper.pages.dev).

Path B (nonlinear) from the stretch4_body README: a single Feetech servo winds a Kevlar
tendon to close the fingers and a return spring opens them, so the five unit types relate
to each other like this:

  command   0..100 'pct'.  0 = fully closed (the homing datum at the closed hardstop),
            100 = fully open. What move_to()/move_by() take.
  actuator  servo angle (rad), 0 at the closed hardstop up to range_deg[1].  Linear in pct.
  aperture  fingertip opening (m).  Measured linear in pct between the closed and open values.
  urdf      brd_finger_*_joint angle (rad).  aperture = 2 * finger_length * sin(urdf), so
            urdf <-> everything else is nonlinear (the asin below) -- hence Path B.
  normalized 0..1, derived by the base class from actuator_range.

Every number comes from robot_params (the `range_deg` / `gripper_conversion` keys of the
`bradito` device block in tool_params.yaml), so editing the YAML re-tunes the conversions
without touching this file.
"""

import math
from collections.abc import Callable

from stretch4_body.core.hello_utils import deg_to_rad
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.utils.tool_metadata import ToolConfigurationError, ToolMetadata

TOOL_NAME = "bradito"


class BraditoMetadata(ToolMetadata):
    @property
    def _params(self) -> dict:
        _, robot_params = RobotParams.get_params()
        params = robot_params.get(TOOL_NAME)
        if not params:
            raise ToolConfigurationError(
                f"Tool '{TOOL_NAME}' is not defined in robot_params. Is it installed in "
                "user_tools and selected with stretch_configure_tool?"
            )
        return params

    @property
    def _gripper_conversion(self) -> dict:
        return self._params["gripper_conversion"]

    @property
    def _finger_length_m(self) -> float:
        return float(self._gripper_conversion["finger_length_m"])

    @property
    def _aperture_open_m(self) -> float:
        return float(self._gripper_conversion["aperture_open_m"])

    @property
    def _aperture_closed_m(self) -> float:
        return float(self._gripper_conversion["aperture_closed_m"])

    # --- Identity ---

    @property
    def joint_name(self) -> str:
        """
        The Stretch Body device name, NOT a URDF joint. This is the key used for the
        `devices` entry in tool_params.yaml, for end_of_arm status, for motion params
        lookup, and for end_of_arm.move_to(<joint>, ...), so it has to stay 'bradito'.
        """
        return TOOL_NAME

    @property
    def tool_joints(self) -> list[str]:
        return ["brd_finger_left_joint", "brd_finger_right_joint"]

    @property
    def primary_joint(self) -> str:
        return "brd_finger_left_joint"

    @property
    def actuated_joints(self) -> list[str]:
        """Both fingers are driven off the one tendon, so the right finger mimics the left."""
        return ["brd_finger_left_joint"]

    @property
    def tool_links(self) -> list[str]:
        return ["brd_body_link", "brd_finger_left_link", "brd_finger_right_link"]

    @property
    def client_class(self) -> Callable:
        # Imported here, not at module scope, to avoid a circular import through
        # robot_client -> tool_metadata.
        from stretch4_body.core.robot_params import RobotParams as _RP

        module = _RP.import_user_tool_module(TOOL_NAME, "client", is_server=False)
        return lambda parent=None, ip_address=None: module.BraditoClient(
            self, parent=parent, ip_address=ip_address
        )

    @property
    def driver_class(self) -> type:
        # Imported inside the property (see the "Heads Up" note in
        # stretch4_body/subsystem/end_of_arm/README.md): tool.py imports this module, so
        # importing it at module scope would be circular.
        from stretch4_body.core.robot_params import RobotParams as _RP

        module = _RP.import_user_tool_module(TOOL_NAME, "tool", is_server=True)
        return module.BraditoGripper

    # --- Ranges ---

    @property
    def actuator_range(self) -> tuple[float, float]:
        """(closed, open) servo angle (radians), from the device's `range_deg`."""
        range_deg = self._params["range_deg"]
        return deg_to_rad(float(range_deg[0])), deg_to_rad(float(range_deg[1]))

    @property
    def command_range(self) -> tuple[float, float]:
        """(closed, open) in pct -- move_to()/move_by()'s own units."""
        return 0.0, 100.0

    @property
    def aperture_range(self) -> tuple[float, float]:
        return self._aperture_closed_m, self._aperture_open_m

    @property
    def poses(self) -> dict[str, float]:
        """Named command positions, in pct."""
        low, high = self.command_range
        return {"zero": 0.0, "close": low, "open": high, "mid": (low + high) / 2.0}

    @property
    def position_tolerance(self) -> float:
        """
        Arrival threshold in URDF units (rad). A user-supplied `position_tolerance` in
        tool_params.yaml wins; otherwise fall back to the base class's fraction of urdf_range.
        """
        user_value = self._params.get("position_tolerance")
        return float(user_value) if user_value is not None else super().position_tolerance

    # --- Conversions ---
    #
    # Only command <-> actuator and aperture <-> actuator are affine. The urdf edge carries
    # the sin/asin of the fingertip chord, so it is left out of _LINEAR_CONVERSIONS and given
    # a closed form in _analytic_gain below.

    _LINEAR_CONVERSIONS = ToolMetadata._LINEAR_CONVERSIONS | frozenset(
        {
            ("command", "actuator"),
            ("actuator", "command"),
            ("aperture", "actuator"),
            ("actuator", "aperture"),
        }
    )

    def command_to_actuator(self, command: float) -> float:
        act_low, act_high = self.actuator_range
        cmd_low, cmd_high = self.command_range
        return self._map_range(command, cmd_low, cmd_high, act_low, act_high)

    def actuator_to_command(self, actuator: float) -> float:
        act_low, act_high = self.actuator_range
        cmd_low, cmd_high = self.command_range
        return self._map_range(actuator, act_low, act_high, cmd_low, cmd_high)

    def actuator_to_aperture(self, actuator: float) -> float:
        """Servo angle (rad) -> fingertip opening (m). Measured linear in servo travel."""
        act_low, act_high = self.actuator_range
        return self._map_range(
            actuator, act_low, act_high, self._aperture_closed_m, self._aperture_open_m
        )

    def aperture_to_actuator(self, aperture: float) -> float:
        act_low, act_high = self.actuator_range
        return self._map_range(
            aperture, self._aperture_closed_m, self._aperture_open_m, act_low, act_high
        )

    def _aperture_to_finger_rad(self, aperture_m: float) -> float:
        """
        Half the angle subtended by the fingertip chord `aperture_m` on a circle of radius
        finger_length: the URDF angle of one finger.
        """
        ratio = aperture_m / (2.0 * self._finger_length_m)
        return math.asin(min(max(ratio, -1.0), 1.0))

    def _finger_rad_to_aperture(self, finger_rad: float) -> float:
        return 2.0 * self._finger_length_m * math.sin(finger_rad)

    def command_to_urdf(self, command: float) -> float:
        """pct -> finger joint angle (rad)."""
        return self._aperture_to_finger_rad(self.command_to_aperture(command))

    def urdf_to_command(self, urdf: float) -> float:
        """Finger joint angle (rad) -> pct."""
        return self.aperture_to_command(self._finger_rad_to_aperture(urdf))

    def _analytic_gain(self, frm: str, to: str, at: float) -> float | None:
        """
        Closed form for the nonlinear urdf edge, so velocities are exact rather than
        numerically differenced.

        aperture = 2*L*sin(urdf)  =>  d(aperture)/d(urdf) = 2*L*cos(urdf), and
        d(aperture)/d(command) is the constant `_map_range` ratio k, so
        d(command)/d(urdf) = 2*L*cos(urdf)/k.
        """
        cmd_low, cmd_high = self.command_range
        span = cmd_high - cmd_low
        if span == 0:
            return None
        k = (self._aperture_open_m - self._aperture_closed_m) / span  # d(aperture)/d(command)
        if k == 0:
            return None

        if (frm, to) == ("urdf", "command"):
            return 2.0 * self._finger_length_m * math.cos(at) / k
        if (frm, to) == ("command", "urdf"):
            # at is in command units; differentiate the asin at the matching aperture.
            aperture = self.command_to_aperture(at)
            cos_finger = math.cos(self._aperture_to_finger_rad(aperture))
            if cos_finger == 0:
                return None
            return k / (2.0 * self._finger_length_m * cos_finger)
        return None

    # --- Status ---

    def status_to_metadata(self, status: dict) -> dict:
        """
        Raw servo status -> the 'gripper_conversion' fields consumed by the ROS joint_state
        publisher, the self-collision model and the pose tools.
        """
        actuator = status.get("pos", 0.0)
        return {
            "aperture_m": self.actuator_to_aperture(actuator),
            "finger_rad": self.actuator_to_urdf(actuator),
            "finger_effort": status.get("effort", 0.0),
            # Time derivative of finger_rad, pushed through the Jacobian rather than through
            # the position conversion (which would be wrong across the asin).
            "finger_vel": self.actuator_to_urdf_velocity(status.get("vel", 0.0), actuator),
        }
