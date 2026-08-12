#!/usr/bin/env python3
import math
from stretch4_body.core.robot_params import RobotParams

# def nyu_gripper_servo_rad_to_mm(servo_rad, params):
#     """
#     Convert custom gripper servo angle (in radians) to gap width (in mm).
#     """
#     range_rad = math.radians(params.get('range_deg', [0.0, 100.0])[1] - params.get('range_deg', [0.0, 100.0])[0])
#     range_mm = params.get('range_mm', 80.0)
#     if range_rad == 0:
#         return 0.0
#     return (servo_rad / range_rad) * range_mm


# def nyu_gripper_mm_to_servo_rad(x_mm, params):
#     """
#     Convert custom gripper gap width (in mm) to servo angle (in radians).
#     """
#     range_rad = math.radians(params.get('range_deg', [0.0, 100.0])[1] - params.get('range_deg', [0.0, 100.0])[0])
#     range_mm = params.get('range_mm', 80.0)
#     if range_mm == 0:
#         return 0.0
#     return (x_mm / range_mm) * range_rad


# def nyu_gripper_pos_mm_to_urdf_m(pos_mm, params):
#     """
#     Convert custom gripper finger aperture (in mm) to URDF finger joint value (in meters).
#     """
#     range_mm = params.get('range_mm', 80.0)
#     pct = pos_mm / range_mm if range_mm != 0 else 0.0
#     lower = -0.04
#     upper = 0.0
#     return upper + pct * (lower - upper)

def nyu_gripper_urdf_to_subsystem(position, params):
    """
    Convert URDF finger joint value (in meters/radians) to custom gripper subsystem units.
    """
    
    _, robot_params = RobotParams.get_params()
    gc = robot_params.get('nyu_gripper', {}).get('gripper_conversion', {})
    aperture_open_m = gc.get('aperture_open_m', 0.08)
    aperture_closed_m = gc.get('aperture_closed_m', 0.0)
    finger_length_m = gc.get('finger_length_m', 0.10)
    aperture_m = 2.0 * finger_length_m * math.sin(position)  # position is finger_rad
    return 100.0 * (aperture_m - aperture_closed_m) / (aperture_open_m - aperture_closed_m)
