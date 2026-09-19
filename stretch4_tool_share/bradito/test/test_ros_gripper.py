#!/usr/bin/env python3
"""
ROS 2 acceptance test for a custom end-of-arm tool (defaults to bradito).

Checks, in order:
  1. the tool_info.* parameters stretch_driver declares from ToolMetadata
  2. that the tool's URDF joints appear in /joint_states
  3. FollowJointTrajectory goals, on each name GripperCommandGroup should accept
     -- plus the name stretch4_web_teleop sends, which it should NOT accept
  4. /joint_vel (JointJog) velocity jogging of the tool

Prerequisites: stretch_body_server running, robot homed, and
    ros2 launch stretch_core stretch_driver.launch.py
already up in another terminal.

    python3 test_ros_gripper.py [--tool bradito]
"""

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION = '/follow_joint_trajectory'
PASS, FAIL, WARN = '\033[32mPASS\033[0m', '\033[31mFAIL\033[0m', '\033[33mWARN\033[0m'


class ToolTester(Node):
    def __init__(self, tool):
        super().__init__('tool_ros_test')
        self.tool = tool
        self.joint_state = {}
        self.results = []
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.jog_pub = self.create_publisher(JointJog, '/joint_vel', 10)
        self.action = ActionClient(self, FollowJointTrajectory, ACTION)
        self.params = self.create_client(GetParameters, '/stretch_driver/get_parameters')
        self.set_params = self.create_client(SetParameters, '/stretch_driver/set_parameters')
        self.original_mode = None

    def _on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_state[name] = pos

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def record(self, status, label, detail=''):
        self.results.append((status, label))
        print(f"  [{status}] {label}" + (f" -- {detail}" if detail else ''))

    # --- driver mode ----------------------------------------------------------
    # /stretch_driver:mode is a parameter, not a service. The trajectory server only
    # accepts goals in 'position'/'navigation'; JointJog only in 'velocity'/'navigation'.
    # Web teleop flips it as you use the UI, so read it, set what each section needs,
    # and put it back at the end.
    def get_mode(self):
        if not self.params.wait_for_service(timeout_sec=10.0):
            return None
        fut = self.params.call_async(GetParameters.Request(names=['mode']))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        return res.values[0].string_value if res and res.values else None

    def set_mode(self, mode):
        if self.get_mode() == mode:
            return True
        if not self.set_params.wait_for_service(timeout_sec=10.0):
            return False
        param = Parameter(name='mode', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=mode))
        fut = self.set_params.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        self.spin(1.0)
        res = fut.result()
        return bool(res and res.results and res.results[0].successful)

    # --- 1. tool_info parameters ---------------------------------------------
    def check_tool_info(self):
        print("\n1. tool_info parameters (declared from ToolMetadata)")
        if not self.params.wait_for_service(timeout_sec=10.0):
            self.record(FAIL, 'stretch_driver parameter service', 'is stretch_driver running?')
            return None
        names = ['tool_info.name', 'tool_info.is_actuated', 'tool_info.tool_joints',
                 'tool_info.urdf_range', 'tool_info.aperture_range', 'tool_info.position_tolerance']
        req = GetParameters.Request(names=names)
        fut = self.params.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            self.record(FAIL, 'read tool_info parameters')
            return None
        # rcl_interfaces/ParameterType: 1 BOOL, 3 DOUBLE, 4 STRING,
        # 8 DOUBLE_ARRAY, 9 STRING_ARRAY
        readers = {
            1: lambda pv: pv.bool_value,
            3: lambda pv: pv.double_value,
            4: lambda pv: pv.string_value,
            8: lambda pv: list(pv.double_array_value),
            9: lambda pv: list(pv.string_array_value),
        }
        values = {}
        for name, pv in zip(names, fut.result().values):
            reader = readers.get(pv.type)
            values[name] = reader(pv) if reader else None
        tool_joints = values.get('tool_info.tool_joints') or []
        self.record(PASS if values.get('tool_info.name') == self.tool else FAIL,
                    f"tool_info.name == '{self.tool}'", str(values.get('tool_info.name')))
        self.record(PASS if values.get('tool_info.is_actuated') else FAIL,
                    'tool_info.is_actuated is true')
        self.record(PASS if tool_joints else FAIL, 'tool_info.tool_joints non-empty', str(tool_joints))
        for k in ('tool_info.urdf_range', 'tool_info.aperture_range', 'tool_info.position_tolerance'):
            self.record(PASS if values.get(k) else FAIL, f'{k} declared', str(values.get(k)))
        return tool_joints

    # --- 2. joint_states ------------------------------------------------------
    def check_joint_states(self, tool_joints):
        print("\n2. /joint_states")
        self.spin(3.0)
        if not self.joint_state:
            self.record(FAIL, '/joint_states publishing')
            return
        self.record(PASS, '/joint_states publishing', f'{len(self.joint_state)} joints')
        for j in tool_joints:
            self.record(PASS if j in self.joint_state else FAIL, f"'{j}' published",
                        f"{self.joint_state.get(j, float('nan')):.4f} rad" if j in self.joint_state else '')
        self.record(PASS if 'gripper_joint' in self.joint_state else WARN,
                    "'gripper_joint' alias published")

    # --- 3. trajectory goals --------------------------------------------------
    def send_goal(self, joint, position, watch, secs=3):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=secs)
        goal.trajectory.points = [point]

        before = self.joint_state.get(watch)
        fut = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None, before, before
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=30.0)
        code = result_fut.result().result.error_code
        self.spin(1.0)
        return code, before, self.joint_state.get(watch)

    def check_trajectories(self, tool_joints):
        print("\n3. FollowJointTrajectory")
        self.record(PASS if self.set_mode('position') else FAIL,
                    "driver in 'position' mode", 'trajectory goals need position/navigation')
        if not self.action.wait_for_server(timeout_sec=15.0):
            self.record(FAIL, f'action server {ACTION} available')
            return
        self.record(PASS, f'action server {ACTION} available')
        watch = tool_joints[0]
        lo, hi = 0.15, 0.50

        for name in (tool_joints[0], 'gripper_joint'):
            target = hi if abs((self.joint_state.get(watch) or 0) - lo) < 0.05 else lo
            code, before, after = self.send_goal(name, target, watch)
            moved = before is not None and after is not None and abs(after - before) > 0.02
            detail = f'{before:.4f} -> {after:.4f} rad (goal {target}), code={code}'
            self.record(PASS if (code == 0 and moved) else FAIL, f"goal on '{name}' moves the tool", detail)

        # The legacy SG4-only joint name. Current stretch4_web_teleop sends the
        # tool-agnostic 'gripper_joint' for the button pad, but its voice control
        # (executeJointMove.ts) still sends this one, so it is worth probing.
        code, before, after = self.send_goal('stretch_gripper_joint', 0.30, watch)
        moved = before is not None and after is not None and abs(after - before) > 0.02
        if code is None:
            self.record(PASS, "legacy 'stretch_gripper_joint' goal rejected", 'correctly refused')
        elif not moved:
            self.record(WARN, "legacy 'stretch_gripper_joint' goal is a silent no-op",
                        f'code={code} but position unchanged ({after:.4f} rad) -- callers still using '
                        'this SG4-only name (e.g. web teleop voice control) get a false success')
        else:
            self.record(PASS, "legacy 'stretch_gripper_joint' goal moves the tool")

    # --- 4. velocity jog ------------------------------------------------------
    def check_joint_vel(self, tool_joints):
        print("\n4. /joint_vel (JointJog)")
        self.record(PASS if self.set_mode('velocity') else FAIL,
                    "driver in 'velocity' mode", 'JointJog needs velocity/navigation')
        watch = tool_joints[0]
        before = self.joint_state.get(watch)
        msg = JointJog()
        msg.joint_names = [f'{self.tool}_joint']
        msg.velocities = [-0.20]        # URDF rad/s, closing
        msg.duration = 0.5
        for _ in range(6):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.jog_pub.publish(msg)
            self.spin(0.25)
        self.spin(2.0)
        after = self.joint_state.get(watch)
        moved = before is not None and after is not None and abs(after - before) > 0.02
        self.record(PASS if moved else FAIL, f"jog '{self.tool}_joint' moves the tool",
                    f'{before:.4f} -> {after:.4f} rad')

    def summary(self):
        failed = [l for s, l in self.results if s == FAIL]
        warned = [l for s, l in self.results if s == WARN]
        print(f"\n{'='*70}\n{len(self.results) - len(failed) - len(warned)} passed, "
              f"{len(warned)} warned, {len(failed)} failed")
        for l in warned: print(f"  {WARN} {l}")
        for l in failed: print(f"  {FAIL} {l}")
        return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tool', default='bradito', help='tool name (default: bradito)')
    args = ap.parse_args()

    rclpy.init()
    node = ToolTester(args.tool)
    try:
        print(f"Testing ROS 2 integration for tool '{args.tool}'")
        node.original_mode = node.get_mode()
        print(f"driver mode on entry: {node.original_mode}")
        tool_joints = node.check_tool_info()
        if not tool_joints:
            print("\nCannot continue without tool_info.tool_joints.")
            return node.summary()
        node.check_joint_states(tool_joints)
        node.check_trajectories(tool_joints)
        node.check_joint_vel(tool_joints)
        return node.summary()
    finally:
        if node.original_mode:
            node.set_mode(node.original_mode)
            print(f"driver mode restored to: {node.get_mode()}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
