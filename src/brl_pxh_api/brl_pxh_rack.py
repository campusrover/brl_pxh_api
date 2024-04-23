#! /usr/bin/env python3
import sys

import rospy
import actionlib
from interbotix_xs_modules.arm import InterbotixManipulatorXS

from brl_pxh_api.msg import (
        CartTrajAction,
        CartTrajResult,
        ConstPoseAction,
        ConstPoseResult,
        EePoseCompAction,
        EePoseCompResult,
        GripperMotionAction,
        GripperMotionResult,
        GripperPressureAction,
        GripperPressureResult,
        JointGroupRadsAction,
        JointGroupRadsResult,
        JointRadAction,
        JointRadResult)

sys.path.insert(0, '/home/brl/interbotix_ws/')

class BrlPxhRack:
    
    def __init__(self):
        self._init_xs_api()
        self._init_servers()
        self._start_servers()
    
    def do_go_to_home_pose(self, goal):
        gmt = goal.moving_time
        self.xs_api.arm.go_to_home_pose(moving_time=gmt)
        # rospy.sleep(gmt)
        self._set_const_pose_result("home", gmt)

    def do_go_to_sleep_pose(self, goal):
        gmt = goal.moving_time
        self.xs_api.arm.go_to_sleep_pose(moving_time=gmt)
        # rospy.sleep(gmt)
        self._set_const_pose_result("sleep", gmt)

    def do_set_ee_cartesian_trajectory(self, goal):
        self.xs_api.arm.set_ee_cartesian_trajectory(
                x=goal.x,
                z=goal.z,
                roll=goal.roll,
                pitch=goal.pitch,
                moving_time=goal.moving_time,
                wp_moving_time=goal.wp_moving_time,
                wp_accel_time=goal.wp_accel_time,
                wp_period=goal.wp_period)
        result = CartTrajResult()
        result.log = (
                f'Tried to move gripper to the point at a '
                f'(x: {goal.x}, z: {goal.z}) offset from the '
                f'`px_100/base_link` frame\'s origin, with a rotation '
                f'of (roll: {goal.roll}, pitch: {goal.pitch}) '
                f'relative to that of the said frame, in '
                f'{goal.moving_time} seconds.')
        result.success = True
        self.server_cart_traj.set_succeeded(result)

    def do_set_single_joint_position(self, goal):
        self.xs_api.arm.set_single_joint_position(
                joint_name=goal.joint_name,
                position=goal.position,
                moving_time=goal.moving_time,
                accel_time=goal.accel_time,
                blocking=goal.blocking)
        result = JointRadResult()
        result.log = (
                f'Tried to move joint {goal.joint_name} by '
                f'{goal.position} radians in {goal.moving_time} '
                f'seconds.')
        result.success = True
        self.server_set_single_joint.set_succeeded(result)

    def do_set_joint_positions(self, goal):
        self.xs_api.arm.set_joint_positions(
                joint_positions=goal.joint_positions,
                moving_time=goal.moving_time,
                accel_time=goal.accel_time,
                blocking=goal.blocking)
        result = JointGroupRadsResult()
        result.log = (
                f'Tried to move joints by the given radians: '
                f'{*goal.joint_positions,}, in {goal.moving_time} '
                f'seconds.')
        result.success = True
        self.server_set_joints.set_succeeded(result)

    def do_set_gripper_pressure(self, goal):
        self.xs_api.gripper.set_pressure(goal.pressure)
        result = GripperPressureResult()
        result.log = (
                f'Tried to set gripper pressure to {goal.pressure}.')
        result.success = True
        self.server_set_gripper_pressure.set_succeeded(result)

    def do_open_gripper(self, goal):
        self.xs_api.gripper.open(delay=goal.delay)
        result = GripperMotionResult()
        result.log = (
                f'Tried to open gripper with a delay of {goal.delay}.')
        result.success = True
        self.server_open_gripper.set_succeeded(result)

    def do_close_gripper(self, goal):
        self.xs_api.gripper.close(delay=goal.delay)
        result = GripperMotionResult()
        result.log = (
                f'Tried to close gripper with a delay of {goal.delay}.')
        result.success = True
        self.server_close_gripper.set_succeeded(result)

    def do_set_ee_pose_components(self, goal):
        self.xs_api.arm.set_ee_pose_components(
                x=goal.x,
                z=goal.z,
                roll=goal.roll,
                pitch=goal.pitch,
                execute=goal.execute,
                moving_time=goal.moving_time,
                accel_time=goal.accel_time,
                blocking=goal.blocking
                )
        result = EePoseCompResult()
        result.log = (
                f'Tried to move gripper to the point at a '
                f'(x: {goal.x}, z: {goal.z}) offset from the '
                f'`px_100/base_link` frame\'s origin, with a rotation '
                f'of (roll: {goal.roll}, pitch: {goal.pitch}) '
                f'relative to that of the said frame, in '
                f'{goal.moving_time} seconds.')
        result.success = True
        self.server_ee_pose_comp.set_succeeded(result)

    def _set_const_pose_result(self, const_pose, goal_movement_time):
        result = ConstPoseResult()
        result.log = (
                f'Went to {const_pose} pose in '
                f'{goal_movement_time} seconds.')
        result.success = True
        if const_pose == "home":
            self.server_home_pose.set_succeeded(result)
        else:
            self.server_sleep_pose.set_succeeded(result)

    def _init_xs_api(self):
        self.xs_api = InterbotixManipulatorXS(
                "px100", 
                "arm", 
                "gripper", 
                init_node=False)

    def _init_servers(self):
        self.server_home_pose = actionlib.SimpleActionServer(
                'server_home_pose', 
                ConstPoseAction, 
                self.do_go_to_home_pose,
                False)
        self.server_sleep_pose = actionlib.SimpleActionServer(
                'server_sleep_pose', 
                ConstPoseAction, 
                self.do_go_to_sleep_pose,
                False)
        self.server_cart_traj = actionlib.SimpleActionServer(
                'server_cart_traj', 
                CartTrajAction, 
                self.do_set_ee_cartesian_trajectory,
                False)
        self.server_set_single_joint = actionlib.SimpleActionServer(
                'server_set_single_joint', 
                JointRadAction, 
                self.do_set_single_joint_position,
                False)
        self.server_set_joints = actionlib.SimpleActionServer(
                'server_set_joints', 
                JointGroupRadsAction, 
                self.do_set_joint_positions,
                False)
        self.server_set_gripper_pressure = actionlib.SimpleActionServer(
                'server_set_gripper_pressure', 
                GripperPressureAction, 
                self.do_set_gripper_pressure,
                False)
        self.server_open_gripper = actionlib.SimpleActionServer(
                'server_open_gripper', 
                GripperMotionAction, 
                self.do_open_gripper,
                False)
        self.server_close_gripper = actionlib.SimpleActionServer(
                'server_close_gripper', 
                GripperMotionAction, 
                self.do_close_gripper,
                False)
        self.server_ee_pose_comp = actionlib.SimpleActionServer(
                'server_ee_pose_comp', 
                EePoseCompAction, 
                self.do_set_ee_pose_components,
                False)

    def _start_servers(self):
        self.server_home_pose.start()
        self.server_sleep_pose.start()
        self.server_cart_traj.start()
        self.server_set_single_joint.start()
        self.server_set_joints.start()
        self.server_set_gripper_pressure.start()
        self.server_open_gripper.start()
        self.server_close_gripper.start()
        self.server_ee_pose_comp.start()

if __name__ == '__main__':
    rospy.init_node('brl_pxh_rack')
    BrlPxhRack()
    rospy.spin()

