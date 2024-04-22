import rospy
import actionlib
from brl_pxh_api.msg import (
        ConstPoseAction,
        ConstPoseGoal,
        CartTrajAction,
        CartTrajGoal,
        JointRadAction,
        JointRadGoal,
        JointGroupRadsAction,
        JointGroupRadsGoal,
        GripperPressureAction,
        GripperPressureGoal,
        GripperMotionAction,
        GripperMotionGoal)

class BrlPxhClient:
    
    def __init__(self):
        self._init_clients()
        self._wait_for_servers() 

    def brl_go_to_home_pose(self, moving_time=1):
        goal = self._init_const_pose_goal(moving_time)
        self.client_home_pose.send_goal(goal)
        self.client_home_pose.wait_for_result()
        self._log_action_result(
                "go_to_home_pose", 
                self.client_home_pose)

    def brl_go_to_sleep_pose(self, moving_time=1):
        goal = self._init_const_pose_goal(moving_time)
        self.client_sleep_pose.send_goal(goal)
        self.client_sleep_pose.wait_for_result()
        self._log_action_result(
                "go_to_sleep_pose", 
                self.client_sleep_pose)

    def brl_set_ee_cartesian_trajectory(
            self,
            x=0,
            z=0,
            roll=0,
            pitch=0,
            moving_time=1,
            wp_moving_time=0.2,
            wp_accel_time=0.1,
            wp_period=0.05):
        goal = CartTrajGoal()
        goal.x = x
        goal.z = z
        goal.roll = roll
        goal.pitch = pitch
        goal.moving_time = moving_time
        goal.wp_moving_time = wp_moving_time
        goal.wp_accel_time = wp_accel_time
        goal.wp_period = wp_period
        self.client_cart_traj.send_goal(goal)
        self.client_cart_traj.wait_for_result()
        self._log_action_result(
                "set_ee_cartesian_trajectory",
                self.client_cart_traj)

    def brl_set_single_joint_position(
            self,
            joint_name,
            position,
            moving_time=1,
            accel_time=1,
            blocking=True):
        goal = JointRadGoal()
        goal.joint_name = joint_name
        goal.position = position
        goal.moving_time = moving_time
        goal.accel_time = accel_time
        goal.blocking = blocking
        self.client_set_single_joint.send_goal(goal)
        self.client_set_single_joint.wait_for_result()
        self._log_action_result(
                "set_single_joint_position",
                self.client_set_single_joint)

    # joint order: waist, shoulder, elbow, wrist_angle
    # cf. the output of `rosparam get /px100/xs_sdk/motor_configs`
    def brl_set_joint_positions(
            self,
            joint_positions,
            moving_time=1,
            accel_time=1,
            blocking=True):
        goal = JointGroupRadsGoal()
        goal.joint_positions = joint_positions
        goal.moving_time = moving_time
        goal.accel_time = accel_time
        goal.blocking = blocking
        self.client_set_joints.send_goal(goal)
        self.client_set_joints.wait_for_result()
        self._log_action_result(
                "set_joints",
                self.client_set_joints)
   
    def brl_set_gripper_pressure(
            self,
            pressure):
        goal = GripperPressureGoal()
        goal.pressure = pressure
        self.client_set_gripper_pressure.send_goal(goal)
        self.client_set_gripper_pressure.wait_for_result()
        self._log_action_result(
                "set_gripper_pressure",
                self.client_set_gripper_pressure)

    def brl_open_gripper(
            self,
            delay=1.0):
        goal = GripperMotionGoal()
        goal.delay = delay
        self.client_open_gripper.send_goal(goal)
        self.client_open_gripper.wait_for_result()
        self._log_action_result(
                "open_gripper",
                self.client_open_gripper)

    def brl_close_gripper(
            self,
            delay=1.0):
        goal = GripperMotionGoal()
        goal.delay = delay
        self.client_close_gripper.send_goal(goal)
        self.client_close_gripper.wait_for_result()
        self._log_action_result(
                "close_gripper",
                self.client_close_gripper)

    def _init_const_pose_goal(self, moving_time):
        goal = ConstPoseGoal()
        goal.moving_time = moving_time
        return goal

    def _log_action_result(self, action, client):
        log = f'Action {action} '
        log += ('processed. Might have failed; '
                'check log of terminal running Interbotix\'s '
                'xsarm_control.launch file.')
        log += f'{client.get_result().log}'
        rospy.loginfo(log)

    def _init_clients(self):
        self.client_home_pose = actionlib.SimpleActionClient(
                'server_home_pose', 
                ConstPoseAction)
        self.client_sleep_pose = actionlib.SimpleActionClient(
                'server_sleep_pose', 
                ConstPoseAction)
        self.client_cart_traj = actionlib.SimpleActionClient(
                'server_cart_traj', 
                CartTrajAction)
        self.client_set_single_joint = actionlib.SimpleActionClient(
                'server_set_single_joint', 
                JointRadAction)
        self.client_set_joints = actionlib.SimpleActionClient(
                'server_set_joints', 
                JointGroupRadsAction)
        self.client_set_gripper_pressure = actionlib.SimpleActionClient(
                'server_set_gripper_pressure', 
                GripperPressureAction)
        self.client_open_gripper = actionlib.SimpleActionClient(
                'server_open_gripper', 
                GripperMotionAction)
        self.client_close_gripper = actionlib.SimpleActionClient(
                'server_close_gripper', 
                GripperMotionAction)

    def _wait_for_servers(self):
        self.client_home_pose.wait_for_server()
        self.client_sleep_pose.wait_for_server()
        self.client_cart_traj.wait_for_server()
        self.client_set_single_joint.wait_for_server()
        self.client_set_joints.wait_for_server()
        self.client_set_gripper_pressure.wait_for_server()
        self.client_open_gripper.wait_for_server()
        self.client_close_gripper.wait_for_server()

