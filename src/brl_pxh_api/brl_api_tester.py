#! /usr/bin/env python3
import rospy

from brl_pxh_api.brl_pxh_client import BrlPxhClient 

class BrlPxhApiTester:

    def __init__(self):
        self.api_client = BrlPxhClient()

    def _test_endpoints(self):
        self.api_client.brl_go_to_home_pose()
        self.api_client.brl_set_ee_pose_components(x=0.2, z=0.1)
        self.api_client.brl_go_to_home_pose()
        self.api_client.brl_set_single_joint_position(
                joint_name="waist",
                position=0.2)
        self.api_client.brl_set_ee_cartesian_trajectory(z=-0.1)
        self.api_client.brl_set_ee_cartesian_trajectory(x=-0.1)
        self.api_client.brl_set_ee_cartesian_trajectory(z=0.1)
        self.api_client.brl_set_ee_cartesian_trajectory(x=0.1)
        self.api_client.brl_set_single_joint_position(
                joint_name="waist",
                position=0)

        # ------------------------------------------------------------
        # Uncomment the below to test the brl_set_gripper_pressure
        # endpoint with a value of 0. This causes the gripper to open
        # less than its full range of motion. I believe this is just
        # part of how the PX-100 sets the gripper strength.
        # self.api_client.brl_set_gripper_pressure(0)
        # self.api_client.brl_open_gripper()
        # self.api_client.brl_close_gripper()
        # ------------------------------------------------------------

        self.api_client.brl_set_gripper_pressure(0.5)
        self.api_client.brl_open_gripper()
        self.api_client.brl_close_gripper()
        # ------------------------------------------------------------
        
        # ------------------------------------------------------------
        # Uncomment the below to test the brl_set_joint_positions
        # endpoint.
        # self.api_client.brl_set_joint_positions(
        #         joint_positions=[0.2, 0.2, 0.2, 0.2])
        # ------------------------------------------------------------
        
        # ------------------------------------------------------------
        # Uncomment to see delayed open_gripper. The delay causes the
        # command after open_gripper to be delayed
        # by 2 seconds.
        # self.api_client.brl_open_gripper(delay=2.0)
        # ------------------------------------------------------------

        # ------------------------------------------------------------
        # Uncomment to see delayed close_gripper. The delay causes the
        # command after open_gripper to be delayed
        # by 2 seconds.

        # self.api_client.brl_close_gripper(delay=2.0)
        # ------------------------------------------------------------

        self.api_client.brl_go_to_sleep_pose()

    def run(self):
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #    self._test_endpoints()
        #    rate.sleep()
        self._test_endpoints()

if __name__ == '__main__':
    rospy.init_node('brl_api_tester')
    BrlPxhApiTester().run()
    
