#!/usr/bin/env python
import rospy

from actionlib.simple_action_client import SimpleActionClient
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._Controller import Controller
from giskard_msgs.msg._MoveAction import MoveAction
from giskard_msgs.msg._MoveCmd import MoveCmd
from giskard_msgs.msg._MoveGoal import MoveGoal
from sensor_msgs.msg._JointState import JointState
import numpy as np

class Test(object):
    def __init__(self, action_server_name):
        # action server
        self.client = SimpleActionClient(action_server_name, MoveAction)
        self.client.wait_for_server()
        self.joint_names = rospy.wait_for_message('/whole_body_controller/state', JointTrajectoryControllerState).joint_names

    def send_cart_goal(self, goal_pose):
        goal = MoveGoal()
        goal.type = MoveGoal.PLAN_AND_EXECUTE

        # translaiton
        controller = Controller()
        controller.type = Controller.TRANSLATION_3D
        controller.tip_link = 'gripper_tool_frame'
        controller.root_link = 'base_footprint'

        controller.goal_pose = goal_pose

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.05
        goal.cmd_seq.append(MoveCmd())
        goal.cmd_seq[-1].controllers.append(controller)

        # rotation
        controller = Controller()
        controller.type = Controller.ROTATION_3D
        controller.tip_link = 'gripper_tool_frame'
        controller.root_link = 'base_footprint'

        controller.goal_pose = goal_pose

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.2
        goal.cmd_seq.append(MoveCmd())
        goal.cmd_seq[-1].controllers.append(controller)

        self.client.send_goal(goal)
        result = self.client.wait_for_result(rospy.Duration(10))
        print('finished in 10s?: {}'.format(result))

    def send_rnd_joint_goal(self):
        goal = MoveGoal()
        goal.type = MoveGoal.PLAN_AND_EXECUTE

        # translation
        controller = Controller()
        controller.type = Controller.JOINT
        controller.tip_link = 'gripper_tool_frame'
        controller.root_link = 'base_footprint'

        for i, joint_name in enumerate(self.joint_names):
            controller.goal_state.name.append(joint_name)
            # controller.goal_state.position.append(0)
            controller.goal_state.position.append(np.random.random()-0.5)

        controller.p_gain = 3
        controller.enable_error_threshold = True
        controller.threshold_value = 0.05
        controller.weight = 1
        goal.cmd_seq.append(MoveCmd())
        goal.cmd_seq[-1].controllers.append(controller)

        self.client.send_goal(goal)
        result = self.client.wait_for_result()
        final_js = rospy.wait_for_message('/whole_body_controller/state', JointTrajectoryControllerState) # type: JointTrajectoryControllerState
        asdf = {}
        for i, joint_name in enumerate(final_js.joint_names):
            asdf[joint_name] = final_js.actual.positions[i]
        for i, joint_name in enumerate(controller.goal_state.name):
            print('{} real:{} | exp:{}'.format(joint_name, asdf[joint_name], controller.goal_state.position[i]))
        print('finished in 10s?: {}'.format(result))


if __name__ == '__main__':
    rospy.init_node('donbot_test_movements')

    test = Test(action_server_name='/qp_controller/command')

    test.send_rnd_joint_goal()

    # goal = PoseStamped()
    # goal.header.frame_id = 'gripper_tool_frame'
    # goal.pose.position.x = 0.1
    # goal.pose.position.z = -0.2
    # goal.pose.orientation.w = 1.0
    # test.send_cart_goal(goal)
    #
    # goal = PoseStamped()
    # goal.header.frame_id = 'gripper_tool_frame'
    # goal.pose.position = Point(0,0,0)
    # goal.pose.orientation = Quaternion(0.0 , 0.0, 0.70710678, 0.70710678)
    # test.send_cart_goal(goal)
    #
    # goal = PoseStamped()
    # goal.header.frame_id = 'gripper_tool_frame'
    # goal.pose.position = Point(0,-0.1,0)
    # goal.pose.orientation = Quaternion(-0.35355339, -0.35355339, -0.14644661,  0.85355339)
    # test.send_cart_goal(goal)
