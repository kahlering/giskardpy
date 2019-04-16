import rospy
import control_msgs.msg
import trajectory_msgs.msg
import threading
#from trajectory_msgs.msg import JointTrajectory
import actionlib

base_joints = ['odom_x', 'odom_y', 'odom_t']
arm_joints = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
all_joints = base_joints + arm_joints


class JointGoalSplitter:
    def __init__(self):
        rospy.init_node('JointGoalSplitter', anonymous=True)
        self.base_client = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)#/hsrb/omni_base_controller/follow_joint_trajectory
        self.arm_client = actionlib.SimpleActionClient('/hsrb/arm_trajectory_controller/follow_joint_trajectory/goal',
                                                        control_msgs.msg.FollowJointTrajectoryAction)

        #rospy.Subscriber("/hsrb/arm_trajectory_controller/follow_joint_trajectory/goal",#     /hsrb/arm_trajectory_controller/follow_joint_trajectory/goal
        #                 control_msgs.msg.FollowJointTrajectoryActionGoal, self.callback)

        self._as = actionlib.SimpleActionServer('/test_action', control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.callback, auto_start=False)
        self._as.start()

        self.pub = rospy.Publisher('/test_action/state', control_msgs.msg.JointTrajectoryControllerState, queue_size=10)
        t = threading.Thread(target=self.publisher_thread())
        t.daemon = True
        t.start()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def callback(self, goal):
        base_ids = []
        arm_ids = []
        for joint_name in base_joints:
            base_ids.append(goal.trajectory.joint_names.index(joint_name))
        for joint_name in arm_joints:
            arm_ids.append(goal.trajectory.joint_names.index(joint_name))
        base_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        arm_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        base_traj = trajectory_msgs.msg.JointTrajectory()
        arm_traj = trajectory_msgs.msg.JointTrajectory()
        base_traj.joint_names = base_joints
        arm_traj.joint_names = arm_joints

        base_traj_points = []
        arm_traj_points = []
        for p in goal.trajectory.points:
            if len(p.positions) > max(base_ids): #einzeln checken
                base_traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
                joint_pos = [p.positions[i] for i in base_ids]
                base_traj_point.positions = tuple(joint_pos)
                base_traj_point.time_from_start.nsecs = p.time_from_start.nsecs
                base_traj_point.time_from_start.secs = p.time_from_start.secs
                base_traj_points.append(base_traj_point)

            if len(p.positions) > max(arm_ids):
                arm_traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
                joint_pos_arm = [p.positions[i] for i in arm_ids]
                arm_traj_point.positions = tuple(joint_pos_arm)
                arm_traj_point.time_from_start.nsecs = p.time_from_start.nsecs
                arm_traj_point.time_from_start.secs = p.time_from_start.secs
                arm_traj_points.append(base_traj_point)

        base_traj.points = tuple(base_traj_points)
        arm_traj.points = tuple(arm_traj_points)

        base_goal.trajectory = base_traj
        arm_goal.trajectory = arm_traj

        self.base_client.send_goal(base_goal)
        #self.arm_client.send_goal(arm_goal)

        result = control_msgs.msg.FollowJointTrajectoryResult()
        self._as.set_succeeded(result)

    def publisher_thread(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            state = control_msgs.msg.JointTrajectoryControllerState()
            state.joint_names = all_joints#['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'base_l_drive_wheel_joint', 'base_l_passive_wheel_x_frame_joint',
                                 #'base_l_passive_wheel_y_frame_joint', 'base_l_passive_wheel_z_joint','base_r_drive_wheel_joint', 'base_roll_joint',
                                 #'base_r_passive_wheel_x_frame_joint','base_r_passive_wheel_y_frame_joint','base_r_passive_wheel_z_joint',
                                 #'hand_l_spring_proximal_joint','hand_motor_joint','hand_r_spring_proximal_joint','head_pan_joint','head_tilt_joint',
                                 #'wrist_flex_joint','wrist_roll_joint','odom_x','odom_y', 'odom_t']
            self.pub.publish(state)
            rate.sleep()




if __name__ == '__main__':
    j = JointGoalSplitter()
