import rospy
import control_msgs.msg
import trajectory_msgs.msg
#import  common_msgs
from trajectory_msgs.msg import JointTrajectory
import actionlib
from std_msgs.msg import String


class JointGoalSplitter:
    def __init__(self):
        self.base_client = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)#/hsrb/omni_base_controller/follow_joint_trajectory
        self.action_client = actionlib.SimpleActionClient('/whole_body_controller/follow_joint_trajectory/goal',
                                                        control_msgs.msg.FollowJointTrajectoryAction)
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/hsrb/arm_trajectory_controller/follow_joint_trajectory/goal",#     /hsrb/arm_trajectory_controller/follow_joint_trajectory/goal
                         control_msgs.msg.FollowJointTrajectoryActionGoal, self.callback)

        self._as = actionlib.SimpleActionServer('/test_action', control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.callback, auto_start=False)
        self._as.start()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def callback(self, data):
        name = data.goal.trajectory.joint_names
        fake_index_x = name.index('test_joint_x')
        fake_index_y = name.index('test_joint_y')
        #points = data.goal.trajectory.points#[0].positions
        base_goal = control_msgs.msg.FollowJointTrajectoryAction()
        move_goal = control_msgs.msg.FollowJointTrajectoryAction()
        base_traj = trajectory_msgs.msg.JointTrajectory()
        move_traj = trajectory_msgs.msg.JointTrajectory()
        base_traj.joint_names = ['test_joint_x', 'test_joint_y']
        other_joints = []
        for joint in name:# funktioniert nicht: liste von liste
            if name is not 'test_joint_x' and name is not 'test_joint_y':
                other_joints.append(name)

        #move_traj.joint_names = other_joints
        base_traj_points = []
        for p in data.goal.trajectory.points:
            if len(p.positions) > max(fake_index_x, fake_index_y): #einzeln checken
                base_traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
                tu = (p.positions[fake_index_x], p.positions[fake_index_y])
                base_traj_point.positions = tu
                base_traj_point.time_from_start.nsecs = p.time_from_start.nsecs
                base_traj_point.time_from_start.secs = p.time_from_start.secs
                base_traj_points.append(base_traj_point)

        base_traj.points = tuple(base_traj_points)

        '''move_traj_points = []
        other_joint_pos = []
        for i in range(len(p.positions)):
            if i == fake_index_x or i == fake_index_y:
                continue
            other_joint_pos.append(p.positions[i])

        other_tu = tuple(other_joint_pos)
        move_traj_point = trajectory_msgs.msg.JointTrajectoryPoint
        move_traj_point.positions = other_tu
        move_traj_points.append(move_traj_point)

        move_traj.points = tuple(move_traj_points)'''

        base_goal.action_goal = base_traj
        print("test")

        self.base_client.send_goal(base_goal)



if __name__ == '__main__':
    j = JointGoalSplitter()
