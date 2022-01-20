#! /usr/bin/env python

import rospy
import math
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler

class MoveitController:
    def __init__(self):
        rospy.init_node('sketch_robot_controller')
        self.arm_group = moveit_commander.MoveGroupCommander('arm_planning_group')
        self.action_client = SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
        self.action_client.wait_for_server()
        rospy.loginfo('>>> Moveit Init done')

    def get_pose(self, xyz, rpy=(), wxyz=()):
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        if len(rpy) > len(wxyz):
            quaternion = quaternion_from_euler(*rpy)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
        elif len(rpy) < len(wxyz):
            pose.orientation.x = wxyz[0]
            pose.orientation.y = wxyz[1]
            pose.orientation.z = wxyz[2]
            pose.orientation.w = wxyz[3]
        else:
            pose.orientation.w = 1.0
        return pose

    def go_to_pose(self, planning_group, pose, planning_group_name=""):
        # self.print_current_state(planning_group, planning_group_name + ' Current')
        planning_group.set_pose_target(pose)
        flag_plan = planning_group.go(wait=True)
        # self.print_current_state(planning_group, planning_group_name + ' Final')

        if flag_plan:
            rospy.loginfo('>>> {} go_to_pose() success'.format(planning_group_name))
        else:
            rospy.loginfo('>>> {} go_to_pose() failed'.format(planning_group_name))
        return flag_plan

    def set_joint_angles(self, planning_group, joint_angles, planning_group_name=""):
        # self.print_current_state(planning_group, planning_group_name + ' Current')
        planning_group.set_joint_value_target(joint_angles)
        flag_plan = planning_group.go(wait=True)
        # self.print_current_state(planning_group, planning_group_name + ' Final')

        if flag_plan:
            rospy.loginfo('>>> set_joint_angles() success')
        else:
            rospy.loginfo('>>> set_joint_angles() failed')
        return flag_plan

    def go_to_predefined_pose(self, planning_group, pose_name):
        rospy.loginfo('>>> Going to Pose: {}'.format(pose_name))

        planning_group.set_named_target(pose_name)
        goal = ExecuteTrajectoryGoal()

        goal.trajectory = planning_group.plan()
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

        rospy.loginfo('Now at Pose: {}'.format(pose_name))

    def print_current_state(self, planning_group, pose_prefix=""):
        rospy.loginfo('>>> {} Pose: {}'.format(pose_prefix, planning_group.get_current_pose().pose))
        rospy.loginfo('>>> {} joint angles: {}'.format(pose_prefix, planning_group.get_current_joint_values()))

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('object deleted')

if __name__ == '__main__':
    mc = MoveitController()
    mc.print_current_state(mc.arm_group)
    # xyz = (0.0696250953621, -0.0072664481219, 0.200716152474)
    # wxyz = (-0.00191673151116, 0.695245818496, 0.00198129230217, 0.718766758055)
    # xyz_rpy = ()
    # mc.go_to_pose(mc.arm_group, mc.get_pose(xyz, wxyz=wxyz), "arm_planning_group")
    mc.go_to_predefined_pose(mc.arm_group, "armReady")
    del(mc)
