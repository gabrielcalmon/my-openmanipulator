#!/usr/bin/env python3
# documentacao do moveit: http://docs.ros.org/en/jade/api/moveit_commander/html/move__group_8py_source.html
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def init_pose():
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    group.go(joint_goal, wait=True)
    rospy.sleep(1)
    group.stop()
    rospy.sleep(1)

    return 'outcome2'


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = group.get_current_pose().pose
        print(current_pose)
        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = 0.1
        pose_goal.position.z = current_pose.position.z
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        # group.execute(plan, wait=True)
        break

def main2():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = group.get_current_pose().pose
        print(current_pose)
        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = 0.1
        pose_goal.position.z = current_pose.position.z
        #group.set_pose_target(pose_goal)
        print(pose_goal.position.x)
        group.set_position_target([0.44,0,0.44])
        plan = group.go(wait=True)
        group.stop()
        #group.clear_position_targets()
        # group.execute(plan, wait=True)
        break

def print_pose():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        current_pose = group.get_current_pose().pose
        print('========================================')
        print(current_pose)
        rospy.sleep(1)

if __name__ == '__main__':
    #main2()
    # ideia: calcular outworkspace através do comprimento do robô
    zstep=0.15
    ystep=0.3
    point1=[0.31238732139108205, 0.3144607016279254, 0.3532427239267847]
    point2=[point1[0], point1[1], point1[2]+zstep]
    # point1.position.x = 0.4132461918243058
    # point1.position.x = 0.002251197407117802
    # point1.position.x = 0.4419417336147707
    print_pose()