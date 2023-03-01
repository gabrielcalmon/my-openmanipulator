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
        #group.execute(plan, wait=True)
        break

def go2position(position_list):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        group.set_position_target(position_list)
        plan = group.go(wait=True)
        group.stop()
        #group.clear_position_targets()
        #group.execute(plan, wait=True)
        break
    #print('========================================')
    #print()

def go2pose(position_list):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = group.get_current_pose().pose

        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.position.x = position_list[0]
        pose_goal.position.y = position_list[1]
        pose_goal.position.z = position_list[2]

        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        #group.clear_position_targets()
        #group.execute(plan, wait=True)
        break

def go2init_pose():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = group.get_current_pose().pose

        pose_goal.orientation.x = 0.5147866096539608
        pose_goal.orientation.y = 0.07352425677103026
        pose_goal.orientation.z = 0.2379017918991315

        pose_goal.orientation.w = -0.7037410864776212
        pose_goal.position.x = 0.10089453311810097
        pose_goal.position.y = 0.20061463041243796
        pose_goal.position.z = 0.6740345291264079

        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        #group.clear_position_targets()
        #group.execute(plan, wait=True)
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
    """
    C <- B
    |    ^
    v    |
    D -> A

    # O end-effector comeca em 'A' e se move ao longo de um retangulo com lados zstep e ystep para, respectivamente, a altura e o comprimento, medidos em metros
    """
    # ideia: calcular outworkspace através do comprimento do robô
    xstep=0.075
    ystep=0.2

    pointA=[0.5147866096539608, 0.07352425677103026, 0.2379017918991315]
    pointB=[pointA[0]+xstep, pointA[1], pointA[2]]
    pointC=[pointB[0], pointB[1]-ystep, pointB[2]]
    pointD=[pointC[0]-xstep, pointC[1], pointC[2]]

    # go2position(pointA)
    # input('===Press enter to go to the next point===\n')

    # go2position(pointB)
    # input('===Press enter to go to the next point===\n')

    # go2position(pointC)
    # input('===Press enter to go to the next point===\n')

    # go2position(pointD)
    # input('===Press enter to go to the next point===\n')

    # go2position(pointA)
    # print('Finished!!')

    go2init_pose()

    go2pose(pointA)
    input('===Press enter to go to the next point===\n')

    go2pose(pointB)
    input('===Press enter to go to the next point===\n')

    go2pose(pointC)
    input('===Press enter to go to the next point===\n')

    go2pose(pointD)
    input('===Press enter to go to the next point===\n')

    go2pose(pointA)
    print('Finished!!')
    print_pose()