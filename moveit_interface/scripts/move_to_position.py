#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
pose_msg = geometry_msgs.msg.Pose()
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

def go2position(position_list):
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    #group.set_goal_tolerance(0.01)
    while not rospy.is_shutdown():
        group.set_position_target(position_list)
        plan = group.go(wait=True)
        group.stop()
        #group.execute(plan, wait=True)
        break
    return True # retorna valor verdadeiro para indicar que finalizou a execucao

def go2pose(position_list):
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    #rospy.sleep(1)
    group = moveit_commander.MoveGroupCommander(group_name)
    #group.set_goal_tolerance(0.1)

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
        group.clear_pose_targets()
        #group.execute(plan, wait=True)
        break

def my_sign(x):
    """retorna o sinal do numero passado (-1 ou +1) ou ainda 0 para valores nulos"""
    return (x > 0) - (x < 0)

def callback(incoming_pose):
    global pose_msg
    pose_msg.position.x = incoming_pose.position.x
    pose_msg.position.y = incoming_pose.position.y
    pose_msg.position.z = incoming_pose.position.z
    pose_msg.orientation.x = incoming_pose.orientation.x
    pose_msg.orientation.y = incoming_pose.orientation.y
    pose_msg.orientation.z = incoming_pose.orientation.z
    pose_msg.orientation.w = incoming_pose.orientation.w

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    try:      
        rospy.init_node('target_listener', anonymous=False)

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber("target_calculator/pose_tag2world", geometry_msgs.msg.Pose, callback)

        safe_dist_x = 0.05    # aplica um offset no eixo fim, a fim de que o end-effector nao se sobreponha ao aruco

        while not rospy.is_shutdown():
            # a soma considerando o sinal da coordenada garante que o parametro de seguranca seja aplicado de forma a diminuir o modulo do ponto objetivo
            go2position([pose_msg.position.x-(my_sign(pose_msg.position.x)*safe_dist_x), pose_msg.position.y, pose_msg.position.z])
    except rospy.ROSInterruptException:
        pass