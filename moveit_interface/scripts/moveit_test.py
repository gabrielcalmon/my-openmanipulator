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
        group.clear_pose_targets()
        #group.execute(plan, wait=True)
        break

def go2pose_xy(position_list):
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
        pose_goal.position.z = current_pose.position.z

        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        #group.execute(plan, wait=True)
        break

def go2pose_z(position_z):
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

        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = current_pose.position.y
        pose_goal.position.z = position_z

        print(current_pose)
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
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
        pose_goal.position.x = 0.46567264870156727
        pose_goal.position.y = 0.32583083817351666
        pose_goal.position.z = 0.22664581792749117

        pose_goal.orientation.x = -0.7069538807378075
        pose_goal.orientation.y = -0.15652695555454
        pose_goal.orientation.z = -0.013479088538918209
        pose_goal.orientation.w = 0.6895896148193801

        

        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        #group.execute(plan, wait=True)
        break

def cartesianMove():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
    group.go(wait=True)
    # group.stop()

    # print "============ Waiting while RVIZ displays plan3..."
    # rospy.sleep(5)

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

def init_ang():
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0 * pi / 180
    joint_goal[1] = 21 * pi / 180
    joint_goal[2] = 20 * pi / 180
    joint_goal[3] = 0 * pi / 180
    joint_goal[4] = 0 * pi /180
    joint_goal[5] = 0 * pi /180

    group.go(joint_goal, wait=True)
    rospy.sleep(1)
    group.stop()
    rospy.sleep(1)

def demo_function():
    zstep=0.2
    ystep=0.6

    pointA = [0.46567264870156727, 0.32583083817351666, 0.22664581792749117]
    pointB=[pointA[0], pointA[1], pointA[2]+zstep]
    pointC=[pointB[0], pointB[1]-ystep, pointB[2]]
    pointD=[pointC[0], pointC[1], pointC[2]-zstep]
    print('Going to home position')
    go2init_pose()

    # zy
    input('===Press enter to use "position control"===\n')
    go2position(pointA)
    go2position(pointB)
    go2position(pointC)
    go2position(pointD)
    go2position(pointA)

    input('===Press enter to use "pose control"===\n')
    go2init_pose()
    go2pose(pointA)
    go2pose(pointB)
    go2pose(pointC)
    go2pose(pointD)
    go2pose(pointA)
    print('Finished!!')


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
    

## position test script
    # zstep=0.2
    # ystep=0.2

    # pointA=[0.15499688221284214, 0.3258172746579836, 0.1389946113328881]
    # pointB=[pointA[0], pointA[1], pointA[2]+zstep]
    # pointC=[pointB[0], pointB[1]-ystep, pointB[2]]
    # pointD=[pointC[0], pointC[1], pointC[2]-zstep]
    # go2position(pointA)
    # #input('===Press enter to go to the next point===\n')

    # go2position(pointB)
    # #input('===Press enter to go to the next point===\n')

    # go2position(pointC)
    # #input('===Press enter to go to the next point===\n')

    # go2position(pointD)
    # #input('===Press enter to go to the next point===\n')

    # go2position(pointA)
    # print('Finished!!')

## pose test script (xy)
    # xstep=0.075
    # ystep=0.5

    # #pointA=[0.5147866096539608, 0.07352425677103026, 0.2379017918991315]
    # pointA = [0.46567264870156727, 0.32583083817351666, 0.22664581792749117]
    # pointB=[pointA[0]+xstep, pointA[1], pointA[2]]
    # pointC=[pointB[0], pointB[1]-ystep, pointB[2]]
    # pointD=[pointC[0]-xstep, pointC[1], pointC[2]]
    # print('Going to home position')
    # go2init_pose()
    # #input('===Get at home position===')

    # go2pose(pointA)
    # #input('===Press enter to go to the next point===\n')

    # go2pose(pointB)
    # #input('===Press enter to go to the next point===\n')

    # go2pose(pointC)
    # #input('===Press enter to go to the next point===\n')

    # go2pose(pointD)
    # #input('===Press enter to go to the next point===\n')

    # go2pose(pointA)
    # print('Finished!!')
    # print_pose()

## pose test script (yz) da demo_function
    zstep=0.2
    ystep=0.5

    pointA = [0.46567264870156727, 0.32583083817351666, 0.22664581792749117]
    pointB=[pointA[0], pointA[1], pointA[2]+zstep]
    pointC=[pointB[0], pointB[1]-ystep, pointB[2]]
    pointD=[pointC[0], pointC[1], pointC[2]-zstep]
    #print('Going to home position')
    #go2init_pose()

    # zy
    # input('===Press enter to use "pose control"===\n')
    #go2init_pose()
    # go2pose(pointA)
    # go2pose(pointB)
    # go2pose(pointC)
    # go2pose(pointD)
    # go2pose(pointA)

    pointgoal = [0.4490318119261384, -0.022545023641873635, 0.11360713599276696]
    # go2pose_z(0.22664581792749117)
    # go2pose_xy([0.46567264870156727, 0.32583083817351666])

    go2position(pointgoal)
    
    print('Finished!!')

    # cartesianMove()
    #init_ang()
    #demo_function()