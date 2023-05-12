#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_world2tag_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose_target = rospy.Publisher('target_calculator/pose_world2tag', geometry_msgs.msg.Pose, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            #transf = tfBuffer.lookup_transform('id_0','end_link',  rospy.Time()) #target_frame, source_frame
            transf = tfBuffer.lookup_transform('world', 'id_0', rospy.Time()) #target_frame, source_frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Pose()
        msg.position = transf.transform.translation
        msg.orientation = transf.transform.rotation

        # msg.angular.z = 4 * math.atan2(transf.transform.translation.y, transf.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(transf.transform.translation.x ** 2 + transf.transform.translation.y ** 2)
        print(transf)
        pose_target.publish(msg)

        rate.sleep()