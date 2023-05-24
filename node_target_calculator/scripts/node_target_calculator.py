#!/usr/bin/env python3
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    try:
        rospy.init_node('tf_tag2world_listener')

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        pose_target = rospy.Publisher('target_calculator/pose_tag2world', geometry_msgs.msg.Pose, queue_size=10)

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                transf = tfBuffer.lookup_transform('world', 'id_0', rospy.Time()) #target_frame, source_frame (de ... para ...)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            msg = geometry_msgs.msg.Pose()
            #msg.position = transf.transform.translation
            
            # msg.position.x = 0.3377399793376716
            # msg.position.y = 0.15642701465270953
            # msg.position.z = 0.21850777203918673
            msg.position = transf.transform.translation
            msg.orientation = transf.transform.rotation

            print(msg)
            pose_target.publish(msg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass