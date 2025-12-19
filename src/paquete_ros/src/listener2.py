#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def pose_callback(msg):
    rospy.loginfo(
        "Pose recibida -> Pos x=%.2f, y=%.2f, z=%.2f ",
        msg.position.x,
        msg.position.y,
        msg.position.z,
    )
    rospy.loginfo("")
    rospy.loginfo(
        "Orientacion x=%.2f, y=%.2f, z=%.2f, w=%.2f ",
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    )
    

def listener():
    rospy.init_node('topico_listener', anonymous=True)
    rospy.Subscriber('geometry_msgs/Pose', Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
