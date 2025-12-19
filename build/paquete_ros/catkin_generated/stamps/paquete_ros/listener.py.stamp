#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Escuche: %s", data.data)

def listener():
    rospy.init_node('topico_listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
