#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('geometry_msgs/Pose', Pose, queue_size=10)
    rospy.init_node('topico_talker', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 0.0
        
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        rospy.loginfo("Publicando x=%.2f y=%.2f", pose.position.x, pose.position.y)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

