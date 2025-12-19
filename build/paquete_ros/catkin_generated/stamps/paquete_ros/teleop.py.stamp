#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def main():
    #nombre del nodo
    rospy.init_node('keyboard_teleop_node', anonymous=True)
    
    # publicaciones
    pub = rospy.Publisher('/cmd_vel_keyboard', Twist, queue_size=10)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 1.0
        twist.linear.y = 2.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 3.0
        
        rospy.loginfo(
            "Publicando x=%.2f y=%.2f z=%.2f", 
            twist.linear.x, 
            twist.linear.y,
            twist.angular.z
        )
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

