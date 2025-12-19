#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

# variable Publish que permite en envio de mensajes desde callbacks
pub = None

def twist_callback(msg):
    global pub
    # despliegue de los comandos desde teclado
    rospy.loginfo(
        "Twist recibido -> Pos x=%.2f, y=%.2f, z=%.2f ",
        msg.linear.x,
        msg.linear.y,
        msg.angular.z,
    )
    
    # publicando velocidades
    twist = Twist()
    twist.linear.x = msg.linear.x
    twist.linear.y = msg.linear.y
    twist.linear.z = 0.0
    
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = msg.angular.z
    
    rospy.loginfo(
        "reenviando x=%.2f y=%.2f z=%.2f", 
        twist.linear.x, 
        twist.linear.y,
        twist.angular.z
    )
    pub.publish(twist)    
    
def pose_callback(msg):
    rospy.loginfo(
        "Pose recibida -> Pos x=%.2f, y=%.2f, theta=%.2f ",
        msg.x,
        msg.y,
        msg.theta,
    )

def main():
    global pub
    # nombre del nodo
    rospy.init_node('kuka_omnicontroller_node', anonymous=True)
    
    # publicaciones
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # suscripciones
    rospy.Subscriber('/cmd_vel_keyboard', Twist, twist_callback)
    rospy.Subscriber('/kuka/pose', Pose2D, pose_callback)
    rospy.spin()
    

if __name__ == '__main__':
    main()
