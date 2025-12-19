#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import roslib
import rospy
import math
import numpy as np  
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Inertia


def wraptopi(x):
    x = x - np.floor(x/(2*math.pi)) *2 *math.pi
    if x >= math.pi:
        x = x - 2*math.pi
    return x

def callback_robot_base_pose(data):
    print(data)
    robot_base_pose_x = data.x
    robot_base_pose_y = data.y
    robot_base_pose_t = wraptopi(data.theta)
  
  
def twist_callback(msg):
    # despliegue de los comandos desde teclado
    rospy.loginfo(
        "Twist recibido -> Pos x=%.2f, y=%.2f, z=%.2f ",
        msg.linear.x,
        msg.linear.y,
        msg.angular.z,
    )
    
    # envio de comandos hacia coppelia
    # TODO
    
def main():
    # nombre del nodo
    rospy.init_node('coppelia_simulator_node', anonymous=True)
    
    # publicaciones
    #pub = rospy.Publisher('/kuka/pose', Pose2D, queue_size=10)
    
    # suscripciones
    rospy.Subscriber('/cmd_vel', Twist, twist_callback)
    rospy.Subscriber('/vrep_sim_robot_base_pose1', Pose2D, callback_robot_base_pose)
    
    rospy.spin()
    
    # conexion a coppelia

    
    # obtencion de la pose desde coppelia
    # TODO
    

if __name__ == '__main__':
    main()
