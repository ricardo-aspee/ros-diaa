#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  Copyright (c) 2025. Diplomado en Inteligencia Artificial Aplicada
#  Ricardo Aspee ricardo.aspee@gmail.com
"""
Nodo 2: Nodo Principal del Robot

Nombre del nodo: kuka_omni_controller_node

Función:
• Suscribirse a los comandos de velocidad del teclado.
• Publicar comandos de velocidad hacia el simulador.
• Suscribirse la información de posición del robot enviada desde el simulador
• Imprimir en consola la posición del robot.
"""
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D


# variable Publish que permite en envio de mensajes desde callbacks
pub = None

def twist_callback(msg):
    """Se gatilla ewsta funcion al recibir las velocidades capturadas desde teclado."""
    global pub
    # despliegue de los comandos publicados desde teclado
    #rospy.loginfo(
    #    "Twist recibido -> Pos x=%.2f, y=%.2f, z=%.2f ",
    #    msg.linear.x,
    #    msg.linear.y,
    #    msg.angular.z,
    #)
    
    # generando el mensaje con las velocidades a publicar
    twist = Twist()
    twist.linear.x = msg.linear.x
    twist.linear.y = msg.linear.y
    twist.linear.z = 0.0
    
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = msg.angular.z
    
    # despliegue de lo publicado
    #rospy.loginfo(
    #    "reenviando x=%.2f y=%.2f z=%.2f", 
    #    twist.linear.x, 
    #    twist.linear.y,
    #    twist.angular.z
    #)
    
    # realizando la publicacion
    pub.publish(twist)    
    
def pose_callback(msg):
    """Esta funcion se gatilla cuando se recibe las coordenadas publicadas por el simulador de Coppelia"""
    rospy.loginfo(
        "Pose recibida -> Pos x=%.2f, y=%.2f, theta=%.2f ",
        msg.x,
        msg.y,
        msg.theta,
    )

def main():
    """Inicializacion de los topicos a publicar y suscribirse"""
    global pub
    # nombre de este nodo
    rospy.init_node('kuka_omnicontroller_node', anonymous=True)
    
    # publicaciones
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # suscripciones
    rospy.Subscriber('/cmd_vel_keyboard', Twist, twist_callback)
    rospy.Subscriber('/kuka/pose', Pose2D, pose_callback)
    rospy.spin()
    

if __name__ == '__main__':
    main()
