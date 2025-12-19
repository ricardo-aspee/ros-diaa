#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sim

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

def conectarCoppelia(connectionPort):
    print("Estableciendo conexión...")
    try:
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', connectionPort, True, True, 2000, 5)
        if clientID != -1:
            print("Conexion exitosa con CoppeliaSim. Cliente: ", str(clientID))
        else:
            print("Error con el cliente de CoppeliaSim. Código: ", str(clientID))
        return clientID
    except Exception as e:
        print("Error de conexión. Código: ", str(e))
        return -1

def desconectarCoppelia(clientID):
    sim.simxFinish(clientID)
    print("Sesión desconectada.")


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
    pub = rospy.Publisher('/kuka/pose', Pose2D, queue_size=10)
    
    # suscripciones
    rospy.Subscriber('/cmd_vel', Twist, twist_callback)
    rospy.spin()
    
    # conexion a coppelia
    clientID = conectarCoppelia(19999)
    if clientID != -1:
        desconectarCoppelia(clientID)
    
    # obtencion de la pose desde coppelia
    # TODO
    

if __name__ == '__main__':
    main()
