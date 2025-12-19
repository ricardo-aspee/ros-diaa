#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import termios
import tty
import sys
import select

from geometry_msgs.msg import Twist

def leer_tecla():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def check_movimiento(velocidad_longitudinal, velocidad_rotacional):
    if velocidad_longitudinal == 0 and velocidad_rotacional == 0:
        print("Detenido")
    elif velocidad_longitudinal > 0:
        print("Avanzando")
    elif velocidad_longitudinal < 0:
        print("Retrocediendo")
    elif velocidad_rotacional > 0:
        print("Girando a la derecha")
    elif velocidad_rotacional < 0:
        print("Girando a la izquierda")
    

def main():
    #nombre del nodo
    rospy.init_node('keyboard_teleop_node', anonymous=True)
    
    # publicaciones
    pub = rospy.Publisher('/cmd_vel_keyboard', Twist, queue_size=10)
    rate = rospy.Rate(1)

    print("Controles:")
    print("W: Avanzar | S: Retroceder | A: Izquierda | D: Derecha")
    print("X: Detener | Q: Salir")
    # Configuración del terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    velocidad_longitudinal = 0.0
    velocidad_rotacional = 0.0
    
    incremento_longitudinal = 0.2
    incremento_rotacional = 0.2
    
    try:
        while not rospy.is_shutdown():
            tecla = leer_tecla()
            if tecla:
                tecla = tecla.lower()
                if tecla == 'w':
                    velocidad_longitudinal += incremento_longitudinal
                    velocidad_rotacional = 0
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)

                elif tecla == 's':
                    velocidad_longitudinal -= incremento_longitudinal
                    velocidad_rotacional = 0
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                    
                elif tecla == 'a':
                    velocidad_longitudinal = 0
                    velocidad_rotacional -= incremento_rotacional
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                elif tecla == 'd':
                    velocidad_longitudinal = 0
                    velocidad_rotacional += incremento_rotacional
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                elif tecla == 'x':
                    velocidad_longitudinal = 0
                    velocidad_rotacional = 0
                    print("Robot detenido")
            
                # publicando las velocidades
                twist = Twist()
                twist.linear.x = velocidad_longitudinal
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = velocidad_rotacional
                
                rospy.loginfo(
                    "Publicando x=%.2f y=%.2f z=%.2f", 
                    twist.linear.x, 
                    twist.linear.y,
                    twist.angular.z
                )
                pub.publish(twist)
            
            rate.sleep()
    except KeyboardInterrupt:
        print("Interrupción por teclado")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

