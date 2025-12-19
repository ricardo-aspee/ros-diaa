#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  Copyright (c) 2025. Diplomado en Inteligencia Artificial Aplicada
#  Ricardo Aspee ricardo.aspee@gmail.com
"""
Nodo 1: Nodo de Teclado

Nombre del nodo: keyboard_teleop_node

Función: Capturar teclas presionadas por el usuario y convertirlas en comandos de
velocidad.
"""
import rospy
import termios
import tty
import sys
import select

from geometry_msgs.msg import Twist

def leer_tecla():
    """ funcion que detecta la tecla presionada """
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def check_movimiento(velocidad_longitudinal, velocidad_rotacional):
    """ Esta funcion indica si el movil deberia moverse, girar o detenerse 
    segun las velocidades a enviar"""
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
    """ se incializa el topico y se genera el mensaje a publicar """
    #nombre de este nodo
    rospy.init_node('keyboard_teleop_node', anonymous=True)
    
    # publicaciones
    pub = rospy.Publisher('/cmd_vel_keyboard', Twist, queue_size=10)
    rate = rospy.Rate(10) # frecuencia de lectura del teclado 

    # Despligue de las teclas que el usuario puede presionar
    print("Controles:")
    print("W: Avanzar | S: Retroceder | A: Izquierda | D: Derecha")
    print("X: Detener | Q: Salir")
    
    # Configuración del terminal como entrada 
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # inicializacion de las velocidades
    velocidad_longitudinal = 0.0
    velocidad_rotacional = 0.0
    
    # incrementos (o reducciones) de las velocidades segun comandos
    incremento_longitudinal = 0.2
    incremento_rotacional = 0.2
    
    # ciclo principal para la lectura de teclado y la accion segun esta
    try:
        while not rospy.is_shutdown():
            tecla = leer_tecla()
            if tecla:
                tecla = tecla.lower()
                # aumento de velocidad hacia adelante
                if tecla == 'w':
                    velocidad_longitudinal += incremento_longitudinal
                    velocidad_rotacional = 0
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                
                # disminucion de velocidad hacia adelante
                elif tecla == 's':
                    velocidad_longitudinal -= incremento_longitudinal
                    velocidad_rotacional = 0
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                
                # aumento de giro hacia la izquierda o reduccion de velocidad de giro hacia la derecha                
                elif tecla == 'a':
                    velocidad_longitudinal = 0
                    velocidad_rotacional -= incremento_rotacional
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)

                # aumento de giro hacia la derecha o reduccion de velocidad de giro hacia la izquierda                
                elif tecla == 'd':
                    velocidad_longitudinal = 0
                    velocidad_rotacional += incremento_rotacional
                    check_movimiento(velocidad_longitudinal, velocidad_rotacional)
                    
                # detencion del movil
                elif tecla == 'x':
                    velocidad_longitudinal = 0
                    velocidad_rotacional = 0
                    print("Robot detenido")
            
                # generando el mensaje a publicar
                twist = Twist()
                twist.linear.x = velocidad_longitudinal
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = velocidad_rotacional
                
                # despligue del mensaje a publicar
                #rospy.loginfo(
                #    "Publicando x=%.2f y=%.2f z=%.2f", 
                #    twist.linear.x, 
                #    twist.linear.y,
                #    twist.angular.z
                #)
                
                # publicando el mensaje con las velocidades
                pub.publish(twist)
            
            # esperando un tiempo para la siguiente lectura
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

