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
    print("Q: Giro Izq  | E: Giro Der  | X: Detener")
    
    # Configuración del terminal como entrada 
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # inicializacion de los incrementos de las velocidades
    incremento_velocidad_longitudinal = 1.2
    incremento_velocidad_lateral = 0.8
    incremento_velocidad_rotacional = 0.75
    
    comandos = ['q','w','e','a','s','d','x']
    
    # ciclo principal para la lectura de teclado y la accion segun esta
    try:
        while not rospy.is_shutdown():
            tecla = leer_tecla()
            if tecla:
                # si tecla no corresponde a los comandos, no realizar accion
                if tecla not in comandos:
                    continue
                
                # inicializacion de las velocidades
                velocidad_longitudinal = 0.0
                velocidad_lateral = 0.0
                velocidad_rotacional = 0.0
                
                tecla = tecla.lower()
                # aumento de velocidad hacia adelante
                if tecla == 'w':
                    print("Avanzando")
                    velocidad_longitudinal = incremento_velocidad_longitudinal
                
                # retrocediendo
                elif tecla == 's':
                    print("Retrocediendo")
                    velocidad_longitudinal = -incremento_velocidad_longitudinal
                
                # avance hacia la izquierda                
                elif tecla == 'a':
                    print("Izquierda")
                    velocidad_lateral = -incremento_velocidad_lateral

                # avance hacia la derecha
                elif tecla == 'd':
                    print("Derecha")
                    velocidad_lateral = incremento_velocidad_lateral
                    
                # giro hacia la izquierda                
                elif tecla == 'q':
                    print("Girando a la izquierda")
                    velocidad_rotacional = -incremento_velocidad_rotacional

                # giro hacia la derecha
                elif tecla == 'e':
                    print("Girando a la derecha")
                    velocidad_rotacional = incremento_velocidad_rotacional
                    
                # detencion del movil
                elif tecla == 'x':
                    print("Detenido")
            
                # generando el mensaje a publicar
                twist = Twist()
                twist.linear.x = velocidad_longitudinal
                twist.linear.y = velocidad_lateral
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

