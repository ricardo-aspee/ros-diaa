# Control en Lazo Abierto de un Robot Omnidireccional con ROS
Tarea 1 modulo Robot Operating System

## Ejecución de cada nodo

1. Antes de ejecutar cada nodo, se debe iniciar roscore en un terminal independiente:
   ```
   roscore
   ```
   > Roscore queda en ejecución en este terminal, por lo que no se puede utilizar para realizar otros comandos.
2. Ejecución de *Nodo 1: Nodo de Teclado*, abrir un segundo terminal y ejecutar:
   ```
   cd ~/Documents/catkin_taller1_ws
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   rosrun paquete_ros teleop.py
   ```
   > Este terminal queda a la espera de recibir las velocidades desde teclado utilizando.
2. Ejecución de *Nodo 2: Nodo Principal del Robot*, abrir un tercer terminal y ejecutar:
   ```
   cd ~/Documents/catkin_taller1_ws
   source devel/setup.bash
   rosrun paquete_ros controller.py
   ```
   > En este terminal se desplegará la posición del robot. 

2. Ejecución de *Nodo 3: Nodo del Simulador Coppelia*, abrir un cuarto terminal y ejecutar:
   ```
   cd ~/Documents/CoppeliaSim_Pro_V4_9_0_rev6_Ubuntu20_04
   ./coppeliaSim.sh
   ```
   > En este terminal iniciará el software de Coppelia.

   Una vez iniciado de Coppelia, se debe cargar la escena, para esto se accede a **File > Open Scene ...** y se selecciona el archivo **\~/Documents/catkin_taller1_ws/simulation_vrep/entorno.ttt**.
> [!CAUTION]
> Debido a que tengo problemas para guardar los cambios al crear una escena, se utiliza el robot *Pioner p3dx*, al cual se le debe cargar el script de comunicación con ROS que se deja a disposición en la ruta **~/Documents/catkin_taller1_ws/simulation_vrep/main.lua**.

Iniciar la simulación dándole click al botón Play en el Menú.

## Descripcion de los tópicos utilizados

Al revisar el listado de tópicos ejecutando:
```
rostopic list
```
Se pueden observar los siguientes:
```
/cmd_vel
/cmd_vel_keyboard
/kuka/pose
/rosout
/rosout_agg
/tf
/vrep/info
```
**/cmd_vel_keyboard**: Publica las teclas ingresadas por el usuario como velocidades en formato **geometry_msgs/Twist**.
**/cmd_vel**: Transmite las velocidades en formato **geometry_msgs/Twist** hacia el simulador en Coppelia.
**/kuka/pose**: El simulador publica las coordenadas de su ubicación en formato **geometry_msgs/Pose2D**.


## Capturas de pantalla
*Pendiente*

