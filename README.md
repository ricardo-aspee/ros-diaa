# Control en Lazo Abierto de un Robot Omnidireccional con ROS
Tarea 1 modulo Robot Operating System

En este proyecto se destacan los siguientes archivos generados correspondientes a los nodos en python y al nodo del simulador de Coppelia en lua.

En **src/paquete_ros/src/** se encuentran los nodos:

**teleop.py**: Script en python que corresponde a Nodo 1: Nodo de Teclado.

**controller.py**: Script en python que corresponde a Nodo 2: Nodo Principal del Robot.

El archivo de configuración de nodos **src/paquete_ros/CMakeLists.txt**.

En **simulation_vrep/** se encuentran los archivos asociados al simulador de Coppelia:

**entorno.ttt**: Scene que se carga en Coppelia con el móvil Pioner p3dx.

**main.lua**: Script en lua que corresponde a Nodo 3: Nodo del Simulador Coppelia.


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
> Debido a que tengo problemas para guardar los cambios al crear una escena, se utiliza el robot *Pioner p3dx*, al cual se le debe cargar el script de comunicación con ROS que se deja a disposición en la ruta **~/Documents/catkin_taller1_ws/simulation_vrep/main.lua**. Los problemas mencionados corresponde a que se me bloquea la máquina virtual al tratar de generar o manipular una escena que involucre al KUKA youBot, y además, la licencia de Coppelia solicitada que se me suministró, no es válida.

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

- Terminales con los nodos y coppelia en funcionamiento al mismo tiempo

![Pantallazo 1](https://github.com/ricardo-aspee/ros-diaa/blob/master/pantallazo1.jpg?raw=true)

- Terminal del Nodo 1 de captura de teclado

![Pantallazo 3](https://github.com/ricardo-aspee/ros-diaa/blob/master/pantallazo3.jpg?raw=true)

- Terminal del Nodo 2 Controller

![Pantallazo 2](https://github.com/ricardo-aspee/ros-diaa/blob/master/pantallazo2.jpg?raw=true)


