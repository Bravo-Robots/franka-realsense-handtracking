# Franka Hand Tracking

Este repositorio tiene como objetivo controlar el robot Franka Emika FR3 utilizando hand tracking de la mano derecha comunicando los nodos por medio de ROS2 humble. El sistema emplea una cámara Intel RealSense y OpenCV para detectar los gestos y obtener las coordenadas de la mano respecto a la cámara. Estas coordenadas y gestos se utilizan para teleoperar el robot en tiempo real.

## Instalación de dependencias principales

Antes de usar este repositorio, es necesario instalar el stack de ROS 2 para Franka. Puedes seguir las instrucciones oficiales aquí:

https://github.com/frankaemika/franka_ros2

## Paquetes principales

* Media Pipe:
```sh
pip install mediapipe
```
* RealSense Camera
    * follow setup instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development

## Descripción

Este proyecto es una adaptación del sistema de hand tracking del siguiente repositorio:

https://github.com/gjcliff/FrankaTeleop.git

La adaptación permite utilizar el hand tracking y teleoperación con el robot Franka FR3 utilizando el moveit y controlador bàsico de Franka FR3, integrando la detección de gestos y posiciones de la mano para controlar el robot de manera intuitiva.

## Gestos reconocidos y su función

A continuación se muestra una lista de los gestos que el sistema reconoce y la acción que realiza cada uno:

- **Pulgar Arriba (Thumbs Up - Iniciar/Detener Tracking):** Este gesto se utiliza para indicar al sistema que debe comenzar o detener el seguimiento de la posición de tu mano derecha. También puedes usar este gesto para ajustar la posición de tu mano en el encuadre de la cámara sin mover el robot. Mientras la cámara detecta el gesto de pulgar arriba, el robot no se moverá, pero una vez que retires la mano, el robot comenzará a seguir la posición de tu mano.
- **Pulgar Abajo (Thumbs Down - Apagar):** Este gesto se utiliza para indicar al sistema que debe dejar de seguir tu mano hasta que vuelva a detectar el pulgar arriba.
- **Puño Cerrado (Close Fist - Cerrar Gripper):** Este gesto cerrará la pinza (gripper) del robot.
- **Palma Abierta (Open Palm - Abrir Gripper):** Este gesto abrirá la pinza (gripper) del robot.

## Nodos principales
- **handcv:** Utiliza OpenCV y una cámara RealSense para detectar la mano y extraer sus gestos y coordenadas.
- **cv_franka_bridge:** El sistema traduce los gestos y posiciones detectados en comandos para el robot Franka FR3 usando ROS 2.

## Uso

1. Instala las dependencias y sigue la guía de instalación de [franka_ros2](https://github.com/frankaemika/franka_ros2).
2. Clona este repositorio y compílalo en tu workspace de ROS 2.
```sh
git clone https://github.com/Bravo-Robots/franka-realsense-handtracking.git
cd ~/Franka_hand_tracking
colcon build --symlink-install
source install/setup.bash
```
3. Para lanzar los nodos y comenzar la teleoperación, corre el siguiente comando launch.
* Para teleoperaciòn con robot real:
```sh
ros2 launch cv_franka_bridge integrate_servo.launch.py   use_fake_hardware:=false   robot_ip:=192.168.1.11   use_rviz:=true   use_realsense:=true   run_franka_teleop:=true
```
* Para teleoperaciòn en simulaciòn con rviz:
```sh
ros2 launch cv_franka_bridge integrate_servo.launch.py   use_fake_hardware:=true   robot_ip:=dont-care   use_rviz:=true   use_realsense:=true   run_franka_teleop:=true
```
---
