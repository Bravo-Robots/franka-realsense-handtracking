# Franka Hand Tracking

Este repositorio tiene como objetivo controlar el robot Franka Emika FR3 utilizando hand tracking de la mano derecha. El sistema emplea una cámara Intel RealSense y OpenCV para detectar los gestos y obtener las coordenadas de la mano respecto a la cámara. Estas coordenadas y gestos se utilizan para teleoperar el robot en tiempo real.

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

La adaptación permite utilizar el hand tracking y teleoperación con el robot Franka FR3, integrando la detección de gestos y posiciones de la mano para controlar el robot de manera intuitiva.

## Componentes principales
- **Hand Tracking:** Utiliza OpenCV y una cámara RealSense para detectar la mano y extraer sus gestos y coordenadas.
- **Integración con Franka FR3:** El sistema traduce los gestos y posiciones detectados en comandos para el robot Franka FR3 usando ROS 2.

## Uso

1. Instala las dependencias y sigue la guía de instalación de [franka_ros2](https://github.com/frankaemika/franka_ros2).
2. Clona este repositorio y compílalo en tu workspace de ROS 2.
```sh
git clone
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
