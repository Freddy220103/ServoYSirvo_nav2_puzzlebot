# ServoYSirvo_nav2_puzzlebot

Este repositorio contiene el desarrollo de un sistema robótico móvil autónomo basado en ROS 2 e Ignition Gazebo. El objetivo del proyecto es diseñar, simular y validar una solución robótica capaz de operar de manera autónoma en un entorno específico, integrando navegación, percepción sensorial y control.

## Objetivo del proyecto

Construir un sistema robótico funcional y autónomo que responda a un reto planteado por un socio formador. El sistema debe utilizar sensores como LIDAR y cámara, ejecutarse sobre ROS 2, y ser validado en simulación antes de su implementación física.

## Funcionalidades actuales

- Publicación de transformaciones TF
- Control cinemático diferencial del robot
- Generación de mapas mediante SLAM Toolbox
- Navegación autónoma utilizando Nav2
- Visualización en tiempo real en RViz
- Simulación de sensores y entorno en Ignition Gazebo

## Estructura del repositorio

- `launch/`: Archivos de lanzamiento para modos de simulación (SLAM y navegación)
- `meshes/`: Modelos 3D del robot
- `puzzlebot_description/`: Modelo del robot en URDF/XACRO
- `puzzlebot_gazebo/`: Archivos de mundo (`.world`) y configuración para simulación
- `config/`: Mapas, configuraciones de RViz y parámetros de navegación
- `scripts/`: Nodos en Python (TF, control, localización, etc.)

## Requisitos

- ROS 2 Humble (u otra versión compatible)
- Ignition Gazebo (Fortress o Garden)
- Paquetes: `nav2_bringup`, `slam_toolbox`, `rviz2`, entre otros

Instalación de dependencias y compilación:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -i -y
colcon build
source install/setup.bash
```

## En desarrollo

- Integración de sensores adicionales
- Ajustes de parámetros de navegación
- Configuración avanzada de mapas y entorno
- Mejora en modelos y detección de obstáculos

## Créditos

Proyecto académico desarrollado por estudiantes del curso TE3003B – Integración de Robótica y Sistemas Inteligentes del Tecnológico de Monterrey.
