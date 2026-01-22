# 🤖 Brazo Robótico 6-DOF (ROS 2 Humble)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![License](https://img.shields.io/badge/License-Apache_2.0-green)
![Build](https://img.shields.io/badge/Build-Colcon-orange)

Paquete de simulación y descripción para un brazo robótico de 6 grados de libertad (6-DOF), diseñado desde cero para **ROS 2 Humble**.

El proyecto incluye la descripción URDF completa, mallas visuales y de colisión optimizadas, y los archivos fuente de diseño CAD.


## ✨ Características

* **Descripción URDF/Xacro:** Modelo cinemático completo con límites de articulación (joint limits) configurados para evitar autocolisiones.
* **Mallas Personalizadas:**
    * Visuales: Exportadas en escala precisa (mm a m).
    * Colisiones: Geometría optimizada para planificadores de movimiento.
    * Código de colores para fácil identificación de eslabones (Links).
* **Fuente CAD:** Incluye el archivo original de **FreeCAD** (`.FCStd`) en la carpeta `cad/`.
* **Automatización:** Incluye un `Makefile` para facilitar la compilación y ejecución.

## 📂 Estructura del Proyecto

```text
.
├── Makefile                # Atajos para compilar y lanzar
├── cad/                    # Archivos fuente de diseño (FreeCAD)
└── src/my_robot_arm/       # Paquete ROS 2
    ├── launch/             # Archivos de lanzamiento (display.launch.py)
    ├── meshes/             # Archivos STL (Visual & Collision)
    └── urdf/               # Descripción del robot (.xacro)

```

## 🔗 Créditos y Referencias 3D
La geometría de este robot está basada en un diseño open-source obtenido de GrabCAD. 
* **Modelo Original:** Simple Robotic Arm 6 Axes
* **Autor:** Kevin Putra Adiwijna
* **Fuente:** https://grabcad.com/library/simple-robotic-arm-6-axes-1