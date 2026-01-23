# Brazo Robótico 6-DOF (ROS 2 Humble)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Humble-purple)
![License](https://img.shields.io/badge/License-Apache_2.0-green)
![Build](https://img.shields.io/badge/Build-Colcon-orange)

Paquete de simulación y descripción para un brazo robótico de 6 grados de libertad (6-DOF), diseñado desde cero para **ROS 2 Humble** con integración completa de **MoveIt 2** para planificación de movimiento.

![Vista del Robot en RViz](preview.png)

## ✨ Características

* **Descripción URDF/Xacro:** Modelo cinemático completo con límites de articulación configurados.
* **Integración ros2_control:** Hardware interface configurado para simulación con `mock_components/GenericSystem`.
* **MoveIt 2:** Configuración completa para planificación de movimiento, incluyendo:
    * Grupo de planificación `arm` con los 5 joints activos
    * Cinemática inversa con KDL
    * Controladores de trayectoria configurados
* **Mallas Personalizadas:**
    * Visuales y de colisión optimizadas
    * Código de colores para fácil identificación de eslabones
* **Fuente CAD:** Archivo original de **FreeCAD** (`.FCStd`) en `cad/`.
* **Automatización:** `Makefile` con comandos para compilar, visualizar y usar MoveIt.

---

## 🚀 Uso

### Compilar el proyecto

```bash
make build
```

### Visualización básica (Solo URDF)

```bash
make launch
```

Abre **RViz** con el modelo del robot y el panel **Joint State Publisher GUI** para mover las articulaciones manualmente.

### 🤖 MoveIt 2 - Planificación de Movimiento

#### Lanzar Demo de MoveIt

```bash
make moveit-demo
```

Abre RViz con el plugin de MoveIt, permitiendo:
- Planificar trayectorias con el marcador interactivo (bola naranja)
- Ejecutar movimientos planificados
- Visualizar estados de objetivo y trayectorias

#### Abrir MoveIt Setup Assistant

```bash
make moveit-setup
```

Útil para regenerar o modificar la configuración de MoveIt.

---

## 📋 Comandos del Makefile

| Comando | Descripción |
|---------|-------------|
| `make build` | Compila con `colcon build --symlink-install` |
| `make clean` | Elimina `build/`, `install/` y `log/` |
| `make rebuild` | `clean` + `build` |
| `make launch` | Visualización básica del URDF en RViz |
| `make deps` | Instala dependencias con `rosdep` |
| `make check` | Muestra archivos instalados |
| **MoveIt** | |
| `make moveit-demo` | Lanza demo de MoveIt con planificación |
| `make moveit-setup` | Abre MoveIt Setup Assistant |
| `make moveit-clean` | Elimina configuración de MoveIt (⚠️ destructivo) |

> **Nota:** El comando `make start` solo es necesario si usas Distrobox.

---

## 📂 Estructura del Proyecto

```text
.
├── Makefile                         # Comandos automatizados
├── cad/                             # Archivos fuente de diseño (FreeCAD)
└── src/
    ├── my_robot_arm/                # Paquete principal ROS 2
    │   ├── launch/                  # display.launch.py
    │   ├── meshes/                  # Archivos STL
    │   └── urdf/                    # arm.urdf.xacro (con ros2_control)
    │
    └── my_robot_arm_moveit_config/  # Configuración MoveIt 2
        ├── config/
        │   ├── joint_limits.yaml    # Límites de velocidad/aceleración
        │   ├── kinematics.yaml      # Solver cinemático (KDL)
        │   ├── my_robot_arm.srdf    # Descripción semántica del robot
        │   └── ros2_controllers.yaml # Configuración de controladores
        └── launch/                  # Archivos de lanzamiento MoveIt
```

---

## 🔧 Articulaciones (Joints)

| Joint | Tipo | Rango (rad) | Descripción |
|-------|------|-------------|-------------|
| `base_to_waist` | Revolute | -3.14 a 3.14 | Rotación de la base |
| `waist_to_shoulder` | Revolute | 0 a 3.14 | Elevación del hombro |
| `shoulder_to_elbow` | Revolute | -1.24 a 2.0 | Flexión del codo |
| `elbow_to_elbow` | Revolute | -2.5 a 2.5 | Rotación del antebrazo |
| `elbow_to_hand` | Revolute | -1.98 a 1.98 | Flexión de la muñeca |
| `hand_to_tool` | Fixed | — | Montaje de herramienta |

---

## 🔗 Créditos y Referencias 3D

La geometría de este robot está basada en un diseño open-source obtenido de GrabCAD:

* **Modelo Original:** Simple Robotic Arm 6 Axes
* **Autor:** Kevin Putra Adiwijna
* **Fuente:** https://grabcad.com/library/simple-robotic-arm-6-axes-1