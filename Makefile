# === CONFIGURACIÓN ===
SHELL := /bin/bash
PKG_NAME = my_robot_arm
MOVEIT_PKG = my_robot_arm_moveit_config
LAUNCH_FILE = display.launch.py

# Configuración del entorno NVIDIA
CONTAINER_NAME = ros2_nvidia
IMAGE_URL = docker.io/osrf/ros:humble-desktop-full

.PHONY: start create-container install-libs build clean rebuild launch deps check moveit-setup moveit-demo moveit-clean

# === GESTIÓN DEL CONTENEDOR (NVIDIA) ===

# 0. Iniciar sesión en el contenedor
start:
	distrobox enter $(CONTAINER_NAME) -- /bin/bash

# Nuevo: Crear el contenedor con soporte GPU explícito
create-container:
	distrobox create --name $(CONTAINER_NAME) --image $(IMAGE_URL) --nvidia

# Nuevo: Instalar las herramientas de trabajo (MoveIt, Controladores, etc.)
# Ejecuta esto UNA VEZ dentro del contenedor nuevo.
install-libs:
	sudo apt update && sudo apt upgrade -y
	sudo apt install -y ros-humble-moveit \
		ros-humble-ros2-control \
		ros-humble-ros2-controllers \
		ros-humble-gripper-controllers \
		ros-humble-robot-state-publisher \
		ros-humble-joint-state-publisher-gui
	@echo "--- LIBRERÍAS INSTALADAS CORRECTAMENTE ---"

# === COMANDOS DE DESARROLLO ===

# 1. Construir (Build normal)
build:
	colcon build --symlink-install

# 2. Limpieza de compilación
clean:
	rm -rf build install log
	@echo "Limpieza de compilación completada."

# 3. Reconstrucción Total
rebuild: clean build

# 4. Lanzar visualización básica (Solo URDF en Rviz)
launch:
	source install/setup.bash && ros2 launch $(PKG_NAME) $(LAUNCH_FILE)

# 5. Instalar dependencias del paquete (Rosdep estándar)
deps:
	rosdep install -i --from-path src --rosdistro humble -y

# 6. Debug de archivos
check:
	@echo "--- Archivos instalados en share ---"
	ls -R install/$(PKG_NAME)/share/$(PKG_NAME)

# === COMANDOS MOVEIT ===

# 7. Abrir MoveIt Setup Assistant
moveit-setup:
	source install/setup.bash && ros2 launch moveit_setup_assistant setup_assistant.launch.py

# 8. Lanzar Demo de MoveIt (Con GPU acelerada)
moveit-demo:
	source install/setup.bash && ros2 launch $(MOVEIT_PKG) demo.launch.py

# 9. Borrar configuración de MoveIt (Opción Nuclear)
moveit-clean:
	rm -rf src/$(MOVEIT_PKG)
	rm -rf build install log
	@echo "--- PAQUETE MOVEIT ELIMINADO ---"