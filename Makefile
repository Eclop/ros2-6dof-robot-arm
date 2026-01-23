# === CONFIGURACIÓN ===
SHELL := /bin/bash
PKG_NAME = my_robot_arm
MOVEIT_PKG = my_robot_arm_moveit_config
LAUNCH_FILE = display.launch.py
CONTAINER_NAME = ros-humble


.PHONY: start build clean rebuild launch deps check moveit-setup moveit-demo moveit-clean
# === COMANDOS GENERALES ===

# 0. Iniciar contenedor (Distrobox)
start:
	distrobox enter $(CONTAINER_NAME) -- /bin/bash

# 1. Construir (Build normal)
build:
	colcon build --symlink-install

# 2. Limpieza de compilación (Borra build/install)
clean:
	rm -rf build install log
	@echo "Limpieza de compilación completada."

# 3. Reconstrucción Total
rebuild: clean build

# 4. Lanzar visualización básica (Solo URDF en Rviz)
launch:
	source install/setup.bash && ros2 launch $(PKG_NAME) $(LAUNCH_FILE)

# 5. Instalar dependencias
deps:
	rosdep install -i --from-path src --rosdistro humble -y

# 6. Debug de archivos
check:
	@echo "--- Archivos instalados en share ---"
	ls -R install/$(PKG_NAME)/share/$(PKG_NAME)

# === COMANDOS MOVEIT ===

# 7. Abrir MoveIt Setup Assistant
# Úsalo para generar o editar la configuración
moveit-setup:
	source install/setup.bash && ros2 launch moveit_setup_assistant setup_assistant.launch.py

# 8. Lanzar Demo de MoveIt (El brazo con la bola interactiva)
# Este es el que usarás para probar si planifica movimiento
moveit-demo:
	source install/setup.bash && ros2 launch $(MOVEIT_PKG) demo.launch.py

# 9. Borrar configuración de MoveIt (Opción Nuclear)
# Úsalo si el paquete de configuración se corrompe y quieres empezar de cero.
# ADVERTENCIA: Borra la carpeta src/my_robot_arm_moveit_config y hace un clean.
moveit-clean:
	rm -rf src/$(MOVEIT_PKG)
	rm -rf build install log
	@echo "--- PAQUETE MOVEIT ELIMINADO ---"
	@echo "Ejecuta 'make build' y luego 'make moveit-setup' para regenerarlo."