# === CONFIGURACIÓN ===
SHELL := /bin/bash
PKG_NAME = my_robot_arm
LAUNCH_FILE = display.launch.py
CONTAINER_NAME = ros-humble

# === COMANDOS ===

#0. Iniciar el contenedor (Solo para Distrobox)
#   NOTA: Si estás en Ubuntu 22.04 con ROS2 Humble instalado nativamente,
#         NO necesitas ejecutar este comando. Puedes usar directamente
#         make build, make launch, etc.
start:
	distrobox enter $(CONTAINER_NAME) -- /bin/bash

# 1. Construir (Build normal)
# Usa symlink para que no tengas que reconstruir si cambias solo Python/XML/Launch
build:
	colcon build --symlink-install

# 2. Limpieza Nuclear (Borra caché)
# Úsalo cuando cambies .stl, colores o cosas que ROS no detecta
clean:
	rm -rf build install log
	@echo "Limpieza completada. Ahora ejecuta 'make build'"

# 3. Reconstrucción Total (Clean + Build)
# El comando mágico para arreglar errores raros
rebuild: clean build

# 4. Lanzar el Robot
# Hace el 'source' automáticamente antes de lanzar
launch:
	source install/setup.bash && ros2 launch $(PKG_NAME) $(LAUNCH_FILE)

# 5. Instalar dependencias faltantes
# Útil si te llevas el proyecto a otro PC
deps:
	rosdep install -i --from-path src --rosdistro humble -y

# 6. Ver estructura de archivos (Debug)
# Te muestra si los archivos están realmente en la carpeta de instalación
check:
	@echo "--- Archivos instalados en share ---"
	ls -R install/$(PKG_NAME)/share/$(PKG_NAME)
