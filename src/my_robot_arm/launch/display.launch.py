import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_arm'
    file_subpath = 'urdf/arm.urdf.xacro'

    # 1. Buscar y procesar el archivo del robot
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Configurar los nodos (programas) que vamos a lanzar
    return LaunchDescription([
        # Publica el estado del robot (tf estáticos)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        # Abre la ventanita con los sliders para mover las articulaciones
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Abre Rviz (el visualizador 3D)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])