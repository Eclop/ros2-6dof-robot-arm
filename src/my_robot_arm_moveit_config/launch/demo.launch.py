from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # 1. Construir la configuración de MoveIt
    moveit_config = MoveItConfigsBuilder("my_robot_arm", package_name="my_robot_arm_moveit_config").to_moveit_configs()

    # 2. Generar el LaunchDescription base de la demo (Rviz, etc.)
    ld = generate_demo_launch(moveit_config)

    # 3. Definir los Spawners para los controladores (¡Tus músculos!)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. AGREGAR los spawners al LaunchDescription existente
   # ld.add_action(joint_state_broadcaster_spawner)
    #ld.add_action(arm_controller_spawner)

    # 5. Devolver todo junto
    return ld