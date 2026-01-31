import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Cargar configuración de MoveIt
    moveit_config = MoveItConfigsBuilder("my_robot_arm", package_name="my_robot_arm_moveit_config").to_moveit_configs()

    # --- CORRECCIÓN: Buscar el archivo YAML manualmente ---
    # Obtenemos la ruta absoluta al archivo ros2_controllers.yaml
    ros2_controllers_path = os.path.join(
        get_package_share_directory("my_robot_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )
    # -----------------------------------------------------

    # 2. NODO: Robot State Publisher
    # Publica el URDF en /robot_description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 3. NODO: ROS2 Control (El Músculo)
    # Aquí pasamos la ruta manual que calculamos arriba
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path], # <--- USO DE LA RUTA MANUAL
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # 4. NODO: Spawners (Activadores)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 5. NODO: Move Group (El Cerebro)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # 6. NODO: Rviz (Visualización)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("my_robot_arm_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 7. Retrasar Spawners para evitar conflictos
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller, spawn_gripper_controller],
        )
    )

    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        move_group_node,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        rviz_node,
    ])