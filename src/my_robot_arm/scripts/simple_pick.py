#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_joint_goal(self, group_name, joint_names, target_values):
        """
        Función genérica para mover cualquier grupo (brazo o gripper)
        usando valores de articulaciones directos.
        """
        print(f"\n--- Moviendo grupo: {group_name} ---")
        
        # 1. Esperar al servidor
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("Error: MoveIt no responde.")
            return False

        # 2. Construir el mensaje de meta
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.group_name = group_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10
        
        # 3. Crear las restricciones (Joint Constraints)
        jc_list = []
        for name, val in zip(joint_names, target_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            jc_list.append(jc)
        
        constraints = Constraints()
        constraints.joint_constraints = jc_list
        goal_msg.request.goal_constraints = [constraints]

        # 4. Enviar y esperar
        print("Enviando trayectoria...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print("Meta rechazada por el servidor.")
            return False

        print("Ejecutando movimiento...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            print("¡Movimiento completado con éxito!")
            return True
        else:
            print(f"Fallo con código de error: {result.error_code.val}")
            return False

    # --- FUNCIONES ESPECÍFICAS PARA TU ROBOT ---

    def move_arm_to_pose(self, pose_name):
        # Nombres de tus joints del brazo
        arm_joints = ["base_to_waist", "waist_to_shoulder", "shoulder_to_elbow", "elbow_to_elbow", "elbow_to_hand"]
        
        # Definir algunas poses pre-grabadas (Valores en radianes)
        poses = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0],
            "vertical": [0.0, 1.57, 0.0, 0.0, 0.0],
            "ready_to_grab": [0.0, 0.5, 1.0, -1.0, 1.57] # Aproximación (ajustar según tu geometría)
        }

        if pose_name in poses:
            return self.send_joint_goal("arm", arm_joints, poses[pose_name])
        else:
            print(f"Pose '{pose_name}' no definida.")
            return False

    def control_gripper(self, state):
        # Nombre del joint conductor del gripper
        gripper_joints = ["gripper_to_finger_right"]
        
        if state == "open":
            # 0.04 metros = 4 cm abierto (tu límite superior)
            val = [0.04]
        elif state == "close":
            # 0.0 metros = Cerrado
            val = [0.00]
        else:
            return False
            
        return self.send_joint_goal("gripper", gripper_joints, val)

def main():
    rclpy.init()
    bot = RobotController()
    
    import time

    try:
        # SECUENCIA DE AGARRE
        
        # 1. Asegurar que el gripper está abierto
        print("\nPASO 1: Abrir Gripper")
        bot.control_gripper("open")
        time.sleep(1)

        # 2. Mover el brazo a posición de "Agarre"
        print("\nPASO 2: Ir a posición de agarre")
        bot.move_arm_to_pose("ready_to_grab")
        time.sleep(1)

        # 3. Cerrar Gripper (Agarrar objeto imaginario)
        print("\nPASO 3: Cerrar Gripper")
        bot.control_gripper("close")
        time.sleep(1)

        # 4. Levantar brazo (Home/Vertical)
        print("\nPASO 4: Levantar objeto")
        bot.move_arm_to_pose("vertical")
        
    except KeyboardInterrupt:
        print("Cancelado por usuario.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()