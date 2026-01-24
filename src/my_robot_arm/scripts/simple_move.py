#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def go_to_safe_pose(self):
        print("\n INICIANDO MISIÓN DE RESCATE (Joint Space)...")
        print("Intentando levantar el robot a posición vertical segura...")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print(" Error: MoveIt no responde.")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 20
        goal_msg.planning_options.replan = True
        
        # --- DEFINIR LA POSE SEGURA (VERTICAL) ---
        # Base=0, Hombro=1.57 (Arriba), Codos=0 (Rectos)
        # Ajusta estos nombres si difieren en tu URDF
        joint_names = ["base_to_waist", "waist_to_shoulder", "shoulder_to_elbow", "elbow_to_elbow", "elbow_to_hand"]
        target_values = [0.0, 1.57, 0.0, 0.0, 0.0] 

        # Crear constraints combinados
        jc_list = []
        for name, val in zip(joint_names, target_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.05 # Tolerancia amplia para facilitar el plan
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            jc_list.append(jc)
        
        # Agregar al mensaje
        constraints = Constraints()
        constraints.name = "safe_pose"
        constraints.joint_constraints = jc_list
        goal_msg.request.goal_constraints = [constraints]

        # Enviar
        print("Enviando comando de rescate...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(" Meta rechazada.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            print("¡ROBOT DESATASCADO! Ahora está en posición segura.")
            return True
        else:
            print(f" Fallo el rescate con código: {result.error_code.val}")
            return False

    def go_to_point(self, x, y, z):
        print(f"\n Intentando ir a coordenada Cartesiana: X={x}, Y={y}, Z={z}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 10.0
        
        pcm = PositionConstraint()
        pcm.header.frame_id = 'world'
        pcm.link_name = 'tool_link'
        pcm.weight = 1.0
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05] 
        pcm.constraint_region.primitives.append(box)
        
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.w = 1.0 # Cuaternión válido dummy
        
        pcm.constraint_region.primitive_poses.append(target_pose)
        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm]))
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(" Meta rechazada.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            print("¡LLEGADA EXITOSA AL PUNTO!")
        else:
            print(f" Falló movimiento cartesiano: {result.error_code.val}")

def main():


    puntos = [
    (0.2, 0.1, 0.3),
    (0.2, -0.1, 0.3),
    (0.2, -0.1, 0.4),
    (0.2, 0.1, 0.4)
    ]
    rclpy.init()
    mover = SimpleMover()
    
    # 1. PRIMERO: Salir del suelo (Pose segura)
    exito = mover.go_to_safe_pose()
    
    # 2. SEGUNDO: Si salió, ir al punto deseado
    if exito:
        import time
        time.sleep(2) # Darle tiempo de estabilizarse
        for p in puntos:
            mover.go_to_point(p[0], p[1], p[2])
            time.sleep(1)
    else:
        print("\n No se pudo rescatar al robot. Intenta moverlo manualmente en Rviz primero.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()