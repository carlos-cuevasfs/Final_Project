import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.estado_activo = False

        # Suscripción al comando de activación desde fsm_node
        self.subscription = self.create_subscription(
            String,
            'orden_gripper',
            self.callback_orden,
            10
        )

        # Publicador al tópico de comandos seriales
        self.cmd_pub = self.create_publisher(String, 'comando_serial', 10)

        self.get_logger().info("🛑 Nodo Gripper en espera de activación del FSM...")

    def callback_orden(self, msg):
        if msg.data == 'activar gripper_node' and not self.estado_activo:
            self.estado_activo = True
            self.get_logger().info("🤖 Brazo en ejecución: Agarrando lata...")

            # Enviar comando 'Brazo' a Arduino
            comando = String()
            comando.data = "Brazo"
            self.cmd_pub.publish(comando)
            self.get_logger().info("📤 Comando 'Brazo' enviado a Arduino.")

            # Esperar 20 segundos
            time.sleep(20)

            self.get_logger().info("✅ Acción completada. Nodo Gripper en espera de nueva activación.")
            self.estado_activo = False

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
