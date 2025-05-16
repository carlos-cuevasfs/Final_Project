import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Constantes
TOPIC_ORDEN_REDIRECCIONAR = 'orden_redireccionar'
TOPIC_COMANDO_SERIAL = 'comando_serial'
TOPIC_RESULTADO_REDIRECCION = 'resultado_redireccion'
COMANDO_ACTIVAR = 'activar redireccionar_node'

class RedireccionarNode(Node):
    def __init__(self):
        super().__init__('redireccionar_node')

        # Suscripción al tópico de orden
        self.subscription = self.create_subscription(
            String,
            TOPIC_ORDEN_REDIRECCIONAR,
            self.callback_orden,
            10
        )

        # Publicadores
        self.publisher_comando = self.create_publisher(String, TOPIC_COMANDO_SERIAL, 10)
        self.publisher_resultado = self.create_publisher(String, TOPIC_RESULTADO_REDIRECCION, 10)

        self.get_logger().info("🕐 redireccionar_node listo y en espera de FSM...")

    def callback_orden(self, msg):
        if msg.data == COMANDO_ACTIVAR:
            self.get_logger().info("🚀 Activación recibida. Iniciando redirección...")

            # Esperar 1 segundo
            time.sleep(1)

            # Enviar comando para avanzar
            self.enviar_comando_serial("Avanzar")

            # Avanzar durante 3 segundos
            time.sleep(3)

            # Detener el robot
            self.enviar_comando_serial("Detener")

            # Notificar al FSM que la redirección ha finalizado
            self.publicar_resultado("REDIRECCION_COMPLETADA")

            self.get_logger().info("✅ Redirección completada. Nodo listo para nueva activación.")

    def enviar_comando_serial(self, comando):
        msg = String()
        msg.data = comando
        self.publisher_comando.publish(msg)
        self.get_logger().info(f"📤 Enviado a pyserial_node: {comando}")

    def publicar_resultado(self, resultado):
        msg = String()
        msg.data = resultado
        self.publisher_resultado.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RedireccionarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
