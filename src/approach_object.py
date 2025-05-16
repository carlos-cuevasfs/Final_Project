import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# === Constantes ===
TOPIC_ORDEN_APPROACH = 'orden_approach'
TOPIC_DISTANCIAS = 'distancias_ultrasonico'
TOPIC_COMANDO_SERIAL = 'comando_serial'
TOPIC_RESULTADO_APPROACH = 'resultado_approach'

COMANDO_ACTIVAR = 'activar approach_object_node'

UMBRAL_INICIAL = 50
UMBRAL_FINAL = 12

class ApproachObjectNode(Node):
    def __init__(self):
        super().__init__('approach_object_node')

        self.estado_activo = False
        self.avanzando = False
        self.recuperando = False
        self.recuperacion_inicio = None
        self.mensaje_enviado = False

        self.subscription_orden = self.create_subscription(
            String,
            TOPIC_ORDEN_APPROACH,
            self.callback_orden,
            10
        )

        self.subscription_distancias = self.create_subscription(
            String,
            TOPIC_DISTANCIAS,
            self.callback_distancias,
            10
        )

        self.publisher_comando = self.create_publisher(
            String,
            TOPIC_COMANDO_SERIAL,
            10
        )

        self.publisher_resultado = self.create_publisher(
            String,
            TOPIC_RESULTADO_APPROACH,
            10
        )

        self.get_logger().info("🕐 approach_object_node listo en espera del FSM...")

    def callback_orden(self, msg):
        if msg.data == COMANDO_ACTIVAR:
            self.get_logger().info("🚀 Activación recibida desde FSM.")
            self.estado_activo = True
            self.avanzando = False
            self.recuperando = False
            self.recuperacion_inicio = None
            self.mensaje_enviado = False
            time.sleep(1)  # Espera para estabilizar sistema

    def callback_distancias(self, msg):
        if not self.estado_activo:
            return

        try:
            distancia = int(msg.data.split(":")[1])
        except Exception:
            self.get_logger().warn(f"⚠️ Error interpretando distancia: {msg.data}")
            return

        # Inicio de movimiento
        if not self.avanzando:
            if distancia < UMBRAL_INICIAL:
                self.enviar_comando("Avanzar")
                self.get_logger().info(f"🚗 Iniciando avance (D: {distancia} cm)")
                self.avanzando = True
            else:
                self.enviar_comando("Detener")
                self.get_logger().info(f"🛑 Sin referencia válida al inicio (D: {distancia} cm)")
                self.estado_activo = False
            return

        # Detección de llegada al objeto
        if distancia <= UMBRAL_FINAL:
            self.enviar_comando("Detener")
            self.get_logger().info(f"🎯 Objeto alcanzado a {distancia} cm")
            self.publicar_resultado("OBJETO_ALCANZADO")
            self.reset_estado()
            return

        # Si se pierde la referencia
        if distancia > UMBRAL_INICIAL:
            if not self.recuperando:
                self.enviar_comando("Detener")
                self.recuperando = True
                self.recuperacion_inicio = time.time()
                self.get_logger().warn(f"🔍 Pérdida de referencia. Esperando recuperación... (D: {distancia})")
            else:
                if time.time() - self.recuperacion_inicio >= 5.0:
                    self.get_logger().error("❌ No se recuperó la referencia. Se detiene el nodo.")
                    self.enviar_comando("Detener")
                    self.reset_estado()
        else:
            if self.recuperando:
                self.enviar_comando("Avanzar")
                self.recuperando = False
                self.recuperacion_inicio = None
                self.get_logger().info(f"✅ Referencia recuperada. Reanudando avance (D: {distancia})")

    def enviar_comando(self, comando):
        msg = String()
        msg.data = comando
        self.publisher_comando.publish(msg)
        self.get_logger().info(f"📤 Enviado a pyserial_node: {comando}")

    def publicar_resultado(self, contenido):
        if not self.mensaje_enviado:
            msg = String()
            msg.data = contenido
            self.publisher_resultado.publish(msg)
            self.get_logger().info(f"📡 Enviado a FSM: {contenido}")
            self.mensaje_enviado = True

    def reset_estado(self):
        self.estado_activo = False
        self.avanzando = False
        self.recuperando = False
        self.recuperacion_inicio = None
        self.mensaje_enviado = False
        self.get_logger().info("🔁 Nodo listo para nueva activación.")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
