import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# === Constantes ===
TOPIC_ORDEN_ROTAR360 = 'orden_rotar360'
TOPIC_DISTANCIAS = 'distancias_ultrasonico'
TOPIC_COMANDO_SERIAL = 'comando_serial'
TOPIC_RESULTADO_ROTACION = 'resultado_rotacion'
COMANDO_ACTIVAR = 'activar rotar360_node'
DISTANCIA_UMBRAL_CM = 65
TIEMPO_MAX_ROTACION = 8.0  # segundos

class Rotar360Node(Node):
    def __init__(self):
        super().__init__('rotar360_node')

        # Subscripciones
        self.subscription_orden = self.create_subscription(
            String,
            TOPIC_ORDEN_ROTAR360,
            self.callback_orden,
            10
        )

        self.subscription_distancia = self.create_subscription(
            String,
            TOPIC_DISTANCIAS,
            self.callback_distancia,
            10
        )

        # Publicadores
        self.publisher_comando = self.create_publisher(String, TOPIC_COMANDO_SERIAL, 10)
        self.publisher_resultado = self.create_publisher(String, TOPIC_RESULTADO_ROTACION, 10)

        self.rutina_activa = False
        self.detectado = False
        self.inicio_rotacion = None

        # Timer para timeout
        self.timer_timeout = self.create_timer(0.1, self.verificar_timeout)

        self.get_logger().info("🕐 rotar360_node listo en espera de FSM...")

    def callback_orden(self, msg):
        if msg.data == COMANDO_ACTIVAR and not self.rutina_activa:
            self.get_logger().info("🚀 Activación recibida. Iniciando rotación...")
            self.rutina_activa = True
            self.detectado = False
            self.inicio_rotacion = time.time()

            self.enviar_comando_serial("RotarDerecha")

    def callback_distancia(self, msg):
        if not self.rutina_activa or self.detectado:
            return

        try:
            distancia = int(msg.data.split(":")[1])
        except Exception:
            self.get_logger().warn(f"⚠️ Error interpretando distancia: {msg.data}")
            return

        if distancia <= DISTANCIA_UMBRAL_CM:
            self.enviar_comando_serial("Detener")
            self.detectado = True
            self.get_logger().info(f"🎯 Objeto detectado a {distancia} cm. Publicando OBJ.")
            self.publicar_resultado("OBJ")
            self.reset_estado()

    def verificar_timeout(self):
        if self.rutina_activa and not self.detectado:
            tiempo_transcurrido = time.time() - self.inicio_rotacion
            if tiempo_transcurrido >= TIEMPO_MAX_ROTACION:
                self.enviar_comando_serial("Detener")
                self.get_logger().info("⏰ Tiempo agotado. No se detectó objeto. Publicando NOOBJ.")
                self.publicar_resultado("NOOBJ")
                self.reset_estado()

    def enviar_comando_serial(self, comando):
        msg = String()
        msg.data = comando
        self.publisher_comando.publish(msg)
        self.get_logger().info(f"📤 Enviado a pyserial_node: {comando}")

    def publicar_resultado(self, resultado):
        msg = String()
        msg.data = resultado
        self.publisher_resultado.publish(msg)

    def reset_estado(self):
        self.rutina_activa = False
        self.detectado = False
        self.inicio_rotacion = None
        self.get_logger().info("🔁 rotar360_node listo para nueva activación.")

def main(args=None):
    rclpy.init(args=args)
    node = Rotar360Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
