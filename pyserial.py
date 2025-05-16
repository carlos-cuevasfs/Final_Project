import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# === Constantes ===
TOPIC_COMANDO_SERIAL = 'comando_serial'
TOPIC_DISTANCIAS = 'distancias_ultrasonico'
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Comandos válidos para Arduino (actualizado)
COMANDOS_VALIDOS = {
    "Avanzar",
    "Reversa",
    "Detener",
    "RotarDerecha",
    "RotarIzquierda",
    "Avanzardos",
    "RotarDerechados",
    "Brazo"  # ✅ Comando del brazo agregado
}

class PySerialNode(Node):
    def __init__(self):
        super().__init__('pyserial_node')

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        # Publicador de distancias
        self.publisher_distancia = self.create_publisher(String, TOPIC_DISTANCIAS, 10)

        # Subscripción a comandos
        self.subscription_comando = self.create_subscription(
            String,
            TOPIC_COMANDO_SERIAL,
            self.callback_comando_serial,
            10
        )

        # Timer para leer datos seriales de Arduino cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.leer_serial)

        self.get_logger().info("🔌 confiemos..... pyserial_node activo y escuchando comandos...")

    def callback_comando_serial(self, msg):
        comando = msg.data.strip()

        if comando in COMANDOS_VALIDOS:
            self.ser.write((comando + "\n").encode())
            self.get_logger().info(f"📤 Enviado a Arduino: {comando}")
        else:
            self.get_logger().warn(f"⚠️ Comando no reconocido: '{comando}'")

    def leer_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("D:"):
                    msg = String()
                    msg.data = line
                    self.publisher_distancia.publish(msg)
                    self.get_logger().info(f"📡 Publicado: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"❌ Error leyendo desde Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PySerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
