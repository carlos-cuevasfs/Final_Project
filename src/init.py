import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import threading

class InitNode(Node):
    def __init__(self):
        super().__init__('init_node')
        self.get_logger().info("Iniciando init_node...")

        self.subscription = self.create_subscription(
            String,
            'orden_fsm',
            self.orden_callback,
            10
        )

        self.publisher = self.create_publisher(String, 'respuesta_init', 10)
        self.ejecucion_activa = False
        self.nodo_finalizado = False  # Bandera para terminar una sola vez

    def orden_callback(self, msg):
        if msg.data == "Iniciar" and not self.ejecucion_activa:
            self.ejecucion_activa = True
            self.get_logger().info("Estado init_node en ejecución")
            thread = threading.Thread(target=self.verificar_dispositivos_usb)
            thread.start()  # Ejecuta en segundo plano para no bloquear spin()

    def verificar_dispositivos_usb(self):
        while rclpy.ok() and not self.nodo_finalizado:
            resultado = subprocess.run(['lsusb'], stdout=subprocess.PIPE, text=True)
            salida = resultado.stdout

            arduino_detectado = "2341:0042" in salida
            webcam_detectada = "32e6:9221" in salida

            if arduino_detectado and webcam_detectada:
                self.get_logger().info("✅ Conexión Arduino y Webcam correcta")
                msg = String()
                msg.data = "OK"
                self.publisher.publish(msg)
                self.nodo_finalizado = True
                time.sleep(1.0)  # Da tiempo a que el mensaje se procese antes de cerrar
                self.get_logger().info("🛑 Finalizando init_node...")
                self.destroy_node()
                rclpy.shutdown()
                break
            else:
                if not arduino_detectado:
                    self.get_logger().warn("⚠️ Arduino no detectado")
                if not webcam_detectada:
                    self.get_logger().warn("⚠️ Webcam no detectada")
                self.get_logger().error("❌ Error: No se detectó Arduino o Webcam")
                msg = String()
                msg.data = "FALLO"
                self.publisher.publish(msg)
                time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = InitNode()
    rclpy.spin(node)
    # En caso de que shutdown ya se haya llamado dentro del nodo:
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
