import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time

class ControlPDNode(Node):
    def __init__(self):
        super().__init__('control_pd_node')

        self.subscription = self.create_subscription(
            String,
            'orden_controlpd',
            self.callback_orden,
            10
        )
        self.subscriber = None
        self.cmd_pub = self.create_publisher(String, 'comando_serial', 10)
        self.status_pub = self.create_publisher(String, 'estado_controlpd', 10)

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.last_time = time.time()

        self.kp_x = 0.00015
        self.kd_x = 0.0002
        self.kp_y = 0.00015
        self.kd_y = 0.0002

        self.tolerance = 40
        self.activo = False
        self.alineado_x = False
        self.alineado_y = False
        self.inicio_y = None
        self.inicio_estabilidad = None

        self.get_logger().info("🕹️ Nodo control PD inicializado. Esperando activación desde fsm_node...")

    def callback_orden(self, msg):
        if msg.data == 'activar controlpd_node' and not self.activo:
            self.get_logger().info("⏳ Activación recibida. Esperando 5 segundos antes de iniciar...")
            time.sleep(5)
            self.activo = True
            self.last_time = time.time()
            self.get_logger().info("✅ Control PD activado. Iniciando alineación en el eje X.")
            self.subscriber = self.create_subscription(
                Point,
                'object_center',
                self.callback_control,
                10
            )

    def callback_control(self, msg: Point):
        if not self.activo:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt == 0:
            dt = 1e-6  # Previene división por cero

        x = msg.x
        y = msg.y

        error_x = x
        d_error_x = (error_x - self.prev_error_x) / dt
        control_x = self.kp_x * error_x + self.kd_x * d_error_x
        self.prev_error_x = error_x

        error_y = y
        d_error_y = (error_y - self.prev_error_y) / dt
        control_y = self.kp_y * error_y + self.kd_y * d_error_y
        self.prev_error_y = error_y

        comando = String()

        # Fase 1: Alineación en X
        if not self.alineado_x:
            if abs(error_x) > self.tolerance:
                comando.data = 'RotarDerechados' if control_x > 0 else 'RotarIzquierda'
                self.inicio_y = None  # Reiniciar temporizador de Y
            else:
                comando.data = 'Detener'
                if self.inicio_y is None:
                    self.inicio_y = time.time()
                    self.get_logger().info("🎯 Alineación X completa. Esperando 1 segundo para iniciar Y...")
                elif time.time() - self.inicio_y >= 1.0:
                    self.alineado_x = True
                    self.get_logger().info("▶️ Iniciando alineación en el eje Y.")
        else:
            # Verificar si X se ha desalineado
            if abs(error_x) > self.tolerance:
                comando.data = 'RotarDerechados' if control_x > 0 else 'RotarIzquierda'
                self.alineado_x = False
                self.alineado_y = False
                self.inicio_y = None
                self.inicio_estabilidad = None
                self.get_logger().info("⚠️ Desalineación en X detectada. Reiniciando alineación en X.")
            else:
                # Fase 2: Alineación en Y
                if not self.alineado_y:
                    if abs(error_y) > self.tolerance:
                        comando.data = 'Reversa' if control_y < 0 else 'Avanzardos'
                        self.inicio_estabilidad = None  # Reiniciar temporizador de estabilidad
                    else:
                        comando.data = 'Detener'
                        self.alineado_y = True
                        self.inicio_estabilidad = time.time()
                        self.get_logger().info("🎯 Alineación Y completa. Verificando estabilidad por 3 segundos.")
                else:
                    # Verificar estabilidad
                    if abs(error_x) > self.tolerance or abs(error_y) > self.tolerance:
                        self.alineado_x = False
                        self.alineado_y = False
                        self.inicio_y = None
                        self.inicio_estabilidad = None
                        self.get_logger().info("⚠️ Desalineación detectada durante verificación de estabilidad. Reiniciando alineación.")
                    elif time.time() - self.inicio_estabilidad >= 3.0:
                        comando.data = 'Detener'
                        self.get_logger().info("✅ Control completado. Publicando mensaje al FSM.")
                        status_msg = String()
                        status_msg.data = 'control_completado'
                        self.status_pub.publish(status_msg)
                        # Reiniciar estado
                        self.activo = False
                        self.alineado_x = False
                        self.alineado_y = False
                        self.inicio_y = None
                        self.inicio_estabilidad = None
                        self.prev_error_x = 0.0
                        self.prev_error_y = 0.0
                        self.last_time = time.time()
                        self.get_logger().info("🕹️ Nodo control PD inicializado. Esperando activación desde fsm_node...")
                        return
                    else:
                        comando.data = 'Detener'

        self.cmd_pub.publish(comando)
        self.get_logger().info(f"📤 Comando: {comando.data} | Error X: {error_x:.2f} | Error Y: {error_y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlPDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
