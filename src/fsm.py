import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# === Constantes ===
TOPIC_ORDEN_INIT = 'orden_fsm'
TOPIC_RESPUESTA_INIT = 'respuesta_init'
TOPIC_ORDEN_ROTAR360 = 'orden_rotar360'
TOPIC_RESULTADO_ROTACION = 'resultado_rotacion'
TOPIC_ORDEN_APPROACH = 'orden_approach'
TOPIC_RESULTADO_APPROACH = 'resultado_approach'
TOPIC_ORDEN_YOLO = 'orden_yolo'
TOPIC_ORDEN_REDIRECCIONAR = 'orden_redireccionar'
TOPIC_RESULTADO_REDIRECCION = 'resultado_redireccion'
TOPIC_ORDEN_CONTROLPD = 'orden_controlpd'
TOPIC_ESTADO_CONTROLPD = 'estado_controlpd'
TOPIC_ORDEN_GRIPPER = 'orden_gripper'

COMANDO_INICIAR = 'Iniciar'
COMANDO_ACTIVAR_ROTAR360 = 'activar rotar360_node'
COMANDO_ACTIVAR_APPROACH = 'activar approach_object_node'
COMANDO_ACTIVAR_YOLO = 'activar yolo_node'
COMANDO_ACTIVAR_REDIRECCIONAR = 'activar redireccionar_node'
COMANDO_ACTIVAR_CONTROLPD = 'activar controlpd_node'
COMANDO_ACTIVAR_GRIPPER = 'activar gripper_node'

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        self.get_logger().info("🤖 Iniciando E.C.O.B.O.T...")

        self.estado_actual = "init_node"
        self.orden_enviada = False

        # === Subscripciones ===
        self.subscriber_init = self.create_subscription(
            String,
            TOPIC_RESPUESTA_INIT,
            self.callback_respuesta_init,
            10
        )

        self.subscriber_resultado_rotacion = self.create_subscription(
            String,
            TOPIC_RESULTADO_ROTACION,
            self.callback_resultado_rotacion,
            10
        )

        self.subscriber_resultado_approach = self.create_subscription(
            String,
            TOPIC_RESULTADO_APPROACH,
            self.callback_resultado_approach,
            10
        )

        self.subscriber_resultado_redireccion = self.create_subscription(
            String,
            TOPIC_RESULTADO_REDIRECCION,
            self.callback_resultado_redireccion,
            10
        )

        self.subscriber_estado_controlpd = self.create_subscription(
            String,
            TOPIC_ESTADO_CONTROLPD,
            self.callback_estado_controlpd,
            10
        )

        # === Publicadores ===
        self.publisher_orden_init = self.create_publisher(String, TOPIC_ORDEN_INIT, 10)
        self.publisher_rotar360 = self.create_publisher(String, TOPIC_ORDEN_ROTAR360, 10)
        self.publisher_approach = self.create_publisher(String, TOPIC_ORDEN_APPROACH, 10)
        self.publisher_yolo = self.create_publisher(String, TOPIC_ORDEN_YOLO, 10)
        self.publisher_redireccionar = self.create_publisher(String, TOPIC_ORDEN_REDIRECCIONAR, 10)
        self.publisher_controlpd = self.create_publisher(String, TOPIC_ORDEN_CONTROLPD, 10)
        self.publisher_gripper = self.create_publisher(String, TOPIC_ORDEN_GRIPPER, 10)

        # Timer para iniciar FSM
        self.timer = self.create_timer(1.0, self.enviar_orden_init)

    def enviar_orden_init(self):
        if not self.orden_enviada:
            msg = String()
            msg.data = COMANDO_INICIAR
            self.publisher_orden_init.publish(msg)
            self.get_logger().info("Enviando orden de inicio a init_node...")
            self.orden_enviada = True

    def callback_respuesta_init(self, msg):
        if msg.data == "OK" and self.estado_actual == "init_node":
            self.estado_actual = "rotar360_node"
            self.get_logger().info("✅ Estado rotar360_node en ejecución")

            msg_rotar = String()
            msg_rotar.data = COMANDO_ACTIVAR_ROTAR360
            self.publisher_rotar360.publish(msg_rotar)

        elif msg.data == "FALLO":
            self.get_logger().warn("⚠️ Conexión fallida. Reintentando...")

    def callback_resultado_rotacion(self, msg):
        if self.estado_actual == "rotar360_node":
            if msg.data == "OBJ":
                self.get_logger().info("➡️ Estado approach_object_node en ejecución")
                self.estado_actual = "approach_object_node"

                msg_approach = String()
                msg_approach.data = COMANDO_ACTIVAR_APPROACH
                self.publisher_approach.publish(msg_approach)

            elif msg.data == "NOOBJ":
                self.get_logger().info("➡️ Estado redireccionar_node en ejecución")
                self.estado_actual = "redireccionar_node"

                msg_redir = String()
                msg_redir.data = COMANDO_ACTIVAR_REDIRECCIONAR
                self.publisher_redireccionar.publish(msg_redir)

    def callback_resultado_redireccion(self, msg):
        if self.estado_actual == "redireccionar_node" and msg.data == "REDIRECCION_COMPLETADA":
            self.get_logger().info("🔄 Redirección completada. Reintentando rotación...")
            self.estado_actual = "rotar360_node"

            msg_rotar = String()
            msg_rotar.data = COMANDO_ACTIVAR_ROTAR360
            self.publisher_rotar360.publish(msg_rotar)

    def callback_resultado_approach(self, msg):
        if self.estado_actual == "approach_object_node" and msg.data == "OBJETO_ALCANZADO":
            self.get_logger().info("🎯 Objeto alcanzado. Activando yolo_node y controlpd_node.")
            self.estado_actual = "yolo_controlpd_node"

            # Activar yolo_node
            msg_yolo = String()
            msg_yolo.data = COMANDO_ACTIVAR_YOLO
            self.publisher_yolo.publish(msg_yolo)

            # Activar controlpd_node
            msg_controlpd = String()
            msg_controlpd.data = COMANDO_ACTIVAR_CONTROLPD
            self.publisher_controlpd.publish(msg_controlpd)

    def callback_estado_controlpd(self, msg):
        if self.estado_actual == "yolo_controlpd_node" and msg.data == "control_completado":
            self.get_logger().info("Cambio de estado a gripper_node")
            self.estado_actual = "gripper_node"

            msg_gripper = String()
            msg_gripper.data = COMANDO_ACTIVAR_GRIPPER
            self.publisher_gripper.publish(msg_gripper)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
