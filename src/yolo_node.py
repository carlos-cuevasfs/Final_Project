import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.estado_activo = False

        self.subscription = self.create_subscription(
            String,
            'orden_yolo',
            self.callback_orden,
            10
        )

        self.status_subscription = self.create_subscription(
            String,
            'estado_controlpd',
            self.callback_estado_controlpd,
            10
        )

        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.point_pub = self.create_publisher(Point, 'object_center', 10)

        self.model_path = os.path.join(os.path.dirname(__file__), 'yolov11_custom_ncnn_model')
        self.threshold = 0.5
        self.user_res = "640x480"
        self.source_type = 'usb'
        self.usb_idx = 0

        self.cap = None
        self.model = None
        self.resW = 640
        self.resH = 480
        self.origen_y_desplazado = self.resH // 2 + 50  # 👉 Desplazado 50 píxeles hacia abajo

        self.frame_rate_buffer = []
        self.fps_avg_len = 200

        self.get_logger().info("🕐 Nodo YOLO en espera de activación del FSM...")

    def callback_orden(self, msg):
        if msg.data == "activar yolo_node" and not self.estado_activo:
            self.get_logger().info("🚀 Activación recibida desde FSM. Iniciando detección YOLO...")
            self.estado_activo = True
            self.init_model()
            self.init_camera()
            self.main_loop()

    def callback_estado_controlpd(self, msg):
        if msg.data == 'control_completado' and self.estado_activo:
            self.get_logger().info("🛑 Mensaje 'control_completado' recibido. Poniendo nodo YOLO en espera.")
            self.estado_activo = False
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            cv2.destroyAllWindows()
            self.get_logger().info("🕐 Nodo YOLO en espera de activación del FSM...")

    def init_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            rclpy.shutdown()
        self.model = YOLO(self.model_path, task='detect')
        self.labels = self.model.names
        self.get_logger().info('YOLO model loaded.')

    def init_camera(self):
        resW, resH = map(int, self.user_res.split('x'))
        self.cap = cv2.VideoCapture(self.usb_idx)
        self.cap.set(3, resW)
        self.cap.set(4, resH)
        self.resW = resW
        self.resH = resH
        self.origen_y_desplazado = self.resH // 2 + 50
        self.get_logger().info('Camera initialized.')

    def main_loop(self):
        bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
                       (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

        avg_frame_rate = 0

        while rclpy.ok() and self.estado_activo:
            t_start = time.perf_counter()

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().error('No se pudo capturar el frame de la cámara.')
                break

            results = self.model(frame, verbose=False)
            detections = results[0].boxes
            object_count = 0
            class_names = []

            for det in detections:
                xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
                xmin, ymin, xmax, ymax = xyxy
                classidx = int(det.cls.item())
                classname = self.labels[classidx]
                conf = det.conf.item()

                if conf > self.threshold:
                    color = bbox_colors[classidx % 10]
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    label = f'{classname}: {int(conf*100)}%'
                    cv2.putText(frame, label, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    center_x_pixel = (xmin + xmax) // 2
                    center_y_pixel = (ymin + ymax) // 2

                    center_x = center_x_pixel - (self.resW // 2)
                    center_y = self.origen_y_desplazado - center_y_pixel  # Nuevo origen Y

                    # Dibujar puntos
                    cv2.circle(frame, (center_x_pixel, center_y_pixel), 6, (0, 255, 0), -1)  # Punto verde
                    cv2.circle(frame, (self.resW // 2, self.origen_y_desplazado), 4, (0, 0, 255), -1)  # Punto rojo

                    pt = Point()
                    pt.x = float(center_x)
                    pt.y = float(center_y)
                    pt.z = 0.0
                    self.point_pub.publish(pt)

                    object_count += 1
                    class_names.append(f"{classname}({center_x},{center_y})")

            msg = String()
            msg.data = f"Detectados: {object_count} objeto(s): {', '.join(class_names)}"
            self.publisher_.publish(msg)

            cv2.putText(frame, msg.data, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            cv2.imshow('YOLO Detection (ROS 2)', frame)

            if cv2.waitKey(5) in [ord('q'), 27]:
                break

            t_stop = time.perf_counter()
            frame_rate_calc = float(1 / (t_stop - t_start))
            self.frame_rate_buffer.append(frame_rate_calc)
            if len(self.frame_rate_buffer) > self.fps_avg_len:
                self.frame_rate_buffer.pop(0)
            avg_frame_rate = np.mean(self.frame_rate_buffer)

        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()
        self.get_logger().info(f'FPS promedio: {avg_frame_rate:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
