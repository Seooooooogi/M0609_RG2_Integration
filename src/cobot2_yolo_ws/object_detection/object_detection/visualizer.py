import json
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

from object_detection.realsense import ImgNode
from object_detection.yolo import YOLO_JSON_PATH, YOLO_MODEL_PATH


COLOR_PALETTE = [
    (255, 56, 56),
    (50, 205, 50),
    (56, 56, 255),
    (255, 159, 56),
    (157, 56, 255),
    (56, 255, 220),
    (255, 220, 56),
    (255, 56, 255),
]


class ImgNodeWithHeader(ImgNode):
    """Cache color frame and header as an atomic tuple to keep stamps consistent."""

    def __init__(self):
        super().__init__()
        self._color_with_header = None

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._color_with_header = (frame, msg.header)
        self.color_frame = frame
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def get_color_with_header(self):
        return self._color_with_header


class YoloVisualizerNode(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')

        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate_hz', 10.0)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        rate_hz = float(self.get_parameter('inference_rate_hz').value)

        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, 'r', encoding='utf-8') as f:
            class_dict = json.load(f)
        self.id_to_name = {int(k): v for k, v in class_dict.items()}

        self.img_node = ImgNodeWithHeader()
        self._img_executor = SingleThreadedExecutor()
        self._img_executor.add_node(self.img_node)
        self._img_thread = threading.Thread(
            target=self._img_executor.spin, daemon=True
        )
        self._img_thread.start()

        self.publisher = self.create_publisher(
            Image,
            '/camera_gripper/camera_gripper/yolo/image_raw',
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._timer_callback)
        self.get_logger().info(
            f"YoloVisualizerNode ready "
            f"(threshold={self.confidence_threshold}, rate={rate_hz} Hz)"
        )

    def _timer_callback(self):
        snapshot = self.img_node.get_color_with_header()
        if snapshot is None:
            return
        frame, header = snapshot

        results = self.model(frame, verbose=False)
        annotated = frame.copy()

        if results:
            res = results[0]
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score < self.confidence_threshold:
                    continue
                self._draw_box(annotated, box, int(label), float(score))

        msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        msg.header = header
        self.publisher.publish(msg)

    def _draw_box(self, img, box, class_id, score):
        x1, y1, x2, y2 = (int(v) for v in box)
        color = COLOR_PALETTE[class_id % len(COLOR_PALETTE)]
        name = self.id_to_name.get(class_id, str(class_id))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        text = f"{name} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(
            img,
            text,
            (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def shutdown(self):
        self._img_executor.shutdown()
        self.img_node.destroy_node()
        self._img_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
