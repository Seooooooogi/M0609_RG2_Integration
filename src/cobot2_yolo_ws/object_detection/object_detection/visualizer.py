"""YOLO bbox 시각화 ROS2 노드 (시연용).

RealSense color stream을 받아 YOLOv8로 추론하고, bbox/라벨/score를 그린 이미지를
``CompressedImage`` (JPEG)로 발행한다. ``rqt_image_view`` 에서 토픽 한 줄로 확인 가능.

주요 설계 선택 (성능 보고서: ``docs/yolo_visualizer_performance_report.md``):

* **CompressedImage(JPEG q80) 발행** — raw ``Image``로 발행하면 920 KB/frame을 DDS로
  넘기는 비용이 가끔 200~400 ms씩 튀어 jitter 원인이 됐다. JPEG q80로 60~80 KB까지
  떨어지자 outlier 소멸, jitter std 16 ms → 1 ms로 감소.
* **별도 executor + thread로 ImgNode spin** — 메인 노드의 timer가 추론으로 잠시
  바빠도 frame 수신은 끊기지 않게 한다. timer는 항상 최신 frame을 가져온다.
* **ultralytics 직접 호출** (yolo.py의 ``YoloModel`` 우회) — ``YoloModel`` 은
  ``/get_3d_position`` 서비스 응답용으로 1초간 다중 프레임을 모아 IoU voting하는
  구조라 시연 스트리밍에 부적합.

실행 환경: 호스트 RealSense ↔ Docker(``zium-detection:humble-cu118``) 분리 패턴.
RMW는 양쪽 모두 ``rmw_cyclonedds_cpp`` 일치 필수. 자세한 명령은
``docs/yolo_detection_pipeline.md`` § 7 참조.
"""
import json
import threading

import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from object_detection.realsense import ImgNode
from object_detection.yolo import YOLO_JSON_PATH, YOLO_MODEL_PATH


# class_id → bbox/text 색상. 5개 클래스 + 여유분(랩어라운드).
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
    """Color frame과 header를 원자 튜플로 캐싱하는 ImgNode 서브클래스.

    base ``ImgNode`` 는 frame과 stamp를 별도 변수로 저장하기 때문에, timer가 둘을
    읽는 사이에 새 callback이 끼어들면 frame은 N+1번째인데 header는 N번째인
    mismatched 상태가 될 수 있다. 여기서는 ``(frame, header)`` 한 쌍을 한 번에
    교체해 항상 동일 시점의 데이터를 보장한다.

    또한 visualizer는 color stream만 쓰므로, base 클래스가 만든 depth/aligned_depth
    subscription은 ``__init__`` 에서 제거해 cv_bridge decode + GIL 경합 비용을 차단한다
    (시연 hz 안정에 유의미한 효과 — 별도 재기동 부담 없음).
    """

    def __init__(self):
        super().__init__()
        # 원자 튜플 슬롯. callback이 한 번 들어와야 None이 아니게 됨.
        self._color_with_header = None
        # 부모가 미리 만든 depth subscription을 정리. ros2 cli에서 토픽이 사라진
        # 것처럼 보이지 않게 launch 단에서 RealSense의 depth 자체도 끈다(launch 참고).
        self.destroy_subscription(self.depth_subscription)
        self.depth_subscription = None

    def color_callback(self, msg):
        # bridge는 부모(ImgNode)에서 생성. bgr8로 디코드해 cv2에서 바로 사용 가능.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 튜플을 새로 할당 (rebind)하면 GIL 단위로 원자적이다 — timer가 중간 상태를
        # 보지 않는다. 부모가 쓰던 attr들도 호환을 위해 유지.
        self._color_with_header = (frame, msg.header)
        self.color_frame = frame
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def get_color_with_header(self):
        """timer가 호출. 아직 frame이 안 들어왔다면 ``None`` 반환."""
        return self._color_with_header


class YoloVisualizerNode(Node):
    """YOLO 추론 결과를 시각화 이미지로 발행하는 메인 노드.

    Parameters (launch arg 또는 ``--ros-args -p`` 로 override):

    * ``confidence_threshold`` (float, default 0.5) — bbox 표시 최소 신뢰도.
    * ``inference_rate_hz`` (float, default 30.0) — timer 주기. RealSense color
      입력 fps 이하로 두면 안정. 저성능 PC면 10~15Hz로 낮추면 됨.
    * ``jpeg_quality`` (int, default 80) — JPEG 품질 (1~100). 80은 시연 적정값.

    검증된 default 조합 (1280×720 입력, RTX 4060 Laptop):
    avg 30.000 Hz · jitter std 0.7~1.2 ms · max period 41 ms.
    """

    def __init__(self):
        super().__init__('yolo_visualizer')

        # 파라미터 선언 — launch에서 ParameterValue로 타입 강제 후 넘겨줌.
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate_hz', 30.0)
        self.declare_parameter('jpeg_quality', 80)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # YOLOv8 weight + 클래스 사전 로드. weight 경로/JSON은 yolo.py 상수 사용.
        # YoloModel(yolo.py)는 voting 기반이라 사용 X — 자세한 이유는 모듈 docstring.
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, 'r', encoding='utf-8') as f:
            class_dict = json.load(f)
        self.id_to_name = {int(k): v for k, v in class_dict.items()}

        # ImgNode를 별도 executor + thread로 spin시켜 메인 timer와 독립화.
        # 메인 노드(이 노드)는 main()의 rclpy.spin()이 돌리고, ImgNode는 여기서.
        # 결과: 추론 중에도 color callback은 멈추지 않고 최신 frame을 받아 둔다.
        self.img_node = ImgNodeWithHeader()
        self._img_executor = SingleThreadedExecutor()
        self._img_executor.add_node(self.img_node)
        self._img_thread = threading.Thread(
            target=self._img_executor.spin, daemon=True
        )
        self._img_thread.start()

        # 발행 타입은 CompressedImage(JPEG). 상세 설명은 모듈 docstring.
        # 시각화 전용이므로 raw Image 발행은 일부러 하지 않는다.
        # 토픽 이름은 image_transport 컨벤션(``.../image_raw/compressed``)을 따른다.
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera_gripper/yolo/image_raw/compressed',
            qos_profile_sensor_data,
        )
        # timer 주기. 추론+JPEG encode가 이 주기 안에 들어와야 hz가 유지됨.
        self.timer = self.create_timer(1.0 / rate_hz, self._timer_callback)

        self.get_logger().info(
            f"YoloVisualizerNode ready "
            f"(threshold={self.confidence_threshold}, rate={rate_hz} Hz, "
            f"jpeg_quality={self.jpeg_quality})"
        )

    def _timer_callback(self):
        """rate_hz 주기로 호출되는 메인 루프.

        흐름: 최신 frame → YOLO 추론 → bbox 그리기 → JPEG encode → publish.
        한 cycle 안에 모든 단계가 들어가야 timer 주기가 유지됨. 기본 33ms 안에
        720p 기준 추론 ~3-5ms + encode ~10-15ms + publish negligible 로 여유 있음.
        """
        snapshot = self.img_node.get_color_with_header()
        if snapshot is None:
            # RealSense가 아직 첫 frame을 안 보냈을 때 — 기다렸다 다음 cycle에 재시도.
            return
        frame, header = snapshot

        # ultralytics 직접 호출. verbose=False로 stdout에 매 frame 추론 로그를 띄우지
        # 않게 한다(시연 시 시각적 노이즈 방지).
        results = self.model(frame, verbose=False)
        # bbox는 사본에 그려야 다음 cycle 추론 입력이 오염되지 않음.
        annotated = frame.copy()

        if results:
            res = results[0]
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                # threshold 미만은 시각화 생략 — confidence_threshold 파라미터로 조정.
                if score < self.confidence_threshold:
                    continue
                self._draw_box(annotated, box, int(label), float(score))

        # JPEG encode. opencv가 numpy ndarray를 그대로 처리 — cv_bridge 불필요.
        ok, encoded = cv2.imencode(
            '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            # encode 실패는 매우 드물지만 안전하게 skip — 다음 cycle에 재시도.
            return
        # CompressedImage는 raw bytes를 그대로 담음. header는 frame과 같은 시점.
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)

    def _draw_box(self, img, box, class_id, score):
        """bbox + 라벨/score 텍스트를 ``img`` 에 in-place로 그린다."""
        x1, y1, x2, y2 = (int(v) for v in box)
        color = COLOR_PALETTE[class_id % len(COLOR_PALETTE)]
        name = self.id_to_name.get(class_id, str(class_id))
        # 외곽 사각형 (선 두께 2px).
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        # 라벨 텍스트는 bbox 위쪽에 배경색 칠한 후 흰 글씨로.
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
        """노드 종료 정리. ImgNode executor를 멈추고 thread join.

        Humble의 ``Executor.shutdown()`` 은 ``wait_for_threads`` 키워드 인자를 받지
        않는다(Iron 이후 추가). distro별 시그니처 차이로 한 번 회귀했던 부분이라
        키워드 인자 없이 그냥 호출한다.
        """
        self._img_executor.shutdown()
        self.img_node.destroy_node()
        self._img_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        # KeyboardInterrupt든 정상 종료든 executor + thread를 정리한 뒤 끝낸다.
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
