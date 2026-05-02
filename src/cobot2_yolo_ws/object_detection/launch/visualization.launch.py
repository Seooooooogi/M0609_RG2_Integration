"""YOLO bbox visualization launch (시연용).

이 launch는 두 노드를 한 번에 띄운다:
  1) RealSense color stream (호스트 USB)
  2) yolo_visualizer (YOLO 추론 + bbox 그린 CompressedImage 발행)

실제 운용은 **호스트 RealSense ↔ 도커 visualizer** 분리 패턴이다 (visualizer는
ultralytics/torch가 필요해 호스트에 미설치 — ``zium-detection:humble-cu118``
도커 이미지 안에서 동작). 호스트에서 이 launch를 통째로 실행하면 visualizer
노드는 ImportError로 종료되고 RealSense만 살아남는다(의도된 동작). 도커 쪽 실행
명령은 ``docs/yolo_detection_pipeline.md`` § 7 참조.

RMW: 양쪽 모두 ``rmw_cyclonedds_cpp`` 필수. FastRTPS는 도커 IPC namespace 격리로
호스트↔도커 데이터 sub가 막힘 (``--ipc=host`` 줘도 불안정).

검증된 default 조합: 1280×720×30 입력 / 30 Hz timer / JPEG q80
→ avg 30.000 Hz · jitter std 1 ms (성능 보고서 참조).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ─── 시연자가 토글 가능한 launch arg ─────────────────────────────────
    # 사용 예: ros2 launch object_detection visualization.launch.py \
    #         confidence_threshold:=0.4 inference_rate_hz:=15.0
    # ────────────────────────────────────────────────────────────────────
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold (0.0 - 1.0). 낮추면 더 많이 검출됨.',
    )
    rate_arg = DeclareLaunchArgument(
        'inference_rate_hz',
        default_value='30.0',
        description='YOLO inference timer rate (Hz). 저성능 PC면 10~15로 낮출 것.',
    )
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG quality for compressed publish (1 - 100). 80 권장.',
    )
    color_profile_arg = DeclareLaunchArgument(
        'color_profile',
        default_value='1280x720x30',
        description='RealSense color stream profile (WIDTHxHEIGHTxFPS). 480p로 가려면 640x480x30.',
    )

    # ─── RealSense 노드 (호스트에서만 실행됨) ────────────────────────────
    # depth/IMU 모두 OFF — visualizer는 color만 사용. 토픽 트래픽/CPU 절약 효과.
    # name='camera_gripper' 단독 → 토픽 prefix /camera_gripper/...
    # ────────────────────────────────────────────────────────────────────
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_gripper',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,         # visualizer 미사용 — 끄면 USB 대역폭 ↓
            'align_depth.enable': False,   # depth가 꺼져 있으므로 align도 의미 없음
            'enable_gyro': False,          # D435i IMU. 시연 시 불필요 + USB 안정성 ↑
            'enable_accel': False,
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        }],
        output='screen',
    )

    # ─── Visualizer 노드 (도커 안에서만 정상 동작) ──────────────────────
    # 호스트에는 ultralytics가 없어 ImportError로 즉사. RealSense만 살아남음.
    # 도커에서 단독 실행하려면:
    #   ros2 run object_detection yolo_visualizer
    # (이 경우 launch arg는 -p 로 직접 override)
    # ────────────────────────────────────────────────────────────────────
    visualizer_node = Node(
        package='object_detection',
        executable='yolo_visualizer',
        name='yolo_visualizer',
        parameters=[{
            # ParameterValue로 타입 강제 — DeclareLaunchArgument는 문자열로 들어옴.
            'confidence_threshold': ParameterValue(
                LaunchConfiguration('confidence_threshold'),
                value_type=float,
            ),
            'inference_rate_hz': ParameterValue(
                LaunchConfiguration('inference_rate_hz'),
                value_type=float,
            ),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'),
                value_type=int,
            ),
        }],
        output='screen',
    )

    return LaunchDescription([
        confidence_arg,
        rate_arg,
        jpeg_quality_arg,
        color_profile_arg,
        realsense_node,
        visualizer_node,
    ])
