# M0609 + RG2 ROS2 Workspace

Doosan M0609 협동로봇 + OnRobot RG2 그리퍼 통합 ROS2 워크스페이스

---

## 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Intel RealSense SDK 2.0

```bash
sudo apt update

# 기본 도구 및 라이브러리
sudo apt install libpoco-dev

# ROS2 빌드 및 실행 관련
sudo apt install ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-gazebo-ros-pkgs

# 제어 및 하드웨어 인터페이스
sudo apt install ros-humble-hardware-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# OnRobot 그리퍼 드라이버 의존성
pip3 install pymodbus==3.3.2
```

---

## 의존성 패키지 설치

```bash
mkdir -p ~/M0609_RG2_Integration/src
cd ~/M0609_RG2_Integration/src

# Doosan 공식 패키지
git clone https://github.com/doosan-robotics/doosan-robot2

# OnRobot RG2 패키지
git clone https://github.com/ABC-iRobotics/onrobot-ros2
```

---

## 초기 설정 (최초 1회)

### Real 모드 사전 조건

- 로봇 IP: `192.168.1.100`
- 그리퍼 IP: `192.168.1.1` (OnRobot 컴퓨트박스, 고정)
- UDP 포트 권한 설정:
  ```bash
  sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0
  # 재부팅 후에도 유지:
  echo 'net.ipv4.ip_unprivileged_port_start=0' | sudo tee /etc/sysctl.d/99-ros2-doosan.conf
  ```

### RealSense udev rules

udev rules 미설치 시 스트리밍 중 `xioctl(VIDIOC_QBUF) failed — No such device` 에러 발생.

```bash
sudo curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

적용 후 USB 재연결 필요.

---

## 빌드

```bash
cd ~/M0609_RG2_Integration
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

환경 설정:

```bash
source /opt/ros/humble/setup.bash
source ~/M0609_RG2_Integration/install/setup.bash
```

### Virtual 모드 (시뮬레이션)

```bash
# 브링업 (그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py

# 브링업 (RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py

```

### Real 모드 (실제 로봇)

```bash
# 브링업 (그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

# 브링업 (RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100

```

### Virtual / Real 모드 그리퍼 동작 차이

virtual 모드에서 `gripper_virtual_node`(bringup에 포함)가 `/onrobot/sendCommand` 서비스로 RViz 시각화 담당. OnRobot RG2 Modbus 제어 미포함.

| 항목 | real 모드 | virtual 모드 |
|------|-----------|-------------|
| 그리퍼 제어 | OnRobot 드라이버 (Modbus TCP) | Modbus 미포함 |
| 완료 신호 | 디지털 입력 핀 감지 | `/onrobot/sendCommand` 응답 (애니메이션 완료 시) |
| RViz 그리퍼 상태 | `/gripper_joint_states` (OnRobot 드라이버) | `/gripper_joint_states` (gripper_virtual_node) |
| 파지력 / 접촉 | 실제 물리 동작 | 시뮬레이션 없음 |
| Tool/TCP 프리셋 | DRCF 등록값 사용 | 설정 스킵 (에뮬레이터 미등록) |

### RealSense 주요 토픽

bringup_camera.launch.py는 RealSense 노드를 `name='camera_gripper'`로 띄운다 (cobot2_yolo_ws 호환, 단일 prefix).

| 토픽 | 설명 |
|------|------|
| `/camera_gripper/color/image_raw` | RGB 컬러 이미지 |
| `/camera_gripper/aligned_depth_to_color/image_raw` | 컬러 정렬 뎁스 이미지 |
| `/camera_gripper/depth/color/points` | RGB 포인트클라우드 |
| `/camera_gripper/color/camera_info` | 컬러 카메라 내부 파라미터 |

`default.rviz` 사전 구성 display:
- **Color Image** — `/camera_gripper/yolo/image_raw/compressed` (시연 시 yolo bbox 표시. 카메라 raw로 보고 싶으면 토픽을 `/camera_gripper/color/image_raw`로 변경)
- **Depth Image** — `/camera_gripper/aligned_depth_to_color/image_raw`
- **PointCloud2** — `/camera_gripper/depth/color/points`

---

## YOLO Pick & Place 시연 (visualizer + detection + robot_control)

`feat/yolo-detection-visualization` 브랜치. RealSense color → YOLOv8 추론 → bbox 시각화(CompressedImage) + `/get_3d_position` 서비스 → 로봇 pick & place 동작.

검증된 시각화 성능 (RTX 4070 Laptop, 1280×720 기준): **avg 30Hz · jitter std 1 ms**. 시연 시 부하 줄이려면 `640x480x30`. 상세는 [`docs/yolo_visualizer_performance_report.md`](docs/yolo_visualizer_performance_report.md), 노드 그래프는 [`docs/yolo_detection_pipeline.md`](docs/yolo_detection_pipeline.md).

### 사전 조건

- NVIDIA GPU + driver
- `zium-detection:humble-cu118` 도커 이미지 — ultralytics+torch+CUDA 런타임. **이 이미지는 본 레포에 없음** — 외부 `cobot2_block_construction` 빌드 재활용. 신규 셋업자용 옵션은 [`docs/DEVELOPMENT_ROADMAP.md`](docs/DEVELOPMENT_ROADMAP.md) 참조.
- 호스트와 도커 양쪽 모두 `rmw_cyclonedds_cpp` (FastRTPS는 IPC namespace 격리로 sub 불안정)

### 호스트 일회성 셋업 (자동화)

```bash
~/cobot2_block_construction/scripts/setup_demo_host.sh
sudo apt install -y ros-humble-image-transport-plugins   # rqt에서 compressed 토픽 디코드
python3 -m pip install --user --upgrade "scipy>=1.13"    # numpy 2.x 호환
```

`setup_demo_host.sh`는 멱등이며 다음을 처리:
- `nvidia-container-toolkit` 설치 + Docker nvidia runtime 등록
- 호스트 `rmw_cyclonedds_cpp` 설치 + `~/.bashrc`에 `cyclone`/`fastdds` alias 추가
- `zium-detection` 이미지에 cyclone DDS 패치 (commit overwrite, 원본 ENTRYPOINT/CMD 보존)

### 빌드

```bash
cd ~/M0609_RG2_Integration
colcon build --symlink-install --packages-select object_detection od_msg robot_control
source install/setup.bash
```

> 원본 `src/cobot2_ws/`는 동일한 패키지명 충돌 회피용 `COLCON_IGNORE`. 시연 대상은 `src/cobot2_yolo_ws/` 카피본.

### 실행 — 4개 터미널

각 터미널 첫 줄에 `cyclone` 토글 필수.

**터미널 1 — 호스트 bringup (RealSense + 로봇팔 + RViz)**
```bash
cyclone
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100
```

RealSense는 `name='camera_gripper'`로 떠서 `/camera_gripper/...` 단일 prefix로 발행. RViz의 Color Image display는 default가 `/camera_gripper/yolo/image_raw/compressed` — yolo 노드가 뜨면 자동으로 bbox 표시.

**터미널 2 — Docker yolo_visualizer**
```bash
~/M0609_RG2_Integration/scripts/run_yolo_visualizer.sh
docker logs -f zium_visualizer    # 모델 로드 확인 후 Ctrl+C로 빠짐
```

`scripts/run_yolo_visualizer.sh`는 `zium-detection` 컨테이너에 워크스페이스를 mount하고 cyclone RMW로 `yolo_visualizer`를 띄움.

**터미널 3 — Docker detection (`/get_3d_position` 서비스)**
```bash
docker exec zium_visualizer bash -c 'source /opt/ros/humble/setup.bash && source /home/rokey/M0609_RG2_Integration/install/setup.bash && cd /home/rokey/M0609_RG2_Integration && ros2 run object_detection object_detection'
```

attached 실행 — stderr 보임. 이 노드가 없으면 robot_control이 service 대기로 stall.

**터미널 4 — 호스트 robot_control (pick & place)**
```bash
cyclone
ros2 run robot_control robot_control
```

프롬프트: `Pick target(s) by number (space-separated, e.g. '1 3')` — 클래스 번호(`0:drill, 1:hammer, 2:pliers, 3:screwdriver, 4:wrench`)로 입력.

### 검증

별도 터미널에서:
```bash
cyclone
ros2 service list | grep get_3d                                    # /get_3d_position 보여야 함
ros2 topic hz /camera_gripper/color/image_raw                       # ~30Hz
ros2 topic hz /camera_gripper/yolo/image_raw/compressed             # ~30Hz with payload
ros2 topic info /camera_gripper/aligned_depth_to_color/image_raw -v # Publisher RELIABLE 확인
```

### 종료

```bash
docker rm -f zium_visualizer       # 컨테이너 정리 (detection 노드도 같이 종료)
# 각 호스트 터미널 Ctrl+C
docker image prune -f              # (선택) commit 누적된 dangling 이미지 정리
```

### 토글 가능한 launch arg (visualization.launch.py 사용 시)

| arg | default | 설명 |
|------|---------|------|
| `confidence_threshold` | `0.5` | bbox 표시 최소 신뢰도 |
| `inference_rate_hz` | `30.0` | timer 주기. 저성능 PC면 10~15로 낮출 것 |
| `jpeg_quality` | `80` | JPEG 압축 품질 (1~100) |
| `color_profile` | `1280x720x30` | RealSense color 프로파일. 480p 원하면 `640x480x30` |

> **주의**: 위 표는 `visualization.launch.py` 단독 모드(시각화만, pick & place 없음) 한정. 본 시연(옵션 B)은 `bringup_camera.launch.py`를 쓰므로 `bringup_camera.launch.py`의 `realsense_node` parameters dict를 직접 수정하거나 launch arg로 추출.

### 발행 토픽

- `/camera_gripper/yolo/image_raw/compressed` — `sensor_msgs/CompressedImage`, JPEG q80. raw는 발행 X (대역폭/jitter 절약).

---

## TF 구조

### bringup.launch.py (그리퍼만)

```
world
└── base_link
    └── link1 → link2 → link3 → link4 → link5 → link6
                                                    └── tool0
                                                        └── rg2_base_link
                                                            ├── rg2_left_outer_knuckle
                                                            │   ├── rg2_left_inner_knuckle
                                                            │   └── rg2_left_inner_finger
                                                            └── rg2_right_outer_knuckle
                                                                ├── rg2_right_inner_knuckle
                                                                └── rg2_right_inner_finger
```

### bringup_camera.launch.py (카메라 포함)

```
world
└── base_link
    └── link1 → ... → tool0
                       ├── rg2_base_link          (그리퍼, 위와 동일)
                       └── bracket_link           (마운트 브라켓)
                           └── camera_link
                               ├── camera_color_frame / camera_color_optical_frame
                               ├── camera_depth_frame / camera_depth_optical_frame
                               ├── camera_infra1_frame / camera_infra1_optical_frame
                               └── camera_infra2_frame / camera_infra2_optical_frame
```

- `world → base_link`: `static_transform_publisher` (identity)
- `tool0 → rg2_base_link`: `joint0` (fixed)
- `tool0 → bracket_link`: `tool0_to_bracket` (fixed)
- `rg2_left/right_inner_knuckle`: mimic joint, `rg2_finger_joint` 기준 연동

---

## 디렉토리 구조

```
M0609_RG2_Integration/
└── src/
    ├── m0609_rg2_bringup/          # 커스텀 브링업 패키지
    │   ├── launch/
    │   │   ├── bringup.launch.py           # 로봇 + 그리퍼
    │   │   └── bringup_camera.launch.py    # 로봇 + 그리퍼 + RealSense
    │   ├── meshes/
    │   │   └── mount_bracket.stl
    │   ├── rviz/
    │   │   ├── default.rviz
    │   │   └── moveit.rviz
    │   ├── scripts/
    │   │   └── gripper_joint_state_publisher.py   # onrobot_joint_states → gripper_joint_states
    │   └── urdf/
    │       ├── m0609_with_rg2.urdf.xacro           # 팔 + 그리퍼 통합 URDF
    │       ├── m0609_with_rg2_camera.urdf.xacro    # 팔 + 그리퍼 + 카메라 통합 URDF
    │       ├── onrobot_rg2.xacro                   # RG2 베이스 링크 정의
    │       ├── onrobot_rg2_model_macro.xacro        # RG2 링크/조인트 매크로
    │       └── realsense_bracket.urdf.xacro         # 브라켓 + D435 마운트 (tool0 기준)
    ├── m0609_rg2_moveit/               # MoveIt2 패키지 (deprecated — 현재 미사용)
    ├── cobot2_yolo_ws/                 # YOLO 시각화 워크스페이스 (시연 브랜치)
    │   ├── object_detection/
    │   │   ├── object_detection/
    │   │   │   ├── visualizer.py       # 시연 대상 — bbox 시각화 노드
    │   │   │   ├── realsense.py        # ImgNode (color/depth/camera_info 구독)
    │   │   │   ├── yolo.py             # YoloModel (서비스용 voting, visualizer 미사용)
    │   │   │   └── detection.py        # /get_3d_position 서비스 서버 (visualizer와 독립)
    │   │   ├── launch/visualization.launch.py
    │   │   └── resource/
    │   │       ├── yolov8n_tools_0122.pt
    │   │       └── class_name_tool.json
    │   ├── od_msg/                     # SrvDepthPosition.srv 정의
    │   └── robot_control/              # /get_3d_position 클라이언트 (시연 무관)
    ├── cobot2_ws/                      # 원본 — COLCON_IGNORE로 빌드 제외 (보존)
    ├── doosan-robot2/                  # 외부 패키지 — read-only
    └── onrobot-ros2/                   # 외부 패키지 — read-only
```
