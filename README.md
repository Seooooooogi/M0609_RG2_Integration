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

### RealSense 주요 토픽 (dual camera)

| 토픽 | 설명 |
|------|------|
| `/camera_gripper/camera_gripper/color/image_raw` | 손목 카메라 RGB |
| `/camera_gripper/camera_gripper/aligned_depth_to_color/image_raw` | 손목 카메라 뎁스 (컬러 정렬) |
| `/camera_gripper/camera_gripper/color/camera_info` | 손목 카메라 내부 파라미터 |
| `/camera_global/camera_global/color/image_raw` | 고정 카메라 RGB |

`default.rviz` 사전 구성 display:
- **Color Image** — `/camera_gripper/camera_gripper/color/image_raw`
- **Depth Image** — `/camera_gripper/camera_gripper/aligned_depth_to_color/image_raw`

---

## VLA 데이터 수집

Python 패키지 설치 (최초 1회):

```bash
pip install pandas pyarrow imageio imageio-ffmpeg "numpy<2"
```

> `numpy<2` 제약: ROS2 Humble의 cv_bridge / OpenCV가 NumPy 1.x로 컴파일됨.

### 1단계 — rosbag 녹화

두 대의 RealSense + 관절 상태 동시 녹화.

```bash
# 브링업 (real 모드, 별도 터미널)
ros2 launch m0609_rg2_bringup bringup_dual_camera.launch.py \
    mode:=real host:=192.168.1.100

# 녹화 시작
ros2 bag record \
    /camera_gripper/camera_gripper/color/image_raw \
    /camera_global/camera_global/color/image_raw \
    /dsr01/joint_states \
    /integrated_joint_states \
    /gripper_joint_states \
    -o ~/rosbag_recordings/<episode_name>

# 녹화 중단: Ctrl+C
```

| 카메라 네임스페이스 | 위치 |
|---|---|
| `camera_gripper` | 손목 마운트 (wrist) |
| `camera_global` | 고정 시점 (global) |

### 2단계 — LeRobot v2.1 포맷 변환

openpi가 pin한 lerobot 커밋(`0cf86487`)의 `CODEBASE_VERSION = "v2.1"` 포맷 기준.

```bash
source /opt/ros/humble/setup.bash

python3 tools/rosbag_to_lerobot.py \
    --bag  ~/rosbag_recordings/<episode_name> \
    --output ~/lerobot_datasets/<dataset_name> \
    --task "pick up the object" \
    --episode-index 0
```

에피소드 추가 시: `--episode-index`를 1씩 증가, 동일 `--output` 경로 재사용. 글로벌 `index` 컬럼은 기존 parquet 파일 스캔 후 자동 누적.

```bash
python3 tools/rosbag_to_lerobot.py \
    --bag  ~/rosbag_recordings/<episode_name_2> \
    --output ~/lerobot_datasets/<dataset_name> \
    --task "pick up the object" \
    --episode-index 1
```

**주요 옵션**

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--bag` | — | rosbag 디렉토리 경로 |
| `--output` | — | 데이터셋 출력 경로 |
| `--task` | `"pick up the object"` | 태스크 설명 문자열 |
| `--episode-index` | `0` | 에피소드 인덱스 |
| `--slop` | `0.02` | 카메라↔관절 동기화 허용 오차 (초) |
| `--img-size` | `224` | 출력 이미지 크기 (정사각형) |
| `--fps` | `30` | 출력 비디오 FPS |

### 출력 구조 (LeRobot v2.1)

```
<dataset_name>/
├── data/
│   └── chunk-000/
│       └── episode_000000.parquet   # frame_index, index, timestamp, episode_index, task_index,
│                                    # observation.state, action
├── videos/
│   └── chunk-000/
│       ├── observation.images.camera_gripper_episode_000000.mp4   # 손목 카메라 224×224 H.264
│       └── observation.images.camera_global_episode_000000.mp4    # 고정 카메라 224×224 H.264
└── meta/
    ├── info.json      # 데이터셋 스펙 (codebase_version, fps, features, robot_type 등)
    ├── stats.json     # observation.state / action 통계
    ├── tasks.jsonl    # 태스크 목록
    └── episodes.jsonl # 에피소드 목록 (길이 포함)
```

`observation.state` / `action` 구성 (7D): `[joint_1, joint_2, joint_4, joint_5, joint_3, joint_6, gripper_open]` — 관절 6개는 `/dsr01/joint_states` 발행 순서, `gripper_open`은 binary (1.0=열림, 0.0=닫힘).

### 에피소드 검증

```bash
# 관절값 통계 출력
python3 tools/inspect_episode.py ~/lerobot_datasets/<dataset_name>/data/chunk-000/episode_000000.parquet

# 관절 궤적 PNG 저장
python3 tools/inspect_episode.py ~/lerobot_datasets/<dataset_name>/data/chunk-000/episode_000000.parquet --plot
```

---

## TF 구조

### bringup.launch.py (그리퍼만)

```
world
└── base_link
    └── link_1 → link_2 → link_3 → link_4 → link_5 → link_6
                                                          └── tool0
                                                              └── rg2_base_link
                                                                  ├── rg2_left_outer_knuckle
                                                                  │   └── rg2_left_inner_finger
                                                                  ├── rg2_right_outer_knuckle
                                                                  │   └── rg2_right_inner_finger
                                                                  ├── rg2_left_inner_knuckle   (mimic)
                                                                  └── rg2_right_inner_knuckle  (mimic)
```

### bringup_camera.launch.py / bringup_dual_camera.launch.py (카메라 포함)

```
world
└── base_link
    └── link_1 → ... → tool0
                          ├── rg2_base_link                    (그리퍼, 위와 동일)
                          └── bracket_link                     (마운트 브라켓)
                              └── camera_bottom_screw_frame
                                  └── camera_link
                                      ├── camera_color_frame
                                      │   └── camera_color_optical_frame
                                      ├── camera_depth_frame
                                      │   └── camera_depth_optical_frame
                                      ├── camera_infra1_frame
                                      │   └── camera_infra1_optical_frame
                                      └── camera_infra2_frame
                                          └── camera_infra2_optical_frame
```

- `world → base_link`: `static_transform_publisher` (identity)
- `tool0 → rg2_base_link`: `joint0` (fixed)
- `tool0 → bracket_link`: `tool0_to_bracket` (fixed)
- `rg2_left/right_inner_knuckle`: mimic joint, `rg2_finger_joint` 기준 연동
- `bringup_dual_camera`의 `camera_global`: URDF 미등록 외부 고정 카메라

---

## 디렉토리 구조

```
M0609_RG2_Integration/
├── tools/
│   ├── rosbag_to_lerobot.py            # rosbag → LeRobot v2.1 변환 스크립트
│   └── inspect_episode.py              # parquet 에피소드 검증 (관절값 통계 + --plot)
└── src/
    ├── m0609_rg2_bringup/              # 커스텀 브링업 패키지
    │   ├── launch/
    │   │   ├── bringup.launch.py               # 로봇 + 그리퍼
    │   │   ├── bringup_camera.launch.py        # 로봇 + 그리퍼 + RealSense 1대
    │   │   └── bringup_dual_camera.launch.py   # 로봇 + 그리퍼 + RealSense 2대 (VLA 데이터 수집)
    │   ├── meshes/
    │   │   └── mount_bracket.stl
    │   ├── rviz/
    │   │   └── default.rviz
    │   ├── scripts/
    │   │   ├── gripper_joint_state_publisher.py   # onrobot_joint_states → gripper_joint_states (real)
    │   │   └── gripper_virtual_node.py            # /onrobot/sendCommand 서비스 + RViz 애니메이션 (virtual)
    │   └── urdf/
    │       ├── m0609_with_rg2.urdf.xacro           # 팔 + 그리퍼
    │       ├── m0609_with_rg2_camera.urdf.xacro    # 팔 + 그리퍼 + 손목 카메라
    │       ├── onrobot_rg2.xacro                   # RG2 베이스 링크 (has_bracket 파라미터 포함)
    │       ├── onrobot_rg2_model_macro.xacro        # RG2 링크/조인트 매크로
    │       └── realsense_bracket.urdf.xacro         # 브라켓 + D435 마운트 (tool0 기준)
    ├── m0609_rg2_moveit/               # MoveIt2 패키지 (deprecated — 현재 미사용)
    ├── doosan-robot2/                  # 외부 패키지 — read-only
    └── onrobot-ros2/                   # 외부 패키지 — read-only
```
