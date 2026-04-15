# M0609 + RG2 ROS2 Workspace

Doosan M0609 협동로봇 + OnRobot RG2 그리퍼 통합 ROS2 워크스페이스

---

## 요구사항

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- Intel RealSense SDK 2.0

```bash
sudo apt update

# 기본 도구 및 라이브러리
sudo apt install libpoco-dev

# ROS2 빌드 및 실행 관련
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description
sudo apt install ros-humble-gazebo-ros-pkgs

# 제어 및 하드웨어 인터페이스
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

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

## 빌드

```bash
cd ~/M0609_RG2_Integration
colcon build
source install/setup.bash
```

---

## 실행

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

# MoveIt2
ros2 launch m0609_rg2_moveit moveit.launch.py
```

### Real 모드 (실제 로봇)

> **사전 조건**
> - 로봇 IP: `192.168.1.100` (DRCF 연결)
> - 그리퍼 IP: `192.168.1.1` (OnRobot 컴퓨트박스, 고정)
> - UDP 포트 권한 설정 (최초 1회):
>   ```bash
>   sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0
>   # 재부팅 후에도 유지하려면:
>   echo 'net.ipv4.ip_unprivileged_port_start=0' | sudo tee /etc/sysctl.d/99-ros2-doosan.conf
>   ```

```bash
# 브링업 (그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

# 브링업 (RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100

# MoveIt2
ros2 launch m0609_rg2_moveit moveit.launch.py
```

---

## RealSense 카메라

### 주요 토픽

| 토픽 | 설명 |
|------|------|
| `/camera/color/image_raw` | RGB 컬러 이미지 |
| `/camera/aligned_depth_to_color/image_raw` | 컬러에 정렬된 뎁스 이미지 |
| `/camera/depth/color/points` | RGB 포인트클라우드 |
| `/camera/color/camera_info` | 컬러 카메라 내부 파라미터 |

### RViz 설정

`default.rviz`에 아래 display가 미리 구성되어 있음:

- **Color Image** — `/camera/color/image_raw`
- **Depth Image** — `/camera/aligned_depth_to_color/image_raw`
- **PointCloud2** — `/camera/depth/color/points`

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

> `world → base_link` 는 `static_transform_publisher` (identity transform)  
> `tool0 → rg2_base_link` 는 `joint0` (fixed)  
> `tool0 → bracket_link` 는 `tool0_to_bracket` (fixed)

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
    │       └── realsense_bracket.urdf.xacro         # 브라켓 + D435 마운트
    ├── m0609_rg2_moveit/           # 커스텀 MoveIt2 패키지
    │   ├── config/
    │   │   ├── joint_limits.yaml
    │   │   ├── kinematics.yaml
    │   │   ├── m0609_rg2.srdf
    │   │   ├── moveit_controllers.yaml
    │   │   └── ompl_planning.yaml
    │   └── launch/
    │       └── moveit.launch.py
    ├── doosan-robot2/              # 외부 패키지 — read-only
    └── onrobot-ros2/               # 외부 패키지 — read-only
```
