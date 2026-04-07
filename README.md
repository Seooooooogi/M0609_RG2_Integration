# M0609 + RG2 ROS2 Workspace

Doosan M0609 협동로봇 + OnRobot RG2 그리퍼 통합 ROS2 워크스페이스

---

## 요구사항

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- Intel RealSense SDK 2.0

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description

# OnRobot 그리퍼 드라이버 의존성
pip3 install pymodbus==3.3.2
```

---

## 의존성 패키지 설치

```bash
mkdir -p ~/doosan_gripper_ws/src
cd ~/doosan_gripper_ws/src

# Doosan 공식 패키지
git clone https://github.com/doosan-robotics/doosan-robot2

# OnRobot 공식 패키지
git clone https://github.com/OnRobotApS/onrobot-ros2
```

---

## 빌드

```bash
cd ~/doosan_gripper_ws
colcon build
source install/setup.bash
```

---

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/doosan_gripper_ws/install/setup.bash
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
