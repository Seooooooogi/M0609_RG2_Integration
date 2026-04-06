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

# 브링업 (rviz 시각화, 그리퍼만)
ros2 launch m0609_rg2_bringup bringup.launch.py

# 브링업 (rviz 시각화, RealSense 카메라 포함)
ros2 launch m0609_rg2_bringup bringup_camera.launch.py

# MoveIt2 (경로 계획)
ros2 launch m0609_rg2_moveit moveit.launch.py
```
