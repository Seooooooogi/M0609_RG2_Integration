# M0609_RG2_Integration — Development Roadmap

전체 목표: Doosan M0609 + RG2 + RealSense D435 통합 환경에서
VLA(pi-zero/openpi) 모델 학습용 LeRobot 포맷 데이터 수집 파이프라인 구축.

기간: 2026-01-02 ~ 2026-06-01

---

## Phase 1: 브링업 및 시각화 ✅ 완료

- [x] 1-1. Doosan M0609 URDF + ROS2 환경 구성
- [x] 1-2. OnRobot RG2 xacro 버그 수정 및 통합 (`tool0` 연결)
- [x] 1-3. 통합 launch 파일 작성 (`bringup.launch.py`)
- [x] 1-4. RViz2 시각화 확인 (가상 모드)

## Phase 1.5: MoveIt2 연동 ✅ 완료

- [x] 1.5-1. `m0609_rg2_moveit` 패키지 생성
- [x] 1.5-2. manipulator 그룹 구성 (`base_link` → `tool0`, 6DOF)
- [x] 1.5-3. gripper 그룹 구성 (`rg2_finger_joint`, 1DOF)
- [x] 1.5-4. 사전 정의 자세 설정 (`all-zeros`, `gripper_open`, `gripper_close`)
- [x] 1.5-5. KDL IK + RRTConnect 플래너 구성

## Phase 2: RealSense D435 통합 ✅ 완료

- [x] 2-1. 3D 프린팅 브라켓 설계 및 xacro 작성 (`bracket_link`)
- [x] 2-2. RealSense D435 TF 연결 (`tool0 → bracket_link → camera_link`)
- [x] 2-3. `bringup_camera.launch.py` 작성
- [x] 2-4. 실제 로봇 하드웨어 연동 확인 (TCP/IP + Modbus TCP)
- [x] 2-5. virtual/real 모드 분기 구현

## Phase 3: VLA 데이터 수집 파이프라인 ⏳ 진행 중

### 3-0. 인프라 검증 (선행)
- [ ] 3-0-1. RealSense 2대 동시 rosbag 기록 가능 여부 확인
  - 토픽: `/camera_*/color/image_raw`, `/dsr01/joint_states`
  - 확인 항목: 프레임 드롭, 타임스탬프 동기화, USB 대역폭
- [ ] 3-0-2. rosbag → 프레임 단위 추출 스크립트 작성 및 검증

### 3-1. 시나리오 구현 (단계적)
- [ ] 3-1-1. 시나리오 A: 물체 집기 (가장 단순, 우선 진행)
  - 단일 물체 pick & place
  - RealSense 2대 (wrist + 고정 시점)
- [ ] 3-1-2. 시나리오 B: 철기둥 옮기기
- [ ] 3-1-3. 시나리오 C: 기어 끼워넣기 (compliance 데이터 포함)

### 3-2. 데이터 포맷 파이프라인
- [ ] 3-2-1. LeRobot 포맷 분석 및 Doosan 커스텀 클래스 설계
  - `/joint_states` + `/camera` 토픽 → LeRobot 에피소드 포맷 변환
- [ ] 3-2-2. rosbag → LeRobot 변환 스크립트
- [ ] 3-2-3. 수집된 데이터셋 검증 및 포맷 확인

## Phase 4: VLA 모델 학습 및 평가 (예정)

- [ ] 4-1. pi-zero(openpi) 모델 학습 환경 구성
- [ ] 4-2. 수집 데이터셋으로 파인튜닝
- [ ] 4-3. 실제 로봇에서 추론 파이프라인 구성
- [ ] 4-4. TTS 연동 (전체 파이프라인 발표 자료 생성)

## Backlog (미확정)

- [ ] 데이터 수집 GUI 도구 (RViz 플러그인 또는 웹 UI)
- [ ] 멀티 에피소드 자동 수집 스크립트
- [ ] 데이터셋 시각화 도구
