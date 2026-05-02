#!/usr/bin/env bash
# run_yolo_visualizer.sh — Docker zium-detection 안에서 yolo_visualizer 실행 (시연 임시)
#
# 토폴로지:
#   [호스트] RealSense + rqt_image_view + 화면 녹화
#   [Docker zium-detection] yolo_visualizer (ultralytics+torch+CUDA 런타임)
# 호스트와 컨테이너 모두 RMW=rmw_cyclonedds_cpp 일치 필요 (FastRTPS는 IPC namespace 격리로 sub 막힘).
#
# zium-detection 이미지는 외부 cobot2_block_construction 빌드 재활용.
# 신규 셋업자용 자체 Dockerfile은 docs/DEVELOPMENT_ROADMAP.md (정리 단계) 참조.
#
# 사용:
#   ./scripts/run_yolo_visualizer.sh           # 컨테이너 띄움 (background)
#   docker logs -f zium_visualizer             # 로그 모니터
#   docker rm -f zium_visualizer               # 종료

set -euo pipefail

WORKSPACE="${HOME}/M0609_RG2_Integration"
IMAGE_TAG="zium-detection:humble-cu118"
CONTAINER_NAME="zium_visualizer"

if ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  echo "[FATAL] 이미지 ${IMAGE_TAG} 없음." >&2
  echo "        외부 cobot2_block_construction 빌드 또는 setup_demo_host.sh 사전 실행 필요." >&2
  exit 1
fi

if [ ! -f "${WORKSPACE}/install/object_detection/lib/object_detection/yolo_visualizer" ]; then
  echo "[FATAL] yolo_visualizer 미빌드." >&2
  echo "        cd ${WORKSPACE} && colcon build --packages-select object_detection od_msg" >&2
  exit 1
fi

# 잔재 컨테이너 정리 (멱등)
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

docker run -d \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc=host \
  --gpus all \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "${WORKSPACE}":"${WORKSPACE}" \
  -w "${WORKSPACE}" \
  --entrypoint bash \
  "${IMAGE_TAG}" \
  -c "source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run object_detection yolo_visualizer" \
  >/dev/null

echo "[OK] ${CONTAINER_NAME} started (RMW=cyclonedds, DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
echo "  로그: docker logs -f ${CONTAINER_NAME}"
echo "  종료: docker rm -f ${CONTAINER_NAME}"
echo "  rqt:  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\"
echo "        ros2 run rqt_image_view rqt_image_view \\"
echo "             /camera_gripper/camera_gripper/yolo/image_raw/compressed"
