#!/bin/bash
# =============================================================================
# YDLidar X4 Pro Docker 실행 스크립트
# 사용법:
#   ./run_ydlidar_docker.sh                         # 기본값으로 실행
#   ./run_ydlidar_docker.sh -p /dev/ttyUSB1         # 포트 지정
#   ./run_ydlidar_docker.sh -p /dev/ttyUSB0 -f lidar_link  # 포트 + 프레임 ID
#   ./run_ydlidar_docker.sh --build                 # 이미지 빌드 후 실행
#   ./run_ydlidar_docker.sh --shell                 # bash 진입
# =============================================================================

set -e

# ---- 기본값 ----
IMAGE_NAME="ydlidar_driver:humble"
LIDAR_PORT="/dev/ttyUSB0"
LIDAR_FRAME_ID="laser_frame"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DO_BUILD=false
DO_SHELL=false

# ---- 인자 파싱 ----
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -p, --port     <port>      시리얼 포트 경로 (기본값: /dev/ttyUSB0)"
    echo "  -f, --frame-id <frame_id>  TF 프레임 ID    (기본값: laser_frame)"
    echo "  -d, --domain   <id>        ROS_DOMAIN_ID  (기본값: 0)"
    echo "  -i, --image    <image>     Docker 이미지  (기본값: ydlidar_driver:humble)"
    echo "      --build                이미지를 빌드하고 실행"
    echo "      --shell                컨테이너 bash 셸 진입 (드라이버 실행 안 함)"
    echo "  -h, --help                 이 도움말 표시"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--port)      LIDAR_PORT="$2";     shift 2 ;;
        -f|--frame-id)  LIDAR_FRAME_ID="$2"; shift 2 ;;
        -d|--domain)    ROS_DOMAIN_ID="$2";  shift 2 ;;
        -i|--image)     IMAGE_NAME="$2";     shift 2 ;;
        --build)        DO_BUILD=true;       shift   ;;
        --shell)        DO_SHELL=true;       shift   ;;
        -h|--help)      usage ;;
        *) echo "[ERROR] 알 수 없는 옵션: $1"; usage ;;
    esac
done

# ---- 스크립트 위치 기준으로 Dockerfile 경로 결정 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 이미지 빌드 ----
if $DO_BUILD; then
    echo "[INFO] Docker 이미지 빌드 중: ${IMAGE_NAME}"
    docker build \
        --target runtime \
        -t "${IMAGE_NAME}" \
        "${SCRIPT_DIR}"
    echo "[INFO] 빌드 완료."
fi

# ---- 이미지 존재 확인 ----
if ! docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
    echo "[ERROR] 이미지 '${IMAGE_NAME}'를 찾을 수 없습니다."
    echo "        먼저 빌드하려면: $0 --build"
    exit 1
fi

# ---- 시리얼 포트 존재 확인 ----
if [[ ! -e "${LIDAR_PORT}" ]]; then
    echo "[WARN] 시리얼 포트 '${LIDAR_PORT}'가 존재하지 않습니다."
    echo "       LiDAR가 연결되어 있는지 확인하세요."
    echo "       연결된 포트 목록:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "       (없음)"
    read -rp "       그래도 계속 실행하시겠습니까? [y/N] " yn
    [[ "${yn}" =~ ^[Yy]$ ]] || exit 1
fi

# ---- docker run 인자 구성 ----
DOCKER_ARGS=(
    --rm
    --network host
    --device "${LIDAR_PORT}:${LIDAR_PORT}"
    -e "LIDAR_PORT=${LIDAR_PORT}"
    -e "LIDAR_FRAME_ID=${LIDAR_FRAME_ID}"
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    --name ydlidar_x4pro
)

echo "========================================"
echo "  YDLidar X4 Pro Docker 실행"
echo "========================================"
echo "  이미지        : ${IMAGE_NAME}"
echo "  시리얼 포트   : ${LIDAR_PORT}"
echo "  TF 프레임 ID  : ${LIDAR_FRAME_ID}"
echo "  ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo "========================================"

if $DO_SHELL; then
    echo "  모드: bash 셸"
    echo "========================================"
    docker run -it "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" bash
else
    echo "  모드: 라이다 드라이버 실행"
    echo "  종료: Ctrl+C"
    echo "========================================"
    docker run -it "${DOCKER_ARGS[@]}" "${IMAGE_NAME}"
fi
