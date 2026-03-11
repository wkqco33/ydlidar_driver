# =============================================================================
# YDLidar X4 Pro ROS2 Humble Driver — Raspberry Pi 4 (ARM64) / AMD64
# =============================================================================
# 멀티스테이지 빌드:
#   builder  — 소스 컴파일
#   runtime  — 실행에 필요한 것만 포함 (이미지 크기 최소화)
#
# 멀티플랫폼 빌드 예시:
#   docker buildx build --platform linux/arm64,linux/amd64 \
#     -t wkqco33/ydlidar:latest . --push
# =============================================================================

# ---- Stage 1: Builder -------------------------------------------------------
# ros:humble-ros-base 는 공식 멀티아치 매니페스트 (arm64/amd64 모두 지원)
FROM ros:humble-ros-base AS builder

# 빌드 도구 및 ROS 의존성 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    ros-humble-launch-ros \
    ros-humble-launch \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /ros2_ws/src/ydlidar_driver

# 소스 복사 (build/ 등 제외 — .dockerignore 참고)
COPY CMakeLists.txt package.xml ./
COPY src/  ./src/
COPY params/ ./params/
COPY launch/ ./launch/

WORKDIR /ros2_ws

# 빌드
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --packages-select ydlidar_driver \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --no-warn-unused-cli


# ---- Stage 2: Runtime -------------------------------------------------------
FROM ros:humble-ros-base AS runtime

# 런타임 의존성만 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    ros-humble-launch-ros \
    ros-humble-launch \
    # 시리얼 포트 권한 확인용 (선택)
    udev \
    && rm -rf /var/lib/apt/lists/*

# 빌드 결과물 복사
COPY --from=builder /ros2_ws/install /ros2_ws/install

# dialout 그룹 설정 (시리얼 포트 접근용)
# 컨테이너 실행 시 --device /dev/ttyUSBx 추가 필요
RUN groupadd -f dialout && usermod -aG dialout root

# 엔트리포인트 스크립트 복사
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws

# 기본 실행: x4pro_launch.py (port는 환경변수로 오버라이드 가능)
ENV LIDAR_PORT=/dev/ttyUSB0
ENV LIDAR_FRAME_ID=laser_frame

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ydlidar_driver", "x4pro_launch.py"]
