#!/bin/bash
# ROS2 환경 소싱 후 CMD 실행

set -e

# ROS2 Humble 기본 환경
source /opt/ros/humble/setup.bash

# 빌드된 패키지 오버레이
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# 환경변수를 launch 인자로 전달
# CMD가 ros2 launch 인 경우 port / frame_id 자동 주입
if [[ "$1" == "ros2" && "$2" == "launch" ]]; then
    exec "$@" \
        port:="${LIDAR_PORT:-/dev/ttyUSB0}" \
        frame_id:="${LIDAR_FRAME_ID:-laser_frame}"
else
    exec "$@"
fi
