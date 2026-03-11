# YDLidar X4 Pro ROS 2 Driver

YDLidar X4 Pro를 위한 ROS 2 Humble 드라이버입니다. 외부 SDK 라이브러리에 의존하지 않고 시리얼 프로토콜을 직접 구현하여 가볍고 효율적으로 동작하며, Raspberry Pi 4(ARM64)와 일반 PC(AMD64) 환경을 모두 지원합니다.

## 주요 특징

- **SDK 미사용**: 제조사 제공 SDK 없이 시리얼 통신으로 직접 데이터 파싱 (성능 최적화)
- **멀티 플랫폼**: Docker를 통한 AMD64 및 ARM64(라즈베리 파이) 환경 지원
- **설정 가능**: 스캔 범위(각도/거리), 프레임 ID, 포트 등을 YAML 및 Launch 인자로 조절 가능
- **경량화**: 빌드 및 실행에 필요한 최소한의 의존성만 사용

## 하드웨어 요구사항

- **장치**: YDLidar X4 Pro
- **통신**: USB-to-Serial (기본 128000 baudrate)
- **운영체제**: ROS 2 Humble (Ubuntu 22.04) 권장

## 설치 및 빌드

### 1. 기본 빌드 (로컬 환경)

```bash
# 워크스페이스 이동
cd ~/ros2_ws/src
git clone <repository_url> ydlidar_driver

# 의존성 설치
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --packages-select ydlidar_driver
source install/setup.bash
```

### 2. Docker 빌드

이미지를 직접 빌드하여 실행 환경을 격리할 수 있습니다.

```bash
./run_ydlidar_docker.sh --build
```

## 사용 방법

### ROS 2 Launch 실행

기본 설정으로 드라이버를 실행합니다.

```bash
ros2 launch ydlidar_driver x4pro_launch.py
```

특정 포트나 프레임 ID를 지정하여 실행할 수도 있습니다.

```bash
ros2 launch ydlidar_driver x4pro_launch.py port:=/dev/ttyUSB1 frame_id:=laser_link
```

### Docker 실행

제공되는 헬퍼 스크립트를 사용하여 간편하게 실행할 수 있습니다.

```bash
# 기본 포트(/dev/ttyUSB0)로 실행
./run_ydlidar_docker.sh

# 특정 포트 지정 실행
./run_ydlidar_docker.sh -p /dev/ttyUSB1 -f lidar_link
```

## 파라미터 설정

`params/x4pro.yaml` 파일에서 주요 동작 방식을 설정할 수 있습니다.

| 파라미터 | 타입 | 기본값 | 설명 |
| :--- | :--- | :--- | :--- |
| `port` | string | `/dev/ttyUSB0` | LiDAR 연결 시리얼 포트 경로 |
| `frame_id` | string | `laser_frame` | LaserScan 메시지의 TF 프레임 ID |
| `baudrate` | int | `128000` | 통신 속도 (X4 Pro 고정값) |
| `angle_min` | double | `-180.0` | 스캔 시작 각도 (도) |
| `angle_max` | double | `180.0` | 스캔 종료 각도 (도) |
| `range_min` | double | `0.1` | 최소 유효 거리 (m) |
| `range_max` | double | `12.0` | 최대 유효 거리 (m) |
| `invalid_range_is_inf` | bool | `false` | 유효 범위 밖 데이터를 inf(true) 또는 0(false)으로 처리 |

## 발행 데이터

- **Topic**: `/scan` (`sensor_msgs/msg/LaserScan`)
- **설명**: LiDAR에서 측정된 거리 및 강도(intensity) 데이터를 ROS 표준 메시지로 발행합니다.

## 라이선스

이 프로젝트는 [Apache-2.0](LICENSE) 라이선스를 따릅니다.
