from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ydlidar_driver")
    default_params = PathJoinSubstitution([pkg_share, "params", "x4pro.yaml"])

    return LaunchDescription(
        [
            # ---- launch 인자 ----
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="YDLidar X4 Pro 시리얼 포트 경로",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="laser_frame",
                description="LaserScan 메시지의 TF 프레임 ID",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="파라미터 YAML 파일 경로",
            ),
            # ---- 노드 ----
            Node(
                package="ydlidar_driver",
                executable="ydlidar_node",
                name="ydlidar_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    # launch 인자로 YAML 값 덮어쓰기
                    {
                        "port": LaunchConfiguration("port"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    },
                ],
            ),
        ]
    )
