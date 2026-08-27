from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ydlidar_driver')
    default_params = PathJoinSubstitution([pkg_share, 'params', 'x4pro.yaml'])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'port',
                default_value='/dev/ttyUSB0',
                description='YDLidar X4 Pro serial port path',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='laser_frame',
                description='TF frame ID for LaserScan messages',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Path to YAML parameter file',
            ),
            Node(
                package='ydlidar_driver',
                executable='ydlidar_node',
                name='ydlidar_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'port': LaunchConfiguration('port'),
                        'frame_id': LaunchConfiguration('frame_id'),
                    },
                ],
            ),
        ]
    )
