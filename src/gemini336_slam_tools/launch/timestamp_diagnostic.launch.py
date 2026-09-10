from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py',
            ])
        ),
        launch_arguments={
            'enable_depth': 'false',
            'enable_laser': 'false',
            'enable_left_ir': 'true',
            'enable_right_ir': 'true',
            'enable_sync_output_accel_gyro': 'true',
        }.items(),
    )

    timestamp_diagnostic = Node(
        package='gemini336_slam_tools',
        executable='timestamp_diagnostic_node',
        name='timestamp_diagnostic_node',
        output='screen',
    )

    return LaunchDescription([
        orbbec_camera,
        timestamp_diagnostic,
    ])
