import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Khai báo các đường dẫn
    pkg_agv_robot = get_package_share_directory('agv_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Đường dẫn file bản đồ (map/map.yaml)
    map_file = os.path.join(pkg_agv_robot, 'map', 'map.yaml')
    
    # Đường dẫn file tham số Nav2
    params_file = os.path.join(pkg_agv_robot, 'config', 'nav2_params.yaml')

    # 2. Khai báo Include để gọi launch file bringup chuẩn của Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        nav2_bringup_launch
    ])
