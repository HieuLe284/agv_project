import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    agv_robot_dir = get_package_share_directory('agv_robot')

    slam_params_file = os.path.join(agv_robot_dir, 'config', 'slam_toolbox_params.yaml')

    # Khởi động SLAM Toolbox với remap topic /scan -> /agv_scan
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ],
        remappings=[
            # Remap topic /scan (mặc định của slam_toolbox) sang /agv_scan (topic Lidar của robot)
            ('/scan', '/agv_scan'),
        ]
    )

    return LaunchDescription([
        # Đã comment slam_node theo yêu cầu để vô hiệu hóa SLAM Toolbox
        slam_node,
    ])
