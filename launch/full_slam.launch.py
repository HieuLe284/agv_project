import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    agv_robot_dir = get_package_share_directory('agv_robot')

    # 1. Khởi động Gazebo (bao gồm cả robot state publisher và spawn robot)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agv_robot_dir, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. Mở RViz với cấu hình slam.rviz
    rviz_config_dir = os.path.join(agv_robot_dir, 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # 3. Chạy node slam_robot (Graph SLAM + Frontier + A* + DWA)
    slam_robot_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='agv_robot',
                executable='slam_robot',
                name='slam_robot',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_node,
        slam_robot_node
    ])
