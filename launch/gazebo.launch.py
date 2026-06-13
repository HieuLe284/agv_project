import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'agv_robot'

    # ---- Robot Description (qua Xacro) ----
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'agv_robot.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ---- World file  ----
    pkg_share = get_package_share_directory(pkg_name)
    world_file = os.path.join(pkg_share, 'worlds', 'experiment_rooms', 'worlds', 'room2', 'world.model')

    # Thiết lập đường dẫn model để Gazebo tìm thấy các vật thể trong world mới
    model_path = os.path.join(pkg_share, 'worlds') + os.pathsep + \
                 os.path.join(pkg_share, 'worlds', 'experiment_rooms', 'models')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path

    # ---- Khởi động Gazebo Classic + load world ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        # Truyền world file để Gazebo có ground plane
        launch_arguments={'world': world_file}.items()
    )

    # ---- Robot State Publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # ---- Spawn Robot vào Gazebo ----
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'agv_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # ---- C++ Node: TF2 Listener ----
    tf2_listener = Node(
        package='agv_robot',
        executable='tf2_listener',
        name='tf2_listener',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=5.0, actions=[tf2_listener]),
    ])
