# ROS2 AGV Robot

ROS2 AGV robot demo using Gazebo and RViz
Sử dụng các tutorial như:
Topic: publish & subscribe
TF: publish & lookup
Parameter
Service
Action
Launch files
URDF & XACRO
Gazebo & RViz
Lidar sử dụng theo thuật toán sau:
- SLAM: dùng thuật toán Graph SLAM / Pose Graph Optimization: Tạo map, xác định vị trí của robot
- Exploration: dùng thuật toán Frontier-based: Chọn đường tốt nhất ( đường mà map chưa mở ) => Quyết định đi đâu tiếp theo 
- Global Planner: dùng thuật toán A star (A*) algorithm: Tìm đường tối ưu từ robot -> goal 
- Local Planner: dùng thuật toán Dynamic Window Approach (DWA): Điều khiển robot đi theo thực tế ( tránh vật cản realtime )

Navigation sử dụng theo thuật toán Dynamic Window Approach (DWA)

# Yêu cầu

- ROS2 Humble
- Gazebo (v11)
- RViz2

# Tạo workspace

```bash
cd ...                  #folder lưu workspace
mkdir -p ~/agv_robot
cd agv_robot

# Build
rosdep install-i--from-path src--rosdistro humble-y
colcon build
. install/setup.bash
```

# Các chương trình chạy test

# Điều khiển robot

```bash
# Trong terminal 1
ros2 launch agv_robot gazebo.launch.py

# Trong terminal 2 (điều khiển robot)
. install/setup.bash && ros2 run agv_robot robot_controller


# Trong terminal 3 (điều khiển action server)
. install/setup.bash && ros2 run agv_robot action_server

# Trong terminal 4 (điều khiển action client)
. install/setup.bash && ros2 action send_goal /move_robot agv_robot/action/MoveCmd "{command: 'up', value: 5}"        # Robot sẽ đi lên trên 5m
. install/setup.bash && ros2 action send_goal /move_robot agv_robot/action/MoveCmd "{command: 'down', value: 12}"     # Robot sẽ đi xuống dưới 12m
. install/setup.bash && ros2 action send_goal /move_robot agv_robot/action/MoveCmd "{command: 'circle', value: 1.37}" # Robot sẽ đi theo hình tròn với góc 1.37 radian
```

# SLAM
```bash
#Terminal
ros2 launch agv_robot full_slam.launch.py

# Terminal 1
ros2 launch agv_robot gazebo.launch.py

# Terminal 2
. install/setup.bash && ros2 launch agv_robot slam.launch.py

# Terminal 3 - Mở RViz với config sẵn
. install/setup.bash && rviz2 -d /home/hieuubuntu/share/AGV_Robot/rviz/slam.rviz
. install/setup.bash && rviz2 -d /home/hieu/Hieu/Project/AGV_Robot/rviz/slam.rviz

# Terminal 4 — Chạy Graph SLAM + tránh vật cản
. install/setup.bash && ros2 run agv_robot slam_robot

# Terminal 5 - Lưu file bản đồ
. install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /home/hieuubuntu/share/AGV_Robot/map/map
```

# Navigation

```bash
cd ~/share/AGV_Robot
colcon build --packages-select agv_robot
source install/setup.bash

# Terminal 1
ros2 launch agv_robot gazebo.launch.py

# Terminal 2 (Khởi động Navigation)
. install/setup.bash && ros2 launch agv_robot navigation.launch.py

# Terminal 3 (Mở RViz Navigation)
. install/setup.bash && ros2 launch agv_robot rviz2.launch.py
. install/setup.bash && rviz2 -d /home/hieuubuntu/share/AGV_Robot/rviz/navigation.rviz

# Terminal 4 (Mở RViz Navigation)
. install/setup.bash && ros2 run agv_robot navigation_robot

```

```bash
# Chạy với Gazebo Room 1
export GAZEBO_MODEL_PATH=/home/<user_name>/experiment_rooms/models/
cd experiment_rooms/worlds/room1
gazebo world_dynamic.model
```

# Xóa build, log, install

```bash
rm -rf build install log
```

# remote-SSH WINDOW to UBUNTU
```bash
export DISPLAY=:0
```

# Tạo ảnh từ file .gv
```bash
dot -Tpng graph_slam_diagram.gv -o graph_slam_diagram.png
dot -Tsvg graph_slam_diagram.gv -o graph_slam_diagram.svg

dot -Tpng frontier_based_diagram.gv -o frontier_based_diagram.png
dot -Tsvg frontier_based_diagram.gv -o frontier_based_diagram.svg

dot -Tpng astar_algorithm_diagram.gv -o astar_algorithm_diagram.png
dot -Tsvg astar_algorithm_diagram.gv -o astar_algorithm_diagram.svg

dot -Tpng dwa_algorithm_diagram.gv -o dwa_algorithm_diagram.png
dot -Tsvg dwa_algorithm_diagram.gv -o dwa_algorithm_diagram.svg
```

# Các lệnh debug:

```bash
ros2 run rqt_console rqt_console
rqt
ros2 topic echo /joint_states                                # Xem dữ liệu encoder / bánh xe / motor
ros2 topic echo /scan                                       # Xem dữ liệu / Lidar
ros2 topic echo /cmd_vel                                    # Xem tín hiệu điều khiển / output / motor 
ros2 topic echo /slam_robot/loop_closure_event
ros2 topic echo /agv_scan --once
ros2 topic echo /tf --once
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_monitor
ros2 run tf2_tools view_frames
ros2 param get /rviz2 use_sim_time
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /slam_node use_sim_time
ros2 param get /dwa_node use_sim_time
ros2 param get /frontier_node use_sim_time

```

# Các lệnh git:
```bash
cd ~/Hieu/Project/AGV_Robot

# máy chính
git status
git add .
git commit -m "..."
git push

# máy phụ
git clone https://github.com/HieuLe284/AGV_Robot.git
git pull
git add .
git commit -m "Update DWA planner"
git push
```