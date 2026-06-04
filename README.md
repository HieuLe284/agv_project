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

# Phân tích

## Pipeline tổng quát
```bash
LiDAR + IMU + Odom
          │
          ▼
      Graph SLAM
          │
          ▼
    Occupancy Grid
          │
     ┌────┴────┐
     ▼         ▼
 Frontier    Costmap (A*)
     │         │
     ▼         ▼
 Goal(x,y)   Path
     │         │
     └────┬────┘
          ▼
    Global Path (vector<waypoint>)
          ▼
 DWA (tự tính goal_angle nội bộ từ path)
          ▼
      cmd_vel
```

------------------------------------------------- Graph SLAM -------------------------------------------------
```
Sơ đồ thuật toán
┌────────────────────────────────────────────────────────┐
│                   GraphSlamCore (Coordinator)          │
│        processLaserScan(), processOdometry()           │
└───────┬──────────────┬──────────────┬──────────────────┘
        │              │              │
    FrontEnd         BackEnd        MapBuilder
 (Nhận dạng ICP,  (Tối ưu hóa G2O, (In-bóng mây quét
  Lập mạng đồ thị) Hủy lỗi Drift)   tạo OccupancyGrid)
```

```
Sơ đồ điều khiển

  Receive Data_Trigger(Odometry, Scan_Msg, IMU):

  1. Front-End (Dựng cây/Local SLAM):
    ┌────────────────────────────────────────┐
    │ ICP Scan Matching / CS Match           │
    │ Khớp mây điểm Laser N với Laser N-1    │
    │ → Suy xuất Biến thiên dời Pose (Δ)     │
    └────────────────────────────────────────┘
           │
           ▼
    Thêm Đỉnh (Node = Vị trí robot) & Cạnh (Edge = Ràng buộc sai số) vào Global Graph
           │
           ▼
    Kiểm tra Loop Closure (Khép Vòng Đo)?
    (Robot phát hiện lại Tường Tọa độ cũ từng đi qua bằng Feature Matching / Branch & Bound)
           │ YES
           ▼
  2. Back-End (Global SLAM):
    Tối ưu hóa Đồ thị Lực xoắn (Ceres Solver / G2O)
    Dùng Least-Square Error xô lệch toàn bộ mạng lưới khớp lùi vè thực tế, xóa Sai số dồn (Drift)
           │
           ▼
  3. MapBuilder:
    updateOccupancy(Nodes) → updateMap logic Log-Odds → Flush (Phóng) mảng 1D Map lên ROS.
```

```
Sơ đồ map (Data Pipeline)

Sensors (LiDAR / IMU / Odom) ──→ Buffer Đồng bộ Message
                                      │
                                 Kiến tạo Front-End
                           (Xây cây Cấu trúc Dữ liệu Pose)
                                      │
                                 Chiết tính Back-End
                   (Giải thuật Bình phương Tối thiểu - Least Squares) 
                                      │
                            PoseGraph Chuẩn Xác (x, y, θ)
                                      │
                                 Tầng MapBuilder 
                         (Chấm xác suất Bayes/Log-Odds)
                      (Value: Free - 0, Occupied - 100, Unk - -1)
                                      │
                                ┌─────┴─────┐
                                ▼           ▼
                        AStarGlobalPlanner  FrontierExploration
```



------------------------------------------------- Frontier Based -------------------------------------------------
```
Sơ đồ thuật toán
┌─────────────────────────────────────────────────────┐
│              FrontierExploration (Coordinator)      │  frontier_exploration.*
│        update(grid) → compute(x,y,θ) → goal(x,y)    │
└───────┬────────────┬──────────────┬─────────────────┘
        │            │              │
   FrontierMap  FrontierDetector  FrontierSelector
   (Lớp dữ   (Phát hiện       (Chọn frontier
    liệu map)  frontier cells)   tốt nhất)
```
```
Sơ đồ điều khiển

  update(M_t) ──→ Cập nhật OccupancyGrid từ SLAM

  compute(x_t, y_t, θ_t):

    ┌──────────────────────────────────────────────┐
    │ need_new_goal?                               │
    │  ├─ !has_goal_                               │
    │  ├─ A* báo signalGoalReached()               │
    │  └─ timeout > 30s (stuck detect)             │
    └──────────────────────────────────────────────┘
           │ YES                              │ NO
           ▼                                  ▼
    detect() → FrontierRegions          (Giữ nguyên goal)
    select() → best region (goal x,y)

    if nullptr → done_ = true
    else → set goal_, reset timer
               │
               ▼
         Gửi goal(x,y) cho A*
```
```
Sơ đồ map
SLAM ──map──→ FrontierMap
                   │
              FrontierDetector ──BFS Wave-Front──→ [FrontierRegion₁, R₂, ...]
                                                          │
                                                   FrontierSelector
                                                   cost = w_dist·D̂ + w_size·(1-Ŝ)
                                                          │ R* = argmin(cost)
                                                   goal(x,y)
                                                          │
                                                       A* Planner
```



------------------------------------------------- A* -------------------------------------------------
```
Sơ đồ thuật toán
┌────────────────────────────────────────────────────────────┐
│               AStarGlobalPlanner (Coordinator)             │
│  updateMap(grid) → setGoal() → compute(x,y,θ) → path       │
└───────┬─────────────────┬────────────────┬─────────────────┘
        │                 │                │
    AStarMap         AStarPlanner    PathSimplifier
 (Tra cứu mảng 1D,  (Lõi BFS / A*   (Cắt tỉa góc tù
  Nới phồng vật cản) Min-heap node)  Douglas-Peucker)
```

```
Sơ đồ điều khiển

  setGoal(goal_x, goal_y) ──→ has_pending_replan = true

  compute(x_t, y_t, θ_t):

    ┌──────────────────────────────────────────────┐
    │ need_replan?                                 │
    │  ├─ has_pending_replan == true               │
    │  ├─ missing / invalid current_path_          │
    │  └─ step_count % replan_interval == 0        │
    └──────────────────────────────────────────────┘
           │ YES                            │ NO
           ▼                                ▼
      runAstar(start, goal)            (Bỏ qua bước vẽ
   (Nếu lỗi: fallback giữ path cũ)      đường chạy xuống)

               ▼
    advance_waypoint (Lookahead): 
      while( dist_to_wp < tolerance ) { current_wp_idx++ }

               ▼
    check_goal (Dừng xe):
      if( dist_to_goal < goal_tolerance ) {
         goal_reached = true
         ──→ báo Frontier::signalGoalReached()
      }

               ▼
    A* chỉ lưu PATH (vector<waypoint>)
    ──→ DWA lấy toàn bộ PATH và tự tính goal_angle
```

```
Sơ đồ map (Data Pipeline)

SLAM ──map_msg──→ updateMap() ──→ AStarMap (Flatten 1D & Bơm phồng Map)
                                        │
                         (Ép hệ mét: WorldToGrid)
                                        │
                                   AStarPlanner
             (Rút node Min-f, Gán node kề, G = g+cost, F = G+H)
                                        │
                                  AStarPath (Raw)
                                        │
                                  PathSimplifier
                            (Lọc Cross-Product điểm thừa)
                                        │
                                 AStarPath (Smooth)
                                        │
                                  AStarGlobalPlanner
                           (Hút Look-ahead vòng tròn Waypoint)
                                        │
                              PATH (vector<waypoint>)
                                        │
                                 DWA Local Planner
```



------------------------------------------------- DWA -------------------------------------------------
```
Sơ đồ thuật toán
┌────────────────────────────────────────────────────────────┐
│                    DWAPlanner (Coordinator)                │
│                 computeVelocity(state, scan, path)         │
└───────┬────────────────┬─────────────────┬─────────────────┘
        │                │                 │ 
   DynamicWindow   TrajectoryScorer   scanToObstacles
   (Chuyển Vd ∩ Vs  (Chấm điểm đa     (Chuyển cản từ Mây cực 
   lấy giới hạn V_f) mục tiêu G-Cost)      Polar → Mây XY)
                LookaheadFinder
            (Tự tìm waypoint từ path
             và tính goal_angle nội bộ)
```

```
Sơ đồ điều khiển

  computeVelocity(state, scan, path):
    
    1. TÍNH GOAL_ANGLE NỘI BỘ:
       findLookaheadGoal(path, robot_pose)
       → Duyệt path tìm waypoint cách robot >= 0.6m
       → goal_angle = atan2(wp_y - robot_y, wp_x - robot_x) - robot_theta

    2. scanToObstacles() → Tọa độ Đề-các (x, y) 

    ┌──────────────────────────────────────────┐
    │ HỆ PREEMPTIVE ESCAPE (Va chạm khẩn cấp)? │
    │  ├─ min_forward_dist < 0.35m (35cm)      │
    └──────────────────────────────────────────┘
           │ YES                             │ NO
           ▼                                 ▼
    Xoay gấp tại chỗ (v=0)             3. DynamicWindow:
    (Dùng goal_angle để chọn hướng)    compute(Vd ∩ Vs) → [v_low, v_high, w_low, w_high]

                                        4. Đải phân mẫu (Grid Sampling)
                                        for v_test in v_samples:
                                          for w_test in w_samples:

                                            ▼
                                        5. Tính Clearance : dist = computeDistance(v,w)
                                        6. Admissible (Va): 
                                           is_safe = v_test <= sqrt(2 * dist * v_break_acc)
                                           if (!is_safe) vứt cụm node.

                                            ▼
                                        7. Chấm Score (Objective Function):
                                           G = α·head + β·clear + γ·vel
                                           if G > max_score → Cập nhật {v*, w*}

                                        8. FALLBACK Mode (Chống Kẹt Không Lối Thoát)
                                           (Cố ép Lùi xe -0.05m/s nếu v*,w* kẹt rác)
```

```
Sơ đồ map (Data Pipeline)

A* ──Path (vector<pair<x,y>>)──→ DWA Planner
                                       │
                          findLookaheadGoal()
                          → Tự tính goal_angle nội bộ
                                       │
                          scanToObstacles()
                          → Polar Scan (r, θ) → (x, y)
                                       │
      ┌────────────────────────────────┴────────────────────────────┐
      │  Trực Đâm thẳng (w ≈ 0)          Đánh Lái vòng (w ≠ 0)     │
      │  Kiểm tra hố lọt thỏm dãi        Sử dụng Định lý Cosin     │
      │  rộng y ≤ R_robot                 tìm góc uốn đâm P        │
      └────────────────────────────────┬────────────────────────────┘
                                       │
                           Clearance Cung tròn (dist)
                                       │
                             TrajectoryScorer
                G = α·head(t_brake) + β·clearance + γ·vel
                                       │
                        Command {v*, w*} To Motor
```


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

# Các lệnh debug:

```bash
ros2 topic echo /joint_states                                # Xem dữ liệu encoder / bánh xe / motor
ros2 topic echo /scan                                       # Xem dữ liệu / Lidar
ros2 topic echo /cmd_vel                                    # Xem tín hiệu điều khiển / output / motor 
ros2 run rqt_console rqt_console
rqt
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /slam_robot/loop_closure_event

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