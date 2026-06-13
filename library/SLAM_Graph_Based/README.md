## 🗺️ SƠ ĐỒ THUẬT TOÁN GRAPH SLAM — TOÀN BỘ HỆ THỐNG

```
┌══════════════════════════════════════════════════════════════════════════┐
│                    GRAPH-BASED SLAM PIPELINE                              │
│           Grisetti et al. (2010) — IEEE Intell. Transp. Syst. Mag.       │
└══════════════════════════════════════════════════════════════════════════┘

                         ┌─────────────────┐
                         │     /agv_scan   │
                         │  (LaserScan)    │
                         └────────┬────────┘
                                  │
                                  ▼
                    ┌─────────────────────────────┐
                    │     scanCallback()          │  ← 10 Hz
                    │     [LiDAR thread]          │
                    │                             │
                    │  • Cache scan ranges        │
                    │  • Log 8-direction dists    │
                    │  • Publish scan viz         │
                    │  • buildOccupancyGrid()     │
                    │    (mỗi 2 lần)              │
                    │                             │
                    │  ┌─────────────────────┐    │
                    │  │ UPDATE MAP          │    │
                    │  │ lookup map→base_link│    │
                    │  │ map_builder_.       │    │
                    │  │   updateFromRanges() │    │
                    │  └─────────┬───────────┘    │
                    └───────────┬─────────────────┘
                                │
                                ▼
          ┌─────────────────────────────────────────────┐
          │        slamTimerCallback()                  │  ← 5 Hz
          │        [SLAM thread]                        │
          │                                             │
          │  • broadcastMapOdomTF()  (identity)         │
          │      map ── [0,0,0] ──→ odom                │
          │                                             │
          │  • lookupTransform("odom","base_link")      │
          │    → (ox, oy, otheta)                       │
          │                                             │
          │  • graphSLAMcall(ox, oy, otheta)  ──────┐   │
          └─────────────────────────────────────────│───┘
                                                    │
                                                    ▼
          ┌─────────────────────────────────────────────────────────────┐
          │          graphSLAMcall(x, y, theta)                        │
          │          [THE CORE — 4 STEPS]                              │
          │                                                             │
          │  ┌─────────────────────────────────────────────────────┐   │
          │  │ STEP 1 — FRONT-END (ODOMETRY)                      │   │
          │  │                                                     │   │
          │  │  addOdometryNode(x, y, θ, scan):                   │   │
          │  │                                                     │   │
          │  │  • Check travel: ‖t−t_prev‖ > 0.3m  OR             │   │
          │  │                  |θ−θ_prev| > 0.15 rad?            │   │
          │  │    → No  ⇒ return -1 (skip)                        │   │
          │  │    → Yes ⇒ continue                                │   │
          │  │                                                     │   │
          │  │  • NEW NODE: x_i = (x, y, θ) + cached scan         │   │
          │  │    pose_graph.nodes.push_back(x_i)                  │   │
          │  │                                                     │   │
          │  │  • ODOM EDGE z_{i-1→i}:                             │   │
          │  │    δt = R_{prev}^T · (t_i − t_{prev})              │   │
          │  │        ┌                 ┐                          │   │
          │  │        │ cosθ  sinθ      │   translated body-frame  │   │
          │  │        │ -sinθ cosθ      │                          │   │
          │  │        └                 ┘                          │   │
          │  │    δθ = normalize(θ_i − θ_{prev})                  │   │
          │  │                                                     │   │
          │  │  • Ω_odom = diag(50, 50, 80)                       │   │
          │  │    → pose_graph.addEdge(i-1, i, z, Ω)              │   │
          │  └──────────────────────┬──────────────────────────────┘   │
          │                         │                                  │
          │                         ▼                                  │
          │  ┌─────────────────────────────────────────────────────┐   │
          │  │ STEP 2 — FRONT-END (LOOP CLOSURE)                  │   │
          │  │                                                     │   │
          │  │  addLoopClosures(new_idx):                          │   │
          │  │                                                     │   │
          │  │  Stage 1 — PROXIMITY FILTER                         │   │
          │  │    for each old node j < new_idx:                   │   │
          │  │      if |new_idx−j| < 10  ⇒ skip (too close)        │   │
          │  │      if ‖t_j−t_i‖ < 2.0m  AND                       │   │
          │  │         |θ_j−θ_i| < 1.2 rad ⇒ candidate             │   │
          │  │                                                     │   │
          │  │  Stage 2 — SCAN CORRELATION                         │   │
          │  │    C = Σ r_i·r_j / (‖r_i‖·‖r_j‖)                   │   │
          │  │    if C > 0.80  ⇒ MATCH!                            │   │
          │  │                                                     │   │
          │  │  Stage 3 — Relative pose:                           │   │
          │  │    δt = R_i^T·(t_j−t_i),  δθ = norm(θ_j−θ_i)      │   │
          │  │    Ω_loop = diag(200, 200, 400)                     │   │
          │  │    → pose_graph.addEdge(i, j, z, Ω, loop=true)     │   │
          │  │    → loop_closure_count_++                          │   │
          │  │    → new_loop_this_step_ = true                     │   │
          │  └──────────────────────┬──────────────────────────────┘   │
          │                         │                                  │
          │                         ▼                                  │
          │  ┌─────────────────────────────────────────────────────┐   │
          │  │ STEP 3 — BACK-END (GAUSS-NEWTON OPTIMIZATION)      │   │
          │  │                                                     │   │
          │  │  optimizeIfNeeded():                                │   │
          │  │    if !new_loop_this_step_ ⇒ return false           │   │
          │  │                                                     │   │
          │  │  For each iteration (default 10):                   │   │
          │  │                                                     │   │
          │  │  ┌─────────────────────────────────────────────┐    │   │
          │  │  │  BUILD LINEAR SYSTEM: H·Δξ = −b             │    │   │
          │  │  │                                             │    │   │
          │  │  │  For each edge (i,j) with z_ij, Ω_ij:      │    │   │
          │  │  │   • e_ij = error(x_i, x_j, z_ij)           │    │   │
          │  │  │   • A_ij, B_ij = J(e_ij)/J(x_i, x_j)       │    │   │
          │  │  │   • H_ii += A^T·Ω·A                        │    │   │
          │  │  │   • H_ij += A^T·Ω·B                        │    │   │
          │  │  │   • H_ji += B^T·Ω·A                        │    │   │
          │  │  │   • H_jj += B^T·Ω·B                        │    │   │
          │  │  │   • b_i    += A^T·Ω·e                      │    │   │
          │  │  │   • b_j    += B^T·Ω·e                      │    │   │
          │  │  └─────────────────────────────────────────────┘    │   │
          │  │                                                     │   │
          │  │  • GAUGE FIX: anchor node 0 (identity)              │   │
          │  │                                                     │   │
          │  │  • SOLVE: Gaussian elimination                     │   │
          │  │    H·Δξ = −b  →  Δξ ∈ R^{3N}                      │   │
          │  │                                                     │   │
          │  │  • UPDATE all nodes:                                │   │
          │  │    x_i += Δξ[3i+0]                                  │   │
          │  │    y_i += Δξ[3i+1]                                  │   │
          │  │    θ_i += Δξ[3i+2]  (normalized)                   │   │
          │  │                                                     │   │
          │  │  • return true                                      │   │
          │  └──────────────────────┬──────────────────────────────┘   │
          │                         │                                  │
          │                         ▼                                  │
          │  ┌─────────────────────────────────────────────────────┐   │
          │  │ STEP 4 — MAP REBUILD (if optimized)                │   │
          │  │                                                     │   │
          │  │  if optimized:                                      │   │
          │  │    • slam_graph_.clearMap()                         │   │
          │  │    • For each node i (1..N−1):                     │   │
          │  │        scan = node[i].scan_ranges                   │   │
          │  │        updateFromRanges(scan, n.x, n.y, n.θ)       │   │
          │  │    → Map được rebuild từ pose đã tối ưu            │   │
          │  └─────────────────────────────────────────────────────┘   │
          │                                                             │
          │  ┌─────────────────────────────────────────────────────┐   │
          │  │ VISUALIZATION — publish every SLAM timer tick       │   │
          │  │                                                     │   │
          │  │  • /slam_robot/graph_nodes  — PoseArray             │   │
          │  │    (all node poses trong map frame)                 │   │
          │  │                                                     │   │
          │  │  • /slam_robot/graph_edges — MarkerArray            │   │
          │  │    - blue lines  = odometry edges                   │   │
          │  │    - red lines   = loop-closure edges               │   │
          │  │                                                     │   │
          │  │  • /slam_robot/loop_closure_event — String           │   │
          │  └─────────────────────────────────────────────────────┘   │
          └─────────────────────────────────────────────────────────────┘


┌══════════════════════════════════════════════════════════════════════════┐
│                TF TREE (SAU KHI SỬA — map = odom)                       │
└══════════════════════════════════════════════════════════════════════════┘

   map  ──────────────────────────────────────────────────────────────────┐
        │ [identity]  broadcastMapOdomTF()                                │
        │  (0, 0, 0)  fixed!                                              │
        ▼                                                                 │
   odom  ──────────────────────────────────────────────────────────────┐  │
        │ [odometry]  robot controller → /odom → /tf                    │  │
        │  (x, y, θ)  raw odometry, drifting                            │  │
        ▼                                                                ▼  ▼
   base_link ─────────────────────────────────────────────────────────────────


┌══════════════════════════════════════════════════════════════════════════┐
│                  ERROR FUNCTION & JACOBIANS                              │
└──────────────────────────────────────────────────────────────────────────┘

   Error e_ij ∈ R³:
        ┌                                 ┐
        │  R_ij^T · (R_i^T·(t_j−t_i) − t_ij) │
   e =  │  normalize(θ_j − θ_i − θ_ij)     │
        └                                 ┘

   Jacobians:
   A_ij = J(e)/J(x_i) ∈ R^{3×3}
   B_ij = J(e)/J(x_j) ∈ R^{3×3}

   Hessian blocks:
   H_ii += A^T·Ω·A
   H_ij += A^T·Ω·B
   H_ji += B^T·Ω·A
   H_jj += B^T·Ω·B
   b_i  += A^T·Ω·e
   b_j  += B^T·Ω·e


┌══════════════════════════════════════════════════════════════════════════┐
│                  OCCUPANCY GRID MAPPING                                  │
└──────────────────────────────────────────────────────────────────────────┘

   MapBuilder (Log-Odds):
        scan ──→ bresenham ray-cast
                 ├── free cells: l += -0.4
                 └── hit cell:   l += +0.85  (max 5.0)

   Publish: /map (frame_id = "map")
        32m × 20m, 0.05m/cell, origin (-17, -10)


┌══════════════════════════════════════════════════════════════════════════┐
│                  CẤU TRÚC DỮ LIỆU                                        │
└──────────────────────────────────────────────────────────────────────────┘

   PoseGraph2D:
        ┌──────────────────────┐
        │  nodes: vector<Node2D>     │  ← vertex: (x, y, θ) + scan
        │  edges: vector<Edge2D>     │  ← (from, to, z, Ω)
        └──────────────────────┘

   Node2D:
        ┌──────────────────────┐
        │  x, y, θ                    │  ← global pose
        │  scan_ranges, angle_min,    │  ← cached LiDAR data
        │  scan_angle_increment       │
        └──────────────────────┘

   Edge2D:
        ┌──────────────────────┐
        │  from, to, is_loop          │  ← indices
        │  z_x, z_y, z_theta         │  ← relative measurement
        │  omega[3][3]               │  ← information matrix
        └──────────────────────┘
```

---

## 📋 DÒNG CHẢY CHÍNH

```
                    ┌──────────────┐
                    │   Gazebo     │
                    │  (simulation) │
                    └──────┬───────┘
                           │ /agv_scan
                           ▼
              ┌─────────────────────┐
              │  scanCallback()     │  ← 10 Hz
              │  ┌───────────────┐  │
              │  │ updateFromRanges │  → /map (OccupancyGrid)
              │  └───────────────┘  │
              │  + cache scan       │
              └──────────┬──────────┘
                         │
              ┌─────────────────────┐
              │  slamTimerCallback()│  ← 5 Hz
              │  lookup odom→base   │
              └──────────┬──────────┘
                         │ (ox, oy, otheta)
                         ▼
              ┌─────────────────────────────────────┐
              │  graphSLAMcall(ox, oy, otheta)      │
              │                                      │
              │  1. addOdometryNode()                │
              │     → thêm node + edge odom          │
              │                                      │
              │  2. addLoopClosures()                │
              │     → tìm candidate + scan match     │
              │                                      │
              │  3. optimizeIfNeeded()              │
              │     → Gauss-Newton solve             │
              │     → nếu optimized:                 │
              │       4. clearMap() + rebuild        │
              │          (dùng pose đã tối ưu)        │
              │                                      │
              │  5. publish graph viz                │
              │     (nodes + edges)                  │
              └──────────────────────────────────────┘
```

### Sơ đồ thuật toán
```
------------------------------------------------- Graph SLAM -------------------------------------------------
                                      ┌───────────────┐
                                      │    BẮT ĐẦU    │
                                      └───────┬───────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Khởi tạo ROS2 Node            │
                                      │ - Subscriber /scan            │
                                      │ - TF Listener                 │
                                      │ - Pose Graph                  │
                                      │ - Loop Closure Detector       │
                                      │ - Optimizer                   │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Chờ dữ liệu LiDAR             │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Nhận LaserScan                │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Lưu scan vào bộ nhớ đệm       │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Lấy pose robot từ TF          │
                                      │ (x, y, yaw)                   │
                                      └───────┬───────────────────────┘
                                              │
                                              ▼
                                      ┌───────────────────────────────┐
                                      │ Đủ điều kiện tạo node mới ?   │
                                      └───────┬────────────────┬──────┘
                                              │Không           │Có
                                              │                │
                                              ▼                ▼
                                      ┌───────────────┐   ┌─────────────────────┐
                                      │ Quay lại chờ  │   │ Tạo node mới        │
                                      │ scan tiếp theo│   └─────────┬───────────┘
                                      └───────────────┘             │
                                                                    ▼
                                                          ┌─────────────────────┐
                                                          │ Tạo odometry edge   │
                                                          └─────────┬───────────┘
                                                                    │
                                                                    ▼
                                                          ┌─────────────────────┐
                                                          │ Phát hiện loop      │
                                                          │ closure             │
                                                          └─────────┬───────────┘
                                                                    │
                                                                    ▼
                                                      ┌──────────────────────────┐
                                                      │ Có loop closure không ?  │
                                                      └─────────┬──────────┬─────┘
                                                                │Không     │Có
                                                                │          │
                                                                ▼          ▼
                                                      ┌────────────────┐  ┌─────────────────────┐
                                                      │ Bỏ qua         │  │ Thêm loop edge      │
                                                      └───────┬────────┘  └─────────┬───────────┘
                                                              │                     │
                                                              └──────────┬──────────┘
                                                                          ▼
                                                          ┌──────────────────────────┐
                                                          │ Tối ưu hóa pose graph    │
                                                          │ (Gauss-Newton)           │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Cập nhật lại pose node   │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Xóa map cũ               │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Vẽ lại map từ graph      │
                                                          │ đã tối ưu                │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Publish Occupancy Grid   │
                                                          └─────────┬────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌──────────────────────────┐
                                                          │ Quay lại chờ scan tiếp   │
                                                          └──────────────────────────┘
```