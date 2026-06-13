/**
 * @file astar_planner.cpp
 *
 * ============================================================
 *  TOÁN HỌC — THUẬT TOÁN A* ĐẦY ĐỦ
 * ============================================================
 *
 * 1. KHỞI TẠO
 *    g(S) = 0,  h(S) = H(S, G),  f(S) = g(S) + h(S)
 *    Push S vào Open Set
 *
 * 2. VÒNG LẶP CHÍNH
 *    While Open Set ≠ ∅ AND iter < max_iter:
 *      n = pop node có f(n) nhỏ nhất (min-heap)
 *      If n == G: trace-back → return path
 *      If n ∈ Closed: skip
 *      Mark n ∈ Closed
 *      For m ∈ N_8(n) (8-connectivity):
 *        If not traversable OR m ∈ Closed: skip
 *        move_cost = 1.0 (ngang/dọc) | √2 (chéo)
 *        obstacle_cost = f(dist_to_nearest_obstacle)
 *        g_new = g(n) + move_cost + obstacle_cost
 *        If g_new < g_score[m]:
 *          g_score[m] = g_new
 *          f_new = g_new + w * H(m, G)
 *          parent[m] = n
 *          Push AStarNode(m, g_new, h) vào Open Set
 *
 * 3. TRACE-BACK
 *    Bắt đầu từ G, đi ngược theo parent[] cho đến khi gặp S
 *    Đảo ngược mảng → path từ S → G
 *
 * 4. HEURISTIC
 *    MANHATTAN : h = |dx| + |dy|
 *    EUCLIDEAN : h = √(dx² + dy²)
 *    DIAGONAL  : h = max(dx,dy) + (√2 - 1) * min(dx,dy)
 * ============================================================
 */

#include "library/A_star_algorithm/include/astar_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

// Hằng số √2 dùng cho chi phí di chuyển chéo
static constexpr double SQRT2 = 1.41421356237;

// 8 hướng di chuyển: {dx, dy, chi_phi} xung quanh 1 cell (x,y) trên grid được ánh xạ như sau:
// (-1,-1) (-1,0) (-1,1)
// ( 0,-1) ( x,y) ( 0,1)
// ( 1,-1) ( 1,0) ( 1,1)
static const int DX[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
static const int DY[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
static const double MOVE_COST[8] = {SQRT2, 1.0, SQRT2, 1.0, 1.0, SQRT2, 1.0, SQRT2};

// ================================================================
//  Heuristic h(n)
// ================================================================
double AStarPlanner::computeHeuristic(
    int x, int y, int gx, int gy,
    const AStarConfig& config) const
{
  double dx = std::abs(gx - x);
  double dy = std::abs(gy - y);
  double h  = 0.0;

  switch (config.heuristic) {
    case HeuristicType::MANHATTAN:
      // Manhattan distance (L1 Norm)
      // h_Manhattan = ||pG - pn||_1 = |dx| + |dy|
      // h = |dx| + |dy|
      h = dx + dy;
      break;

    case HeuristicType::EUCLIDEAN:
      // Euclidean distance (L2 Norm)
      // h_Euclidean = ||pG - pn||_2 = sqrt(dx^2 + dy^2)
      // h = √(dx² + dy²)
      h = std::sqrt(dx * dx + dy * dy);
      break;

    case HeuristicType::DIAGONAL:
      // Diagonal distance (Chebyshev distance)
      // h_Diagonal = ||pG - pn||_inf = Δmax + (√2 - 1) * Δmin
      // Trong đó:
      //  - Δmax = max(dx, dy)
      //  - Δmin = min(dx, dy)
      //  - dx = |x_n - x_g|
      //  - dy = |y_n - y_g|
      // Tương đương: h = max(dx, dy) + (√2 - 1) * min(dx, dy)
      h = std::max(dx, dy) + (SQRT2 - 1.0) * std::min(dx, dy);
      break;
  }

  // Nhân với trọng số (Weighted A*)
  return config.heuristic_weight * h;
}

// ================================================================
//  Trace-back: đi ngược từ goal về start theo mảng parent
// ================================================================
std::vector<std::pair<int, int>> AStarPlanner::tracePath(
    int goal_x, int goal_y,
    const std::vector<int>& parent, int W,
    int start_x, int start_y) const
{
  std::vector<std::pair<int, int>> path;
  int cx = goal_x, cy = goal_y; // (cx, cy) là tọa độ hiện tại

  // Đi ngược từ Goal → Start theo mảng parent
  while (!(cx == start_x && cy == start_y) && rclcpp::ok()) {
    path.push_back({cx, cy});
    int p_idx = parent[cy * W + cx]; // lấy nút cha của node hiện tại 
    if (p_idx == -1) break;  // Không tìm được trace
    cx = p_idx % W;
    cy = p_idx / W;
  }
  // Không thêm start vào path (robot đã ở đó)

  // Đảo ngược để có thứ tự từ start → goal
  std::reverse(path.begin(), path.end());
  return path;
}

// ================================================================
//  A* CHÍNH
// ================================================================
std::vector<std::pair<int, int>> AStarPlanner::plan(
    int start_x, int start_y,
    int goal_x,  int goal_y,
    const AStarMap&    map,
    const AStarConfig& config) const
{
  if (!map.hasMap()) return {};
  if (!map.isValid(start_x, start_y)) return {};
  if (!map.isValid(goal_x, goal_y))   return {};
  if (!map.isTraversable(goal_x, goal_y)) return {};  // Goal bị chặn

  const int W = map.getWidth();
  const int H = map.getHeight();

  // Tối ưu hóa: Thay 2D vector bằng 1D vector để tránh cấp phát động quá mức (gây nghẽn SLAM).
  // g_score[y * W + x] = chi phí tốt nhất từ Start đến (x,y)
  std::vector<double> g_score(W * H, std::numeric_limits<double>::max());

  // parent[y * W + x] = 1D index của cha
  std::vector<int> parent(W * H, -1);

  // Closed set: cell đã được expand xong
  std::vector<uint8_t> closed(W * H, 0);

  // Open set: min-heap theo f = g + h
  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

  // Khởi tạo Start node
  double h_start = computeHeuristic(start_x, start_y, goal_x, goal_y, config);
  g_score[start_y * W + start_x] = 0.0;
  open_set.push(AStarNode(start_x, start_y, 0.0, h_start, -1, -1));

  int iter = 0;

  while (!open_set.empty() && iter++ < config.max_iterations && rclcpp::ok()) {
    // Lấy node có f nhỏ nhất
    AStarNode cur = open_set.top();
    open_set.pop();

    // Đã đến Goal!
    if (cur.x == goal_x && cur.y == goal_y) {
      return tracePath(goal_x, goal_y, parent, W, start_x, start_y);
    }

    // Bỏ qua nếu đã xét (lazy deletion heap)
    if (closed[cur.y * W + cur.x]) continue;
    closed[cur.y * W + cur.x] = 1;

    // Expand 8 neighbors
    for (int d = 0; d < 8; ++d) {
      int nx = cur.x + DX[d];
      int ny = cur.y + DY[d];

      if (!map.isValid(nx, ny))      continue;
      if (closed[ny * W + nx])       continue;
      if (!map.isTraversable(nx, ny)) continue;

      // Chi phí di chuyển + chi phí gần vật cản có công thức như sau:
      // g_new = g(cur) + c(cur, n_neighbor) + c_obs(nneighbor)
      double move_cost     = MOVE_COST[d];
      double obstacle_cost = map.computeObstacleCost(nx, ny, config.safety_margin, config.obstacle_penalty);
      double g_new = cur.g + move_cost + obstacle_cost;

      // Chỉ update nếu tìm được đường tốt hơn
      if (g_new < g_score[ny * W + nx]) {
        g_score[ny * W + nx] = g_new;
        parent[ny * W + nx]  = cur.y * W + cur.x;

        double h_new = computeHeuristic(nx, ny, goal_x, goal_y, config);
        open_set.push(AStarNode(nx, ny, g_new, h_new, cur.x, cur.y));
      }
    }
  }

  return {};  // Không tìm được đường
}