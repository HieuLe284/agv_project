#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "library/A_star_algorithm/include/astar_config.h"
#include "library/A_star_algorithm/include/astar_map.h"
#include "library/A_star_algorithm/include/astar_node.h"

#include <utility>
#include <vector>

/**
 * @file astar_planner.h
 * @brief Thuật toán A* tìm đường tối ưu trên OccupancyGrid
 *
 * ============================================================
 *  TOÁN HỌC — THUẬT TOÁN A* TRÊN LƯỚI 8-CONNECTIVITY
 * ============================================================
 *
 * INPUT:
 *   Start S = (sx, sy) — vị trí robot trong grid
 *   Goal  G = (gx, gy) — vị trí đích trong grid
 *   Map   M             — OccupancyGrid đã parse
 *   Config              — Tham số cấu hình
 *
 * CẤU TRÚC DỮ LIỆU:
 *   Open Set  : priority_queue<AStarNode, min-heap theo f>
 *   Closed Set: 2D vector<bool> — cell đã được xét xong
 *   g_score   : 2D vector<double> — chi phí tốt nhất từ S đến mỗi cell
 *   parent    : 2D vector<pair<int,int>> — cell cha để trace-back
 *
 * THUẬT TOÁN:
 *   1. g_score[S] = 0; push AStarNode(S, g=0, h=H(S,G)) vào Open Set
 *   2. Lặp khi Open Set không rỗng (và iter < max_iterations):
 *      a. Pop node n có f(n) nhỏ nhất từ Open Set
 *      b. Nếu n == G → trace-back path → return
 *      c. Nếu n đã trong Closed Set → skip
 *      d. Đánh dấu n vào Closed Set
 *      e. Với mỗi neighbor m ∈ N_8(n):
 *         - Bỏ qua nếu không traversable hoặc trong Closed Set
 *         - g_new = g(n) + move_cost(n, m) + obstacle_penalty(m)
 *         - Nếu g_new < g_score[m]:
 *             g_score[m]  = g_new
 *             h(m)        = H(m, G)     ← heuristic
 *             f(m)        = g_new + h(m)
 *             parent[m]   = n
 *             Push AStarNode(m, g_new, h(m)) vào Open Set
 *   3. Nếu không tìm được → return empty path
 *
 * CHI PHÍ DI CHUYỂN 8-HƯỚNG:
 *   Hướng ngang/dọc (4 hướng): cost = 1.0
 *   Hướng chéo (4 hướng):      cost = √2 ≈ 1.4142
 *
 * HEURISTIC (xem astar_config.h để biết công thức chi tiết)
 * ============================================================
 */
class AStarPlanner {
 public:
  /**
   * @brief Tìm đường từ start đến goal trên grid
   *
   * @param start_x, start_y  Tọa độ bắt đầu (grid coordinates)
   * @param goal_x,  goal_y   Tọa độ đích (grid coordinates)
   * @param map               AStarMap đã được update
   * @param config            AStarConfig
   * @return                  Danh sách grid cells từ start → goal (không bao gồm start)
   *                          Empty nếu không tìm được đường
   */
  std::vector<std::pair<int, int>> plan(
      int start_x, int start_y,
      int goal_x,  int goal_y,
      const AStarMap&    map,
      const AStarConfig& config) const;

 private:
  /**
   * @brief Tính giá trị heuristic h(n) → G theo loại heuristic đã chọn
   *
   * MANHATTAN : h = |dx| + |dy|
   * EUCLIDEAN : h = √(dx² + dy²)
   * DIAGONAL  : h = max(dx,dy) + (√2-1)·min(dx,dy)
   *
   * @param x, y    Vị trí node hiện tại (grid)
   * @param gx, gy  Vị trí goal (grid)
   * @param config  AStarConfig (chứa heuristic type và weight)
   * @return        h(n) = weight * h_raw(n)
   */
  double computeHeuristic(int x, int y, int gx, int gy, const AStarConfig& config) const;

  /**
   * @brief Trace-back từ goal về start để tạo đường đi
   *
   * @param goal_x, goal_y   Cell đích
   * @param parent           Ma trận lưu cell cha
   * @param start_x, start_y Cell bắt đầu (điểm dừng trace-back)
   * @return                 Đường đi từ start → goal
   */
  std::vector<std::pair<int, int>> tracePath(
      int goal_x, int goal_y,
      const std::vector<int>& parent, int W,
      int start_x, int start_y) const;
};

#endif  // ASTAR_PLANNER_H
