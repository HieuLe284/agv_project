#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

/**
 * @file astar_node.h
 * @brief Node struct dùng trong thuật toán A*
 *
 * ============================================================
 *  TOÁN HỌC — CẤU TRÚC NODE TRONG A*
 * ============================================================
 *
 *  Mỗi node n biểu diễn một cell (x, y) trong OccupancyGrid:
 *
 *    g(n): Chi phí thực tế từ Start đến n
 *          g(start) = 0
 *          g(neighbor) = g(current) + move_cost + obstacle_penalty
 *
 *    h(n): Heuristic ước lượng từ n đến Goal
 *          (xem AStarConfig để biết các loại heuristic)
 *
 *    f(n) = g(n) + h(n)
 *          Open Set được sắp xếp theo f tăng dần (min-heap)
 *          → Node có f nhỏ nhất được xét trước
 *
 *  Pointer về parent: (parent_x, parent_y)
 *    Dùng để trace-back đường đi sau khi A* tìm được Goal
 *    parent = (-1, -1) khi là Start node
 * ============================================================
 */

/**
 * @struct AStarNode
 * @brief Một node trong thuật toán A*
 */
struct AStarNode {
  int x{0};   ///< Tọa độ cột trong OccupancyGrid
  int y{0};   ///< Tọa độ hàng trong OccupancyGrid

  double f{0.0};  ///< f(n) = g(n) + h(n)  — tổng chi phí ước tính
  double g{0.0};  ///< g(n): chi phí thực tế từ Start → n
  double h{0.0};  ///< h(n): heuristic ước lượng từ n → Goal

  int parent_x{-1};  ///< Tọa độ x của node cha (-1 nếu là Start)
  int parent_y{-1};  ///< Tọa độ y của node cha (-1 nếu là Start)

  AStarNode() = default;

  AStarNode(int x_, int y_, double g_, double h_, int px, int py)
    : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent_x(px), parent_y(py) {}

  /**
   * @brief Toán tử so sánh: priority_queue dùng để tạo min-heap theo f
   *        Node có f nhỏ hơn có ưu tiên cao hơn (được xét trước)
   */
  bool operator>(const AStarNode& other) const { return f > other.f; }
};

#endif  // ASTAR_NODE_H
