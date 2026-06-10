#ifndef ASTAR_CONFIG_H
#define ASTAR_CONFIG_H

/**
 * @file astar_config.h
 * @brief Tham số cấu hình cho thuật toán A* Global Planner
 *
 * ============================================================
 *  TOÁN HỌC — CHI PHÍ VÀ HEURISTIC CỦA A*
 * ============================================================
 *
 * 1. TỔNG CHI PHÍ NODE
 *    f(n) = g(n) + h(n)
 *    Trong đó:
 *      g(n): Chi phí thực tế từ Start đến n
 *      h(n): Heuristic ước lượng từ n đến Goal
 *
 * 2. CHI PHÍ DI CHUYỂN g(n)
 *    Ngang/dọc (4 hướng):   g(next) = g(cur) + 1.0
 *    Chéo (4 hướng chéo):   g(next) = g(cur) + √2 ≈ 1.4142
 *    Phạt gần vật cản:      g(next) += obstacle_penalty
 *
 * 3. CÁC LOẠI HEURISTIC h(n)
 *    Đặt: dx = |goal_x - x|,  dy = |goal_y - y|
 *
 *    MANHATTAN  (4-connectivity):
 *      h(n) = dx + dy
 *
 *    EUCLIDEAN  (8-connectivity) — DEFAULT:
 *      h(n) = √(dx² + dy²)
 *      Admissible: luôn ≤ chi phí thực → đảm bảo tối ưu
 *
 *    DIAGONAL / OCTILE  (8-connectivity):
 *      h(n) = (dx + dy) + (√2 - 2) * min(dx, dy)
 *           = max(dx, dy) + (√2 - 1) * min(dx, dy)
 *      Chính xác hơn Euclidean với lưới 8-hướng
 *
 * 4. WEIGHTED A* (tốc độ vs. tối ưu)
 *    h_w(n) = heuristic_weight * h(n)
 *    w = 1.0 → A* chuẩn, tối ưu tuyệt đối
 *    w > 1.0 → Tìm đường nhanh hơn, có thể không tối ưu (ε-optimal)
 * ============================================================
 */

/**
 * @enum HeuristicType
 * @brief Loại heuristic sử dụng trong A*
 */
enum class HeuristicType {
  MANHATTAN,  ///< h = |dx| + |dy| — 4-connectivity
  EUCLIDEAN,  ///< h = √(dx²+dy²) — DEFAULT, admissible với 8-connectivity
  DIAGONAL    ///< h = max(dx,dy) + (√2-1)*min(dx,dy) — tối ưu cho 8-connectivity
};

/**
 * @struct AStarConfig
 * @brief Toàn bộ tham số cấu hình cho A* Global Planner
 */
struct AStarConfig {
  // ================================================================
  //  Heuristic
  // ================================================================
  HeuristicType heuristic{HeuristicType::EUCLIDEAN};  ///< Loại heuristic
  double heuristic_weight{1.0};  ///< w ≥ 1.0 (1.0 = optimal, >1.0 = faster)

  // ================================================================
  //  Obstacle Handling
  // ================================================================
  int    obstacle_threshold{50};   ///< Giá trị grid > ngưỡng = vật cản (0-100)
  // [FIX Bug#5]: safety_margin = 2 cells (0.10m) — vừa đủ để A* tìm đường qua khe hẹp
  // mà vẫn đảm bảo robot không cắt góc quá gần tường.
  // Kết hợp với DWA robot_radius = 0.15m → tổng clearance = 0.25m từ tường.
  double obstacle_penalty{2.0};   ///< Chi phí thêm khi đi qua cell gần vật cản
  int    safety_margin{4};        ///< Số cell an toàn xung quanh vật cản [cells] (2 cells = 0.10m)

  // ================================================================
  //  Navigation
  // ================================================================
  // [FIX]: Giảm waypoint_tolerance từ 0.45m xuống 0.20m.
  // Trước: 0.45m → robot nhảy waypoint sớm, đi tắt qua gần tường → va chạm.
  // Sau:   0.20m → robot bám sát từng waypoint của A*, không cắt góc.
  double waypoint_tolerance{0.20};  ///< Khoảng cách để chuyển sang waypoint tiếp theo [m]
  double goal_tolerance{0.30};      ///< Khoảng cách để coi đã đến goal [m]

  // ================================================================
  //  Planning
  // ================================================================
  int max_iterations{100000};  ///< Giới hạn số vòng lặp A* (tránh treo)
  // [FIX Bug#4]: Giảm replan_interval từ 50 xuống 20 (10s → 4s).
  // Khi robot bị kẹt, A* sẽ tìm lại đường sau 4s thay vì 10s.
  int replan_interval{20};     ///< Số bước SLAM giữa 2 lần replan định kỳ
                               ///< (20 bước × 200ms = 4 giây)

  // ================================================================
  //  Path Simplification
  // ================================================================
  double simplify_tolerance{0.01};  // Ngưỡng bỏ điểm thẳng hàng [m]
  double max_waypoint_spacing{0.18}; // Chèn waypoint mới nếu khoảng cách > 0.18 (dày hơn cho góc cua)

};

#endif  // ASTAR_CONFIG_H
