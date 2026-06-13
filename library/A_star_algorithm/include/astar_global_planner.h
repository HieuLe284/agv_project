#ifndef ASTAR_GLOBAL_PLANNER_H
#define ASTAR_GLOBAL_PLANNER_H

#include "library/A_star_algorithm/include/astar_config.h"
#include "library/A_star_algorithm/include/astar_map.h"
#include "library/A_star_algorithm/include/astar_path.h"
#include "library/A_star_algorithm/include/astar_planner.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <utility>

/**
 * @file astar_global_planner.h
 * @brief Coordinator chính cho A* Global Planner
 *
 * ============================================================
 *  VÒNG LẶP GLOBAL PLANNING
 * ============================================================
 *
 *  Mỗi chu kỳ 200ms (cùng timer SLAM):
 *
 *  1. updateMap(grid)  — Cập nhật OccupancyGrid mới nhất
 *
 *  2. setGoal(gx, gy)  — Đặt mục tiêu mới → trigger replan ngay
 *
 *  3. compute(x, y, θ) — Cơ chế 2 lớp:
 *     [a] Replan khi cần:
 *         - Mới nhận goal (has_pending_replan_ = true)
 *         - Đến waypoint cuối nhưng chưa đến goal
 *         - Định kỳ mỗi replan_interval bước (map thay đổi do SLAM)
 *     [b] Bám theo đường:
 *         - Kiểm tra robot đến waypoint hiện tại chưa → advance
 *         - Chỉ cập nhật state nội bộ (current_wp_idx_)
 *
 *  4. getCurrentWaypointX() / getCurrentWaypointY() → waypoint hiện tại
 *     DWA sẽ dùng waypoint này để tự tính goal_angle
 *
 *  CHIẾN LƯỢC REPLAN:
 *    - Replan ngay khi: nhận goal mới / đường cũ không hợp lệ
 *    - Replan định kỳ: mỗi N bước (do map SLAM cập nhật liên tục)
 *    - Nếu A* không tìm được đường: giữ path cũ, log warning
 * ============================================================
 */
class AStarGlobalPlanner {
 public:
  explicit AStarGlobalPlanner(const AStarConfig& config = AStarConfig());

  /**
   * @brief Cập nhật OccupancyGrid mới nhất (gọi từ mapCallback)
   * @param grid OccupancyGrid message từ topic /map
   */
  void updateMap(const nav_msgs::msg::OccupancyGrid& grid);

  /**
   * @brief Đặt goal mới → trigger replan ngay lập tức
   *
   * @param goal_x, goal_y  Tọa độ đích (world frame) [m]
   * @note Khi đặt goal mới theo hệ tọa độ (map/world frame), planner sẽ lập tức chạy A* để tìm đường mới.
   */
  void setGoal(double goal_x, double goal_y);

  /**
   * @brief Tính toán điều hướng theo A* path (chỉ cập nhật state nội bộ)
   *
   * Gọi mỗi 200ms từ slam_global_planner():
   *   - Kiểm tra cần replan không → chạy A* nếu cần
   *   - Advance waypoint nếu đến đủ gần
   *   - Kiểm tra đến goal chưa
   *
   * @param robot_x, robot_y   Vị trí robot (world frame) [m]
   * @param robot_theta        Góc hướng robot [rad]
   * @param step_count         Số bước hiện tại (dùng cho periodic replan)
   * 
   * @note Hàm này KHÔNG trả về goal_angle. DWA sẽ tự tính goal_angle từ waypoint.
   */
  void compute(double robot_x, double robot_y, double robot_theta,
               int step_count);

  /**
   * @brief Trả về tọa độ X của waypoint hiện tại (world frame)
   * @return x [m], hoặc 0.0 nếu không có path
   */
  double getCurrentWaypointX() const;

  /**
   * @brief Trả về tọa độ Y của waypoint hiện tại (world frame)
   * @return y [m], hoặc 0.0 nếu không có path
   */
  double getCurrentWaypointY() const;

  // ================================================================
  //  Getters / Status
  // ================================================================
  // Kiểm tra xem có goal đang active không
  bool hasGoal()        const { return has_goal_; } 
  // Kiểm tra xem có path hiện tại không
  bool hasPath()        const { return current_path_.valid && !current_path_.empty(); }
  // Kiểm tra xem đã đến goal chưa
  bool isGoalReached()  const { return goal_reached_; }

  const AStarPath&    getCurrentPath()   const { return current_path_; }

  // Reset lại toàn bộ trạng thái
  void reset();

 private:
  /**
   * @brief Chạy A*, convert kết quả sang world coordinates, simplify path
   *
   * @param robot_x, robot_y   Vị trí robot hiện tại [m]
   * @return true nếu tìm được đường
   * @note ép tọa đồ thực (m) về dạng lưới (grid) để tính toán A*.
   */
  bool runAstar(double robot_x, double robot_y);

  /**
   * @brief Tính góc từ (fx, fy) đến (tx, ty) trong robot frame và khoảng cách 
   * trong Euclid 2D
   * @return angle_error ∈ [-π, π]
   */
  // Khoảng cách Euclidean 2D
  static double dist(double x1, double y1, double x2, double y2);
  
  AStarConfig  config_;
  AStarMap     map_;
  AStarPlanner planner_;

  bool   has_goal_{false};        // Có goal đang active không
  bool   goal_reached_{false};    // Đã đến goal chưa
  bool   has_pending_replan_{false};  // Cần replan ngay không

  // Tọa độ goal [m]
  double goal_x_{0.0};   
  double goal_y_{0.0};

  AStarPath current_path_;       // Đường đi hiện tại
  int       current_wp_idx_{0};  // Index waypoint đang bám theo
};

#endif  // ASTAR_GLOBAL_PLANNER_H
