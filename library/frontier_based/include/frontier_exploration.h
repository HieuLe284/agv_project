#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include "library/frontier_based/include/frontier_config.h"
#include "library/frontier_based/include/frontier_detector.h"
#include "library/frontier_based/include/frontier_map.h"
#include "library/frontier_based/include/frontier_selector.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <utility>
#include <vector>
// Thuật toán này không tự tính toán gì cả mà gọi các module khác như Map, Detector, Selector
/**
 * @class FrontierExploration
 * @brief Coordinator chính cho thuật toán Frontier-Based Exploration
 *
 * ================================================================
 *  VÒNG LẶP THÁM HIỂM (từ bài báo Yamauchi 1997)
 * ================================================================
 *
 *  Mỗi chu kỳ điều khiển:
 *
 *  1. update(grid)   — Cập nhật OccupancyGrid mới nhất từ SLAM
 *
 *  2. compute(x,y,θ) — Tìm frontier goal mới nếu cần:
 *       a. Nếu chưa có goal, hoặc được A* báo đã đến goal cũ (signalGoalReached):
 *            → detect() tất cả frontier regions
 *            → select() frontier tốt nhất
 *            → Nếu không có frontier: done_ = true
 *       b. Nếu đang có goal: giữ nguyên, chờ A* hoàn thành
 *
 *  3. isDone() → true khi không còn frontier (map đã đầy đủ)
 *  4. reset()  → khởi động lại để thám hiểm lại
 *
 *  Lưu ý: FrontierExploration CHỈ cung cấp goal (x,y).
 *  Việc lập path, bám waypoint, tránh vật cản được xử lý bởi A* + DWA.
 */
class FrontierExploration {
 public:
  // Khai báo explicit cho constructor để ngăn C++ tự động ép kiểu, ép ngầm định tham số cấu hình
  explicit FrontierExploration(const FrontierConfig& config = FrontierConfig());

  /**
   * @brief Cập nhật OccupancyGrid mới nhất (gọi từ mapCallback)
   * @param grid OccupancyGrid message từ topic /map
   * @note Hàm này để ROS đưa bảo đồ mới từ topic/map vào thuật toán mỗi khi SLAM cập nhật bản đồ
   */
  void update(const nav_msgs::msg::OccupancyGrid& grid);

  /**
   * @brief Tìm frontier goal mới nếu cần (cập nhật state nội bộ)
   *
   * @param robot_x, robot_y   Vị trí robot (world frame) [m] — từ SLAM graph
   * @param robot_theta        Góc hướng robot [rad] — từ SLAM graph
   *
   * Frontier sẽ tìm goal mới khi:
   *   - Chưa có goal (has_goal_ == false)
   *   - Được A* báo đã đến goal cũ (goal_reached_by_astar_ == true)
   *   - Timeout stuck > 30s
   */
  void compute(double robot_x, double robot_y, double robot_theta);

  /**
   * @brief Được A* gọi khi đã đến frontier goal
   *
   * Khi A* phát hiện robot đã đến goal, nó gọi hàm này
   * để frontier biết và tìm goal mới.
   */
  void signalGoalReached();

  /**
   * @brief Kiểm tra thám hiểm đã hoàn thành chưa (không còn frontier nào)
   */
  bool isDone() const { return done_; }

  double getGoalX() const { return goal_x_; }
  double getGoalY() const { return goal_y_; }

  /**
   * @brief Kiểm tra đang có frontier goal đang active không
   */
  bool hasGoal() const { return has_goal_; }

  /**
   * @brief Reset lại trạng thái để thám hiểm lại từ đầu
   */
  void reset();

  /**
   * @brief Trả về danh sách frontier regions từ lần detect cuối (debug)
   */
  const std::vector<FrontierRegion>& getLastRegions() const {
    return last_regions_;
  }

  // Các composed objects 
 private:
  FrontierConfig    config_;
  FrontierMap       map_;
  FrontierDetector  detector_;
  FrontierSelector  selector_;

  bool   has_goal_{false};  // Đang có frontier goal active
  bool   done_{false};      // Map đã được thám hiểm hoàn toàn
  double goal_x_{0.0};      // Tọa độ X của frontier đang nhắm đến [m]
  double goal_y_{0.0};      // Tọa độ Y của frontier đang nhắm đến [m]

  // Cờ do A* báo: đã đến frontier goal thành công
  bool   goal_reached_by_astar_{false};

  // Timeout: thời điểm set goal gần nhất (dùng để detect stuck)
  std::chrono::steady_clock::time_point goal_set_time_{std::chrono::steady_clock::now()};

  // Frontier regions từ lần detect gần nhất
  std::vector<FrontierRegion> last_regions_;  
};

#endif  // FRONTIER_EXPLORATION_H
