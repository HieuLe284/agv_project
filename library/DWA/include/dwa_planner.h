#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <rclcpp/rclcpp.hpp>

#include "library/DWA/include/dwa_config.h"
#include "library/DWA/include/dwa_scoring.h"
#include "library/DWA/include/dwa_state.h"
#include "library/DWA/include/dwa_window.h"

#include <utility>
#include <vector>

/**
 * @class DWAPlanner
 * @brief Class điều phối chính của Dynamic Window Approach theo paper (1997)
 *
 * ================================================================
 *  THUẬT TOÁN DWA — QUY TRÌNH MỖI CHU KỲ ĐIỀU KHIỂN
 * ================================================================
 *
 * Input:  s_cur = (x, y, θ, v, w)  — trạng thái robot hiện tại
 *         scan_ranges               — dữ liệu LiDAR thô
 *         robot_x, robot_y, robot_theta — pose robot trong world frame
 *         path                      — Full A* path (vector of waypoints in world frame)
 *
 * DWA tự động tìm waypoint lookahead từ path và tính goal_angle nội bộ.
 *
 * Bước 1 — Dynamic Window:
 *   Vd: cửa sổ vận tốc bị giới hạn bởi gia tốc tối đa trong 1 dt
 *   Vs: giới hạn vật lý [v_min, v_max] × [-w_max, w_max]
 *   Tìm V_s ∩ V_d
 *
 * Bước 2 — Chuyển đổi LiDAR → vật cản
 *   (x_obs, y_obs) = (d · cos(angle), d · sin(angle))  [robot frame]
 *
 * Bước 3 — Grid sampling:
 *   Duyệt qua các mẫu (v_i, w_j) trong V_s ∩ V_d
 *
 * Bước 4 — Admissible Velocities (Va):
 *   Tính khoảng cách an toàn dist(v_i, w_j)
 *   Chỉ giữ lại nếu v_i <= sqrt(2 * dist * v_dot_b) và w_j <= sqrt(2 * dist * w_dot_b)
 *
 *
 * Bước 5 — Objective Function:
 *   G(v, w) = α·heading(v,w) + β·clearance(v,w) + γ·velocity(v,w)
 *   Trong đó heading = 1 - |θ_pred - θ_goal| / π
 *
 * Output: (v*, w*) tối ưu
 */
class DWAPlanner {
 public:
  explicit DWAPlanner(const DWAConfig& config = DWAConfig());

  /**
   * @brief Tính vận tốc điều khiển tối ưu (v*, w*) theo DWA
   *
   * @param state            Trạng thái hiện tại s = (x,y,θ,v,w) trong robot frame
   * @param scan_ranges      Mảng khoảng cách LiDAR [m] (float từ ROS msg)
   * @param angle_min        Góc bắt đầu của scan [rad]
   * @param angle_increment  Bước góc giữa các tia liên tiếp [rad]
   * @param yaw_offset       Góc xoay của LiDAR so với gốc robot [rad]
   * @param robot_x          Vị trí X của robot trong world frame [m]
   * @param robot_y          Vị trí Y của robot trong world frame [m]
   * @param robot_theta      Góc hướng robot trong world frame [rad]
   * @param path             Full A* path: vector of (x,y) waypoints [world frame, m]
   *                         DWA sẽ tự tìm waypoint lookahead phù hợp và tính goal_angle
   * @return std::pair<double, double>  (v*, w*) đơn vị [m/s], [rad/s]
   */
  std::pair<double, double> computeVelocity(
    const DWAState& state,
    const std::vector<float>& scan_ranges,
    double angle_min,
    double angle_increment,
    double yaw_offset,
    double robot_x,
    double robot_y,
    double robot_theta,
    const std::vector<std::pair<double, double>>& path);

  const DWAConfig& getConfig() const { return config_; }

 private:
  DWAConfig config_;
  DynamicWindow window_;
  TrajectoryScorer scorer_;

  /**
   * @brief Chuyển dữ liệu LiDAR sang danh sách vật cản trong robot frame
   */
  std::vector<std::pair<double, double>> scanToObstacles(
    const std::vector<float>& ranges,
    double angle_min,
    double angle_increment,
    double yaw_offset) const;

  /**
   * @brief Tìm waypoint lookahead từ path và tính goal_angle trong robot frame
   *
   * Duyệt từ waypoint đầu tiên trong path, tìm waypoint đầu tiên cách robot
   * một khoảng >= lookahead_dist. Nếu không có, lấy waypoint cuối cùng.
   * Tính góc từ robot đến waypoint đó trong robot frame.
   *
   * @param robot_x, robot_y, robot_theta  Pose robot (world frame)
   * @param path                           Danh sách waypoints từ A*
   * @param[out] goal_angle                Góc mục tiêu trong robot frame [rad]
   * @return true nếu tìm được waypoint hợp lệ, false nếu path rỗng
   */
  bool findLookaheadGoal(
    double robot_x, double robot_y, double robot_theta,
    const std::vector<std::pair<double, double>>& path,
    double& goal_angle) const;
};

#endif  // DWA_PLANNER_H
