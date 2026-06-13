#ifndef DWA_SCORING_H
#define DWA_SCORING_H

#include "library/DWA/include/dwa_config.h"
#include "library/DWA/include/dwa_state.h"

#include <utility>
#include <vector>

/**
 * @class TrajectoryScorer
 * @brief Tính điểm hàm mục tiêu G(v,w) và tính khoảng cách va chạm theo exact circular curvature
 */
class TrajectoryScorer {
 public:
  /**
   * @brief Tính tổng điểm G(v,w) = α·heading + β·clearance + γ·velocity
   *
   * @param state      Trạng thái ban đầu
   * @param v          Vận tốc tịnh tiến thử nghiệm [m/s]
   * @param w          Vận tốc góc thử nghiệm [rad/s]
   * @param dist       Khoảng cách an toàn (tính từ hàm computeDistance)
   * @param goal_angle Góc hướng mục tiêu θ_goal trong robot frame [rad]
   * @param config     Cấu hình DWA
   * @return           Điểm G tổng hợp
   */
  double score(
    const DWAState& state,
    double v,
    double w,
    double dist,
    double goal_angle,
    const DWAConfig& config) const;

  /**
   * @brief Tính khoảng cách an toàn (dist) đến vật cản gần nhất trên quỹ đạo tròn (hoặc thẳng)
   * Theo bài báo: quỹ đạo là đường tròn bán kính r = v/w.
   * Tính chính xác độ dài cung tròn đến khi mép robot (r_robot) chạm vật cản.
   * 
   * @param v          Vận tốc tịnh tiến [m/s]
   * @param w          Vận tốc góc [rad/s]
   * @param obstacles  Danh sách vật cản (x, y) [robot frame]
   * @param config     Cấu hình DWA (chứa robot_radius)
   * @return           Khoảng cách di chuyển dọc theo quỹ đạo trước khi va chạm [m]
   */
  double computeDistance(
    double v,
    double w,
    const std::vector<std::pair<double, double>>& obstacles,
    const DWAConfig& config) const;

 private:
  // heading(v, w) = 1 - |θ_pred - θ_goal| / π (θ_pred dự đoán sau 1 khoảng thời gian khi phanh)
  double computeHeading(
    const DWAState& state,
    double v,
    double w,
    double goal_angle,
    const DWAConfig& config) const;

  // clearance(v, w) = dist (đã chuẩn hóa)
  double computeClearanceScore(
    double dist,
    const DWAConfig& config) const;

  // velocity(v, w) = |v| / v_max
  double computeVelocityScore(double v, double v_max) const;
};

#endif  // DWA_SCORING_H
