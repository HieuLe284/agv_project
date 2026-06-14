#include "library/DWA/include/dwa_scoring.h"
#include "library/common/math_utils.h"

#include <algorithm>
#include <cmath>

// Công thức Objective Function của DWA
// G(v,ω) = α⋅heading(v,ω) + β⋅clearance(v,ω) + γ⋅velocity(v,ω)
// Trong đó:
// - heading(v,ω): Hướng của robot so với hướng mục tiêu
// - clearance(v,ω): Khoảng cách an toàn đến vật cản gần nhất
// - velocity(v,ω): Vận tốc tịnh tiến của robot
// - α, β, γ: Trọng số của từng thành phần
double TrajectoryScorer::score(
  const DWAState& state,
  double v,
  double w,
  double dist,
  double goal_angle,
  const DWAConfig& config) const
{
  const double heading_score   = computeHeading(state, v, w, goal_angle, config);
  const double clearance_score = computeClearanceScore(dist, config);
  const double velocity_score  = computeVelocityScore(v, config.v_max);

  return config.alpha * heading_score +
         config.beta  * clearance_score +
         config.gamma * velocity_score;
}

double TrajectoryScorer::computeDistance(
  double v,
  double w,
  const std::vector<std::pair<double, double>>& obstacles,
  const DWAConfig& config) const
{
  double min_dist = config.sensor_max_range;
  const double R = config.robot_radius;

  // 0. Kiểm tra va chạm tức thời: Nếu có bất kỳ vật cản nào nằm trong bán kính robot ngay lúc này
  for (const auto& obs : obstacles) {
    if (std::sqrt(obs.first * obs.first + obs.second * obs.second) < R) {
      return 0.0;
    }
  }

  if (std::abs(w) < 1e-4) {
    // ================================================================
    // 1. Quỹ đạo thẳng (w ≈ 0)
    // Robot đi thẳng dọc theo trục X.
    // Công thức khoảng cách tới va chạm với vật cản O(x, y):
    // dist = x - sqrt(R^2 - y^2) (với điều kiện |y| <= R)
    // ================================================================
    for (const auto& obs : obstacles) {
      // Chỉ xét vật cản nằm trong dải chiều rộng của robot
      if (std::abs(obs.second) <= R) {
        // Giao điểm của tia (trục X) và đường tròn vật cản bán kính R
        double d = obs.first - std::sqrt(R * R - obs.second * obs.second);
        
        // Nếu d < 0, nghĩa là robot đang đè lên vật cản hoặc đã vượt qua điểm va chạm đầu tiên
        // Nếu obs.first > -R (vật cản không nằm hoàn toàn phía sau robot), ta coi là va chạm
        if (obs.first > -R && d < min_dist) {
          min_dist = std::max(0.0, d);
        }
      }
    }
  } else {
    // ================================================================
    // 2. Quỹ đạo cong (w != 0)
    // Cắt hình học giữa quỹ đạo tròn của xe và vùng biên an toàn vật cản.
    // Bán kính quỹ đạo: r = v / w
    // Tâm quỹ đạo (C): (0, r)
    // Khoảng cách từ tâm C đến vật cản O(x, y): D = sqrt(x^2 + (y - r)^2)
    // Va chạm xảy ra khi D nằm trong dải quét của xe: |r| - R <= D <= |r| + R
    // ================================================================
    const double r = v / w; // Bán kính quỹ đạo
    const double cx = 0.0;
    const double cy = r;
    const double start_angle = std::atan2(-r, 0.0); // Hướng từ tâm đến gốc (0,0)

    for (const auto& obs : obstacles) {
      double dx = obs.first - cx;
      double dy = obs.second - cy;
      double D = std::sqrt(dx * dx + dy * dy);

      // Nếu khoảng cách từ tâm quỹ đạo đến vật cản nằm ngoài dải bán kính quét của robot
      if (D > std::abs(r) + R || D < std::abs(r) - R) {
        continue;
      }

      // ================================================================
      // Theo định lý cosin để tìm góc lệch delta khi va chạm
      // Tam giác: Tâm quỹ đạo (C), Vật cản (O), Điểm va chạm (P) trên quỹ đạo
      // cos(delta) = (r^2 + D^2 - R^2) / (2 * |r| * D)
      // ================================================================
      if (std::abs(r) < 1e-4) {
        continue; // Bán kính quá nhỏ (gần như xoay tại chỗ), không tạo thêm va chạm cho robot hình tròn
      }
      double cos_delta = (r * r + D * D - R * R) / (2.0 * std::abs(r) * D);
      // Đảm bảo không bị lỗi do sai số float
      cos_delta = std::max(-1.0, std::min(1.0, cos_delta)); 
      double delta = std::acos(cos_delta);

      double obs_angle = std::atan2(dy, dx);

      // Hai góc có thể xảy ra va chạm
      double a1 = normalizeAngle(obs_angle - delta - start_angle);
      double a2 = normalizeAngle(obs_angle + delta - start_angle);

      double travel_angle = 1e9;

      if (w > 0) {
        // Đi sang trái, góc tăng dần (ngược chiều kim đồng hồ)
        if (a1 < 0) a1 += 2.0 * M_PI;
        if (a2 < 0) a2 += 2.0 * M_PI;
        travel_angle = std::min(a1, a2);
      } else {
        // Đi sang phải, góc giảm dần (cùng chiều kim đồng hồ)
        if (a1 > 0) a1 -= 2.0 * M_PI;
        if (a2 > 0) a2 -= 2.0 * M_PI;
        travel_angle = std::min(std::abs(a1), std::abs(a2));
      }

      // Chỉ xét va chạm ở phía trước robot (travel_angle < PI)
      if (travel_angle < M_PI) {
        double d = travel_angle * std::abs(r);
        if (d < min_dist) {
          min_dist = d;
        }
      }
    }
  }

  return min_dist;
}

double TrajectoryScorer::computeHeading(
  const DWAState& state,
  double v,
  double w,
  double goal_angle,
  const DWAConfig& config) const
{
  // ================================================================
  // Heading Score:
  // Dự đoán góc của robot sau khoảng thời gian dt + thời gian phanh.
  // Thời gian phanh: t_break = |w| / w_dot_b
  // Trong đó:
  //   - w_dot_b: Gia tốc góc giảm tốc cực đại
  // Dự đoán góc drift bằng Forward Euler:
  //   θ_pred = w * Δt + 0.5 * w * t_break
  // Trong đó:
  //   - w * Δt: Góc quay hiện tại 
  //   - 0.5 * w * t_break: quãng quay khi phanh
  // Heading Score có công thức là:
  //   H(v,w) = 1 - ( |θ_pred - θ_goal| / π )
  // Nếu: H -> 1: robot đúng hướng về phía mục tiêu
  // Nếu: H -> 0: robot lệch hướng về phía mục tiêu
  // ================================================================
  
  // Thời gian phanh
  double t_break = std::abs(w) / config.w_dot_b;
  
  // Tổng góc thay đổi = góc thay đổi trong dt + góc thay đổi khi phanh
  // Khi phanh, vận tốc góc giảm dần đều về 0, nên quãng đường góc = 0.5 * w * t_break
  double theta_pred = w * config.dt + 0.5 * w * t_break; // θ_pred

  double diff = normalizeAngle(theta_pred - goal_angle);

  // Góc lệch càng nhỏ thì heading score càng gần 1.0
  double heading = 1.0 - (std::abs(diff) / M_PI) * (std::abs(diff) / M_PI); // Heading Score
  
  return heading;
}

double TrajectoryScorer::computeClearanceScore(
  double dist,
  const DWAConfig& config) const
{
  // ================================================================
  // Clearance Score:
  // score = dist / D_normalize
  // Trước: tường 2m → score = min(1.0, 2.0/1.0) = 1.0 (không phạt gì!)
  //        → robot không "thấy" tường cho đến khi cách 1m → tránh quá muộn.
  // Sau:   tường 2m → score = min(1.0, 2.0/3.0) = 0.67 (có phạt!)
  //        tường 1m → score = 0.33, tường 0.5m → score = 0.17
  //        → Robot bắt đầu né tường từ 3m → tránh mượt mà, không bị kẹt.
  // ================================================================
  return std::min(1.0, dist / config.D_normalize);
}


double TrajectoryScorer::computeVelocityScore(double v, double v_max) const {
  // ================================================================
  // Velocity Score:
  // score = |v| / v_max
  // ================================================================
  if (v < 0.001) return 0.01; // Điểm cực thấp cho việc đứng im nhưng không phải âm (để fallback tốt hơn)
  return std::abs(v) / v_max;
}
