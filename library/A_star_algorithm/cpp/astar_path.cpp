#include "library/A_star_algorithm/include/astar_path.h"

#include <cmath>

// ================================================================
//  Tính tổng chiều dài đường đi [m]
// Tính chiều dài quỹ đạo rời rạc (Discrete Path Integration) sử dụng:
// Phép cộng dồn khoảng cách Euclidean giữa các waypoint liên tiếp trong chuỗi rời rạc:
// L = ∑_{i=1}^{K−1}∥w_i − w_{i−1}∥_2
// Theo hệ tọa độ cartesian (x,y) trong không gian 2 chiều
// L = ∑_{i=1}^{K−1} sqrt((x_i − x_{i−1})^2 + (y_i − y_{i−1})^2)
// Trong đó:
//  - w_i = (x_i, y_i): Điểm waypoint của quỹ đạo
// ================================================================
double PathSimplifier::computeLength(const AStarPath& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.waypoints.size(); ++i) {
    double dx = path.waypoints[i].first  - path.waypoints[i-1].first;
    double dy = path.waypoints[i].second - path.waypoints[i-1].second;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

// ================================================================
//  Đơn giản hóa đường đi — Collinearity Check
//
// Step 1: Collinearity check với tolerance nhỏ hơn
//  Với 3 điểm A=(x_A,y_A), B=(x_B,y_B), C=(x_C,y_C):
//    vector(AB) = (x_B - x_A, y_B - y_A)
//    vector(AC) = (x_C - x_A, y_C - y_A)
//    cross = |vector(AB) x vector(AC)| = det(vector(AB), vector(AC))
//          = (x_B - x_A)*(y_C - y_A) - (y_B - y_A)*(x_C - x_A)
//  Khoảng cách từ điểm đến đường thẳng:
//    dist_B_to_AC = |cross| / vector(|AC|)
//
//  Nếu dist_B_to_AC < tolerance → B thẳng hàng với AC → loại B
// Step 2: 
// ================================================================
AStarPath PathSimplifier::simplify(const AStarPath& raw_path, double tolerance, double max_spacing) {
  // Step1: Collinearity check với tolerance nhỏ hơn
  AStarPath collinearity;
  collinearity.valid = raw_path.valid; // sao chép cờ hợp lệ

  // Điều kiện biên: Nếu đường đi có 2 điểm hoặc ít hơn thì không cần đơn giản hóa
  const auto& pts = raw_path.waypoints;
  if (pts.size() <= 2) {
    collinearity.waypoints = pts;
    collinearity.total_length = computeLength(collinearity);
    collinearity.waypoint_count = static_cast<int>(collinearity.waypoints.size());
    return collinearity;
  }

  collinearity.waypoints.push_back(pts.front()); // Luôn giữ điểm đầu tiên

  // Khai báo vector
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    const auto& A = collinearity.waypoints.back();  // Điểm cuối đã giữ
    const auto& B = pts[i];                   // Điểm hiện tại
    const auto& C = pts[i + 1];               // Điểm tiếp theo

    // Vector AC
    double ac_x = C.first  - A.first;
    double ac_y = C.second - A.second;
    double ac_len = std::sqrt(ac_x * ac_x + ac_y * ac_y);

    if (ac_len < 1e-9) {
      // A và C trùng nhau → bỏ qua B
      continue;
    }

    // Tích vô hướng của vector AB và AC
    double ab_x = B.first  - A.first;
    double ab_y = B.second - A.second;
    double cross = std::abs(ab_x * ac_y - ab_y * ac_x);

    // Khoảng cách từ B đến đường thẳng AC
    double dist_b_to_ac = cross / ac_len;

    // Nếu B quá gần đường AC → loại bỏ B (thẳng hàng)
    if (dist_b_to_ac >= tolerance) {
      collinearity.waypoints.push_back(B);
    }
  }

  // Luôn giữ điểm cuối cùng (goal)
  collinearity.waypoints.push_back(pts.back());

  // Step 2: Chèn waypoint mới nếu khoảng cách giữa 2 điểm > max_spacing
  AStarPath spacing;
  spacing.valid = collinearity.valid;

  if(collinearity.waypoints.empty()) return spacing;

  // Luôn giữ điểm đầu tiên
  spacing.waypoints.push_back(collinearity.waypoints.front());

  for (size_t i = 1; i < collinearity.waypoints.size(); ++i) {
    const auto& prev = collinearity.waypoints[i - 1];
    const auto& curr = collinearity.waypoints[i];

    double dx = curr.first - prev.first;
    double dy = curr.second - prev.second;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist > max_spacing && max_spacing > 0.01) {
      // Chèn thêm waypoint trung gian
      int num_segments = static_cast<int>(std::ceil(dist / max_spacing));
      for (int j = 1; j < num_segments; ++j) {
        double ratio = static_cast<double>(j) / num_segments;
        double wx = prev.first + ratio * dx;
        double wy = prev.second + ratio * dy;
        spacing.waypoints.emplace_back(wx, wy);
      }
    }
    spacing.waypoints.push_back(curr);
  }

  spacing.total_length   = computeLength(spacing);
  spacing.waypoint_count = static_cast<int>(spacing.waypoints.size());
  return spacing;
}
