#ifndef FRONTIER_SELECTOR_H
#define FRONTIER_SELECTOR_H

#include "library/frontier_based/include/frontier_config.h"
#include "library/frontier_based/include/frontier_detector.h"

#include <vector>

/**
 * @class FrontierSelector
 * @brief Chọn frontier region tốt nhất để robot điều hướng đến
 *
 * ================================================================
 *  HÀM CHI PHÍ (Cost Function — Normalized)
 * ================================================================
 *
 *  dist_norm_i  = dist(robot, centroid_i) / max_dist
 *  size_norm_i  = size_i / max_size
 *  cost_i = w_dist * dist_norm_i + w_size * (1 - size_norm_i)
 *
 *  Trong đó:
 *    dist(robot, centroid_i) — khoảng cách Euclidean [m]
 *    size_i                  — số cell trong frontier region i
 *    max_dist, max_size      — giá trị lớn nhất trong tập ứng viên
 *    w_dist                  — trọng số khoảng cách (ưu tiên frontier gần)
 *    w_size                  — trọng số kích thước (ưu tiên frontier lớn)
 * @note Dựa vào bài toán quy hoạch tuyến tính nội suy (Linear Programming Heuristic)
 * nhiều mục tiêu (Multi-Objective Optimization). Vì robot phải đối mặt với một tập
 * hợp các Frontier R = {R_1, R_2,..., R_n}, mỗi frontier R_i có hai thuộc tính:
 *    - Khoảng cách đến robot: d_i ( càng gần càng tốn ít thời gian di chuyển)
 *    - Kích thước: s_i ( càng lớn càng có nhiều không gian để di chuyển)
 * Vậy nên robot cần chọn frontier R_i sao cho thỏa mãn cả hai mục tiêu:
 *    - Tối thiểu hóa khoảng cách: min(d_i)
 *    - Tối đa hóa kích thước: max(s_i)
 */
class FrontierSelector {
 public:
  /**
   * @brief Chọn frontier tốt nhất từ danh sách regions
   *
   * @param regions   Danh sách FrontierRegion từ FrontierDetector
   * @param robot_x   Vị trí robot X (world frame) [m]
   * @param robot_y   Vị trí robot Y (world frame) [m]
   * @param config    FrontierConfig (trọng số, max_dist...)
   * @return          Pointer đến FrontierRegion tốt nhất,
   *                  hoặc nullptr nếu không có frontier nào hợp lệ
   */
  const FrontierRegion* select(
      const std::vector<FrontierRegion>& regions,
      double robot_x,
      double robot_y,
      const FrontierConfig& config) const;
};

#endif  // FRONTIER_SELECTOR_H
