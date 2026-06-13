/**
 * @file frontier_selector.cpp
 *
 * ============================================================
 *  TOÁN HỌC — LỰA CHỌN FRONTIER (FRONTIER SELECTION)
 * ============================================================
 *
 * 1. KHOẢNG CÁCH EUCLIDEAN (NORMALIZED)
 *    D_i = sqrt( (X_{Gi} - X_R)^2 + (Y_{Gi} - Y_R)^2 )
 *    D_i_norm = D_i / max(D_j)  → [0, 1]
 * Trong đó:
 * - D_i: khoảng cách từ robot đến centroid của frontier region i
 * - X_{Gi}, Y_{Gi}: tọa độ centroid của frontier region i
 * - X_R, Y_R: tọa độ robot
 * - max(D_j): khoảng cách lớn nhất từ robot đến centroid của tất cả các frontier region
 * 
 * 2. KÍCH THƯỚC FRONTIER (NORMALIZED)
 *    S_i_norm = S_i / max(S_j)  → [0, 1]
 * Trong đó:
 * - S_i: kích thước của frontier region i
 * - S_i_norm: kích thước của frontier region i, đã được normalize về [0, 1]
 * - max(S_j): kích thước lớn nhất của tất cả các frontier region
 * 
 * 3. HÀM CHI PHÍ (COST FUNCTION — ĐÃ NORMALIZE)
 *    C_i = W_dist * D_i_norm + W_size * (1 - S_i_norm)
 * Trong đó:
 * - C_i: cost của frontier region i
 * - D_i_norm: khoảng cách từ robot đến centroid của frontier region i, đã được normalize về [0, 1]
 * - S_i_norm: kích thước của frontier region i, đã được normalize về [0, 1]
 * - W_dist: trọng số khoảng cách, ưu tiên frontier gần
 * - W_size: trọng số kích thước, ưu tiên frontier lớn
 *
 *    → Frontier gần (D nhỏ) và lớn (S lớn) được ưu tiên.
 *    → W_dist và W_size giờ có ý nghĩa thực: cùng phạm vi [0,1].
 *
 * 4. LỰA CHỌN TỐI ƯU
 *    R* = argmin_{R_i} (C_i)
 * Trong đó:
 * - R*: frontier region được chọn
 * - argmin: hàm tìm giá trị nhỏ nhất
 * - R_i: frontier region thứ i
 * - C_i: cost của frontier region thứ i
 * => Frontier gần => D_i nhỏ => Cost nhỏ => Được ưu tiên
 * => Frontier lớn => S_i lớn => 1 - S_i nhỏ => Cost nhỏ => Được ưu tiên
 * 
 * Rằng buộc lọc thêm
 * - D_i > 0.1m: loại bỏ frontier quá gần (nhiễu)
 * - D_i < max_goal_dist: loại bỏ frontier quá xa
 * ============================================================
 */

#include "library/frontier_based/include/frontier_selector.h"

#include <cmath>
#include <limits>

const FrontierRegion* FrontierSelector::select(
    const std::vector<FrontierRegion>& regions,
    double robot_x, double robot_y,
    const FrontierConfig& config) const
{
  if (regions.empty()) return nullptr;

  // --- Bước 1: Lọc các frontier hợp lệ và tính khoảng cách ---
  struct Candidate {
    const FrontierRegion* region;
    double dist;
  };
  std::vector<Candidate> candidates;
  // Cấp phát bộ nhớ từ đầu giúp tránh hành vi tự Re-allocation nhằm tăng tốc O(1)
  candidates.reserve(regions.size()); 

  double max_dist = 0.0;
  double max_size = 0.0;

  //Bộ lọc tần số (Spatial Low-High Pass Filter)
  for (const auto& region : regions) {
    double dx = region.centroid_x - robot_x;
    double dy = region.centroid_y - robot_y;
    double dist = std::sqrt(dx * dx + dy * dy); //D_i​=sqrt((x_Gi​−x_R​)^2+(y_Gi​−y_R​)^2)

    // Lọc frontier quá xa hoặc quá gần (nhiễu)
    if (dist > config.max_goal_dist) continue;  // D_i < D_max
    // Lọc frontier cận dưới (Lower-bound cutoff)
    if (dist < 0.1) continue;                   // D_i > 0.1(m)

    candidates.push_back({&region, dist});
    if (dist > max_dist) max_dist = dist;
    double sz = static_cast<double>(region.size());
    if (sz > max_size) max_size = sz;
  }

  if (candidates.empty()) return nullptr;

  // --- Bước 2: Tính normalized cost và chọn frontier tốt nhất ---
  // Tránh chia cho 0 nếu chỉ có 1 frontier
  if (max_dist < 1e-6) max_dist = 1.0;
  if (max_size < 1.0) max_size = 1.0;

  const FrontierRegion* best = nullptr;
  double min_cost = std::numeric_limits<double>::max();

  for (const auto& cand : candidates) {
    // Normalize về [0, 1]
    double dist_norm = cand.dist / max_dist; // D_i​ = D_j/max_j(D_j)​​
    double size_norm = static_cast<double>(cand.region->size()) / max_size; // S_i​ = S_j/max_j(S_j)​​

    // cost thấp = frontier GẦN + LỚN
    double cost = config.w_dist * dist_norm + config.w_size * (1.0 - size_norm); // C_i​ = W_dist * D_i ​+ W_size​ * ( 1 − S_i​)

    if (cost < min_cost) { //R* = argmin_{R_i} (C_i)
      min_cost = cost;
      best = cand.region;
    }
    // Nếu cost function không tìm được best, fallback về F0
    if (best == nullptr) {
      best = candidates[0].region;
    }
  }

  return best;  // nullptr nếu không có frontier hợp lệ nào
}
