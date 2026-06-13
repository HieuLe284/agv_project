#include "library/A_star_algorithm/include/astar_map.h"

#include <algorithm>
#include <cmath>

// Nhận map từ ROS2 sang biến local
// grid.info chứa metadata của map
// grid.data chứa dữ liệu của map
void AStarMap::update(const nav_msgs::msg::OccupancyGrid& grid) {
  width_      = static_cast<int>(grid.info.width);
  height_     = static_cast<int>(grid.info.height);
  resolution_ = grid.info.resolution;
  origin_x_   = grid.info.origin.position.x;
  origin_y_   = grid.info.origin.position.y;
  data_       = grid.data;
  has_map_    = true; // Đã có map
}

// Kiểm tra điều kiên j ma trận theo: x ∈ [0,width−1] và y ∈ [0,height−1]
bool AStarMap::isValid(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

// Do bản đồ SLAM thường trả về là 1 matrix 2 chiều. Nhưng để truyền C++ tối ưu hơn
// ta chuyển sang mảng array theo phép nội suy array:
// i = y * width + x 
//Trong đó:
// - i: vị trí tương ứng với mảng 1 chiều data[]
// - x: chỉ số cột (tọa độ x)
// - y: chỉ số hàng (tọa độ y)
// - width: chiều rộng của map
// Kiểm tra cell có giá trị 0 (đi được)
bool AStarMap::isFree(int x, int y) const {
  if (!isValid(x, y)) return false;
  return data_[y * width_ + x] == 0;
}

// Kiểm tra cell có giá trị > threshold (vật cản)
bool AStarMap::isOccupied(int x, int y, int threshold) const {
  if (!isValid(x, y)) return true;  // Ngoài biên → coi là vật cản
  int8_t val = data_[y * width_ + x];
  return val > static_cast<int8_t>(threshold);
}

bool AStarMap::isTraversable(int x, int y, int threshold) const {
  if (!isValid(x, y)) return false;
  int8_t val = data_[y * width_ + x];
  // Cho phép đi qua các ô có độ tin cậy vật cản thấp hơn ngưỡng (threshold)
  // và không được đi vào ô Unknown (-1)
  return val >= 0 && val < threshold;
}

// ================================================================
//  Chi phí gần vật cản (Obstacle Inflation)
//
//  Với cell (x,y), tìm vật cản gần nhất trong bán kính safety_margin.
//  cost = penalty * (1 - d / safety_margin) nếu d < safety_margin
//       = 0.0                               nếu d > safety_margin
// Trong đó:
// - d: khoảng cách Euclidean từ cell (x,y) đến vật cản gần nhất
// - safety_margin: bán kính Inflation
// - penalty: hệ số phạt tối đa
// ================================================================
double AStarMap::computeObstacleCost(int x, int y, int safety_margin, double penalty) const {
  double min_dist = static_cast<double>(safety_margin) + 1.0;

  for (int dy = -safety_margin; dy <= safety_margin; ++dy) {
    for (int dx = -safety_margin; dx <= safety_margin; ++dx) {
      int nx = x + dx;
      int ny = y + dy;
      if (!isValid(nx, ny)) continue;
      if (isOccupied(nx, ny)) {
        double d = std::sqrt(static_cast<double>(dx * dx + dy * dy));
        if (d < min_dist) {
          min_dist = d;
        }
      }
    }
  }

  if (min_dist < static_cast<double>(safety_margin)) {
    // cost tỉ lệ nghịch với khoảng cách: gần vật cản hơn → chi phí cao hơn
    return penalty * (1.0 - min_dist / static_cast<double>(safety_margin));
  }
  return 0.0;
}

// ================================================================
//  Chuyển đổi tọa độ thực → grid
// ================================================================
void AStarMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
}

// ================================================================
//  Chuyển đổi tọa độ grid → thực
// ================================================================
void AStarMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}
