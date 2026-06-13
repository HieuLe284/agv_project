#include "library/frontier_based/include/frontier_map.h"

void FrontierMap::update(const nav_msgs::msg::OccupancyGrid& grid) {
  width_      = static_cast<int>(grid.info.width);
  height_     = static_cast<int>(grid.info.height);
  resolution_ = grid.info.resolution;
  origin_x_   = grid.info.origin.position.x;
  origin_y_   = grid.info.origin.position.y;

  // Copy dữ liệu grid (int8_t: 0=free, -1=unknown, 1-100=occupied)
  data_ = grid.data;
  has_map_ = true;
}

// Thuật toán biên logic: 
// Ω={(x,y) ∈ Z^2 ∣ 0 ≤ x < w, 0 ≤ y < h}
bool FrontierMap::isValid(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

// Công thức: index = y * width + x
bool FrontierMap::isFree(int x, int y) const {
  if (!isValid(x, y)) return false;
  int8_t val = data_[y * width_ + x];
  // ROS convention: 0 = definitely free, 1-49 = likely free, 50-100 = occupied, -1 = unknown
  // MapBuilder outputs probability*100 (e.g., ~41 for cells hit by LOG_ODDS_FREE=-0.35)
  // Accept any value in [0, 50) as free
  return val >= 0 && val < 50;
}

bool FrontierMap::isUnknown(int x, int y) const {
  if (!isValid(x, y)) return false;
  return data_[y * width_ + x] == -1;
}

bool FrontierMap::isOccupied(int x, int y) const {
  if (!isValid(x, y)) return false;
  int8_t val = data_[y * width_ + x];
  return val >= 50;  // Ngưỡng 50/100 — >=50 là occupied (theo ROS convention)
}

//phép biến đổi không gian
void FrontierMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  // Chuyển tọa độ thế giới → tọa độ lưới (floor)
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
}

void FrontierMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  // Tâm của cell
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}
