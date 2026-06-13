#ifndef ASTAR_MAP_H
#define ASTAR_MAP_H

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstdint>
#include <vector>

/**
 * @file astar_map.h
 * @brief Wrapper bao quanh nav_msgs::msg::OccupancyGrid dùng cho A*
 *
 * ============================================================
 *  TOÁN HỌC — BẢN ĐỒ LƯỚI VÀ CHUYỂN ĐỔI TỌA ĐỘ
 * ============================================================
 *
 * 1. QUY ƯỚC GIÁ TRỊ CELL
 *    value = 0        → FREE (đi được)
 *    value = -1       → UNKNOWN (chưa biết, coi là không đi được trong A*)
 *    value = 1..100   → OCCUPIED theo xác suất
 *    value > threshold → Coi là vật cản (blocked)
 *
 * 2. CHUYỂN ĐỔI TỌA ĐỘ THỰC ↔ LƯỚI
 *    World → Grid:
 *      gx = floor((wx - origin_x) / resolution)
 *      gy = floor((wy - origin_y) / resolution)
 *
 *    Grid → World (tâm cell):
 *      wx = origin_x + (gx + 0.5) * resolution
 *      wy = origin_y + (gy + 0.5) * resolution
 *
 * 3. CHI PHÍ GẦN VẬT CẢN (Obstacle Inflation Cost)
 *    Với mỗi cell (x,y), tìm khoảng cách tối thiểu d đến vật cản gần nhất
 *    trong bán kính safety_margin cells:
 *
 *      d = min distance to nearest obstacle in [safety_margin] radius
 *      cost = obstacle_penalty * (1 - d / safety_margin)  nếu d < safety_margin
 *           = 0.0                                          nếu không có vật cản gần
 *
 *    Giúp A* tìm đường đi an toàn, cách xa vật cản.
 * @note lib ccos vai trò là grid map nội bộ để đóng gói wrapper bản đồ ros2 cục bộ
 * cung cấp các hàm chuyển đổi tọa độ thực - lưới, kiểm tra cell, tính chi phí inflation
 * ============================================================
 */
class AStarMap {
 public:
  AStarMap() = default;

  /**
   * @brief Cập nhật map từ OccupancyGrid message
   * @param grid OccupancyGrid từ topic /map
   */
  void update(const nav_msgs::msg::OccupancyGrid& grid);

  // ================================================================
  //  Truy vấn cell
  // ================================================================
  // Kiểm tra nằm trong biên của mảng hay không
  bool isValid(int x, int y) const;
  // Kiểm tra cell có giá trị 0 (đi được)
  bool isFree(int x, int y) const;       // value == 0
  // Kiểm tra cell có giá trị > threshold (vật cản)
  bool isOccupied(int x, int y, int threshold = 50) const;  //< value > threshold

  /**
   * @brief Kiểm tra cell có thể đi được không (Free và không bị inflation)
   *        UNKNOWN coi là không đi được trong A*
   */
  bool isTraversable(int x, int y, int threshold = 50) const;

  /**
   * @brief Tính chi phí inflation do gần vật cản
   *
   * cost = obstacle_penalty * (1 - dist_to_obstacle / safety_margin)
   *        Trả về 0.0 nếu không có vật cản trong vùng safety_margin
   *
   * @param x, y          Tọa độ cell cần kiểm tra
   * @param safety_margin Bán kính tìm kiếm vật cản [cells]
   * @param penalty       Hệ số phạt tối đa
   * @note Mục đích không cho AGV đi quá gần vật cản dù điểm sát tường có isFree = true
   */
  double computeObstacleCost(int x, int y, int safety_margin, double penalty) const;

  // ================================================================
  //  Chuyển đổi tọa độ
  // ================================================================
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  // ================================================================
  //  Getters
  // ================================================================
  int    getWidth()      const { return width_; }     // Lấy chiều rộng của map 
  int    getHeight()     const { return height_; }    // Lấy chiều cao của map 
  bool   hasMap()        const { return has_map_; }   // Kiểm tra map có tồn tại không

 private:
  std::vector<int8_t> data_;
  int    width_{0};
  int    height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  bool   has_map_{false};
};

#endif  // ASTAR_MAP_H
