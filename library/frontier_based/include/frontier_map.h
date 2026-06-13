#ifndef FRONTIER_MAP_H
#define FRONTIER_MAP_H

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstdint>
#include <vector>

/*
ROS OccupancyGrid => FrontierMap => FrontierDetector
( nghĩa là FrontierDectector bên trên không làm việc trực tiếp với ROS message 
 mà làm thông qua abstraction layer FrontierMap )
*/
/**
 * @struct FrontierCell
 * @brief Một ô (cell) trong OccupancyGrid, biểu diễn bằng tọa độ lưới (grid) với không gian index rời rạc Z^2
 */
struct FrontierCell {
  int x;  // Cột trong grid
  int y;  // Hàng trong grid

  FrontierCell(int x_, int y_) : x(x_), y(y_) {}
};

/**
 * @class FrontierMap
 * @brief Wrapper bao quanh nav_msgs::msg::OccupancyGrid
 *
 * Cung cấp các hàm tiện ích để:
 *  - Kiểm tra trạng thái ô (FREE / UNKNOWN / OCCUPIED)
 *  - Chuyển đổi tọa độ thế giới ↔ tọa độ lưới
 *
 * M: Z^2 -> State = {FREE, UNKNOWN, OCCUPIED}
 * ROS ánh xạ xác suất P(m_x,y)
 * Quy ước giá trị OccupancyGrid:
 *   0   → FREE (không có vật cản)
 *  -1   → UNKNOWN (chưa được khám phá)
 *  1–100 → OCCUPIED (có vật cản, thường 100 = chắc chắn)
 * 
 */
class FrontierMap {
 public:
  FrontierMap() = default;

  /**
   * @brief Cập nhật map từ OccupancyGrid message mới nhất
   * @param grid OccupancyGrid từ ROS topic /map
   * @note đảm nhận việc đồng bộ trạng thái world state vào mô hình bộ nhớ nội dung của agent
   */
  void update(const nav_msgs::msg::OccupancyGrid& grid);

  // Kiểm tra ô có nằm trong giới hạn grid không
  bool isValid(int x, int y) const;

  // Kiểm tra ô có FREE không (value == 0)
  bool isFree(int x, int y) const;

  // Kiểm tra ô có UNKNOWN không (value == -1)
  bool isUnknown(int x, int y) const;

  // Kiểm tra ô có OCCUPIED không (value > 50)
  bool isOccupied(int x, int y) const;

  // worldToGrid và gridToWorld là hai hàm để chuyển đổi tọa độ theo phép biến đổi không gian
  /**
   * @brief Chuyển tọa độ thế giới (m) → tọa độ lưới (cells)
   * @param wx, wy  Tọa độ thế giới [m]
   * @param gx, gy  Tọa độ lưới [cells] (output)
   * @param g_x = (w_x - o_x) / r
   * @param g_y = (w_y - o_y) / r
   * @note Trong đó: o_x, o_y là tọa độ origin của map
   *           r là resolution của map (m/cell)
   */
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;

  /**
   * @brief Chuyển tọa độ lưới → tọa độ thế giới (tâm của cell) [m]
   * @param gx, gy  Tọa độ lưới [cells]
   * @param wx, wy  Tọa độ thế giới [m] (output)
   * @param w_x = ( g_x + 0.5 ) * r + o_x 
   * @param w_y = ( g_y + 0.5 ) * r + o_y 
   * @note Trong đó: + 0.5 mục đích để đảm bảo lấy tâm của cell thay vì góc dưới trái
   */
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  // Lấy kích thước map dưới dạng w x h
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  
  bool hasMap() const { return has_map_; }

  // Mảng không gian 2D được lưu dưới dạng 1D
 private:
  std::vector<int8_t> data_;  // Dữ liệu grid phẳng [height * width]
  int width_{0};              // Chiều rộng grid [cells]
  int height_{0};             // Chiều cao grid [cells]
  double resolution_{0.05};   // Kích thước 1 cell [m/cell]
  double origin_x_{0.0};     // Gốc tọa độ thế giới của góc (0,0) của grid [m]
  double origin_y_{0.0};
  bool has_map_{false};
};

#endif  // FRONTIER_MAP_H
