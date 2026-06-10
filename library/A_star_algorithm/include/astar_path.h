#ifndef ASTAR_PATH_H
#define ASTAR_PATH_H

#include <utility>
#include <vector>

/**
 * @file astar_path.h
 * @brief Biểu diễn đường đi sau khi A* tìm được + đơn giản hóa đường
 *
 * ============================================================
 *  TOÁN HỌC — ĐƠN GIẢN HÓA ĐƯỜNG ĐI (PATH SIMPLIFICATION)
 * ============================================================
 *
 *  Mục tiêu: Loại bỏ các điểm trung gian không cần thiết trên đường
 *  thẳng, giảm số lượng waypoint mà robot phải theo dõi.
 *
 *  Thuật toán (Collinearity Check):
 *    Với 3 điểm liên tiếp: A = (x1,y1), B = (x2,y2), C = (x3,y3)
 *
 *    Cross Product 2D (diện tích hình bình hành ABC):
 *      cross = (B.x - A.x)*(C.y - A.y) - (B.y - A.y)*(C.x - A.x)
 *
 *    Nếu |cross| / |AC| < simplify_tolerance:
 *      → A, B, C gần thẳng hàng → loại bỏ B
 *
 *    Kết quả: Đường đi từ N điểm giảm xuống còn K điểm (K ≤ N),
 *    trong đó chỉ giữ lại các điểm góc cua quan trọng.
 * @note Do A* chạy trên bản đồ grid nên quỹ đạo sinh ra có dạng zig-zag
 * nên cần phải lọc vavf loại bỏ các điểm thừa (collinear points) để tạo
 * thành một đường gãy khúc tối giản nhất. Nhằm giúp DWA chạy mượt và tiết
 * kiệm tài nguyên tính toán hơn.
 * ============================================================
 */

/**
 * @struct AStarPath
 * @brief Đường đi từ A* trong tọa độ thực tế (world frame) [m]
 */
struct AStarPath {
  std::vector<std::pair<double, double>> waypoints;  // Danh sách waypoints (x,y) [m]
  bool   valid{false};         // Cờ đánh dấu quỹ đạo xem khả thi không
  double total_length{0.0};    // Tổng chiều dài đường [m]
  int    waypoint_count{0};    // Số waypoints

  bool empty() const { return waypoints.empty(); } // Kiểm tra xem vector tọa độ có rỗng không
};

/**
 * @class PathSimplifier
 * @brief Đơn giản hóa đường đi bằng Collinearity Check
 */
class PathSimplifier {
 public:
  /**
   * @brief Loại bỏ các waypoint thẳng hàng không cần thiết
   *
   * Kiểm tra diện tích tam giác (cross product) giữa 3 điểm liên tiếp.
   * Nếu |cross|/|AC| < tolerance → điểm giữa B được loại bỏ.
   *
   * @param raw_path    Đường đi thô từ A*
   * @param tolerance   Ngưỡng đơn giản hóa [m]
   * @param max_spacing Giới hạn cho waypoint mới
   * @return          
   */
  static AStarPath simplify(const AStarPath& raw_path, double tolerance, double max_spacing);

  /**
   * @brief Tính tổng chiều dài đường đi [m]
   * @param path Đường đi cần tính
   * @return Tổng chiều dài [m]
   */
  static double computeLength(const AStarPath& path);
};

#endif  // ASTAR_PATH_H
