#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include "library/frontier_based/include/frontier_map.h"

#include <cstdint>
#include <vector>

/**
 * ============================================================
 * (FRONTIER DETECTION)
 * ============================================================
 *
 * 1. ĐỊNH NGHĨA FRONTIER CELL
 *    Một ô c = (x, y) là Frontier Cell khi:
 *      P(x, y) == 0  (Free space)
 *      Và ∃ n ∈ N_8(x,y) sao cho P(n) == -1 (Unknown)
 *    Trong đó P(x, y) là xác suất có vật cản, N_8 là 8 ô xung quanh.
 *
 * 2. GOM CỤM (CLUSTERING)
 *    Áp dụng BFS (Breadth-First Search) để tìm các Frontier Cell liên thông.
 *    Tập hợp R_i = {c_1, c_2, ..., c_k} tạo thành một Frontier Region.
 *    Kích thước: S_i = |R_i|. Điều kiện: S_i >= min_frontier_size.
 *
 * 3. TRỌNG TÂM (CENTROID)
 *    G_i = (X_{Gi}, Y_{Gi}) với:
 *      X_{Gi} = (1 / S_i) * Σ(x_j)
 *      Y_{Gi} = (1 / S_i) * Σ(y_j)
 *
 * ============================================================
 */
 
/*
Robot di chuyển trong 1 bản đồ occupancy grid, trong đó mỗi cell ở map là:
 - FREE ( đã biết )
 - OCCUPIED ( vật cản )
 - UNKNOWN ( chưa biết )
Thuật toán này sẽ giúp robot:
 Tìm frontier cells => gom thành frontier regions => chọn region phù hợp
 => Điều hướng robot tới đó => Update map => Lặp lại.
*/

/**
 * @struct FrontierRegion
 * @brief Một vùng frontier: tập hợp các cell liên kết tạo thành ranh giới
 *        giữa không gian đã biết (FREE) và chưa biết (UNKNOWN)
 *
 * Thuật toán (Yamauchi 1997):
 *   Frontier cell = cell FREE có ít nhất 1 neighbor UNKNOWN (8-connectivity)
 *   Frontier region = cluster các frontier cell liền kề
 */
struct FrontierRegion {
  std::vector<FrontierCell> cells;  // Danh sách các frontier cell
  double centroid_x{0.0};           // Tọa độ X của trọng tâm (world frame) [m]
  double centroid_y{0.0};           // Tọa độ Y của trọng tâm (world frame) [m]

  int size() const { return static_cast<int>(cells.size()); } // Trả về số lượng frontier cell
};

/**
 * @class FrontierDetector
 * @brief Phát hiện và phân cụm (cluster) frontier regions trong OccupancyGrid
 *
 * ================================================================
 *  THUẬT TOÁN — WAVE-FRONT PROPAGATION (WFD) (Yamauchi 1997)
 * ================================================================
 *
 * Input:  FrontierMap (OccupancyGrid đã parse)
 *         Robot position (robot_x, robot_y) — điểm bắt đầu BFS
 *
 * Bước 1 — BFS trên không gian FREE từ vị trí robot:
 *   Queue ← {cell của robot}
 *   Với mỗi FREE cell c được dequeue:
 *     - Nếu c có ≥1 neighbor UNKNOWN → c là frontier cell
 *     - Enqueue các FREE neighbor chưa thăm
 *
 * Bước 2 — Clustering frontier cells:
 *   Với mỗi frontier cell chưa được cluster:
 *     - BFS nội bộ trên các frontier cell liền kề (8-connectivity)
 *     - Tạo thành 1 FrontierRegion
 *
 * Bước 3 — Lọc:
 *   Loại bỏ FrontierRegion có size() < min_frontier_size
 *
 * Output: vector<FrontierRegion>
 * 
 * Độ phức tạp: O(M × N)
 * Trong đó: M × N là kích thước của occupancy grid
 * Giải thích: Thuật toán này tương đương với việc tìm kiếm theo chiều rộng trên bản đồ không có trọng số, đảm bảo tìm
 * các frontier reachable (có thể đến được từ vị trí robot hiện tại) - loại bỏ hoàn toàn các frontier phía sau bức tường.
 * Nếu cell tại vị trí robot không phải FREE do inflation_radius của costmap, thuật toán tìm kiếm cell FREE gần nhất trong
 * bán kinh 10 cells bằng khoảng cách Euclidean: d^2 = dx^2 + dy^2 với argmin(dx,dy) (dx^2 + dy^2)
 */
class FrontierDetector {
 public:
  /**
   * @brief Phát hiện tất cả frontier regions trong map
   *
   * @param map               FrontierMap đã được update
   * @param robot_x           Vị trí robot X (world frame) [m]
   * @param robot_y           Vị trí robot Y (world frame) [m]
   * @param min_frontier_size Số cell tối thiểu của một region hợp lệ
   * @return                  Danh sách FrontierRegion tìm được
   */
  std::vector<FrontierRegion> detect( 
    const FrontierMap& map,
    double robot_x,
    double robot_y,
    int min_frontier_size) const;

 private:
  /**
   * @brief Kiểm tra 1 cell FREE có phải là frontier cell không
   *        (có ≥1 neighbor UNKNOWN trong 8-connectivity)
   */
  bool isFrontierCell(const FrontierMap& map, int x, int y) const;

  /**
   * @brief Cluster BFS: từ 1 frontier cell chưa thăm, tìm tất cả
   *        frontier cell liền kề và tạo thành 1 FrontierRegion
   *
   * @param map             FrontierMap
   * @param start_x, start_y  Cell khởi đầu của cluster
   * @param frontier_visited  Mảng đánh dấu frontier cell đã cluster
   * @return FrontierRegion mới
   * Công thức: Centroid theo trục x và y
   * x_c = (1/N) * Σ(N, i = 1, x_i)
   * y_c = (1/N) * Σ(N, i = 1, y_i)
   * Trong đó: 
   * - N: số frontier cells trong region
   * - x_i, y_i: tọa độ từng frontier cell
   * - x_c, y_c: tọa độ tâm hình học (centroid)
   * Mục đích: Robot thường navigation tới centroid thay vì đi tới từng cell.
   * Đây là cách giảm dao động, path noise, local oscillation.
   */
  FrontierRegion buildRegion(
      const FrontierMap& map,
      int start_x,
      int start_y,
      std::vector<uint8_t>& frontier_visited, int W) const;
};

#endif  // FRONTIER_DETECTOR_H
