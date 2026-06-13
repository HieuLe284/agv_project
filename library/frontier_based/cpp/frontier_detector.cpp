#include "library/frontier_based/include/frontier_detector.h"

#include <queue>
#include <utility>
#include <rclcpp/rclcpp.hpp>

/*================================================================
 Kiểm tra 1 cell FREE có phải frontier cell không
 (8-connectivity: có ít nhất 1 neighbor UNKNOWN)

 ???????
 ???...?         
 ???...?

 => Nhiều frontier cell liền kề nhau như này sẽ được gom lại thành 1 region
 => Trong đó các dấu . là điểm đã biết, ? là UNKNOWN chưa biết
================================================================
Công thức:
 - x_c = 1/N * Σ(N, i = 1, x_i)
 - y_c = 1/N * Σ(N, i = 1, y_i)
 Trong đó:
 - N: số frontier cells trong region
 - x_i, y_i: tọa độ từng frontier cell
 - x_c, y_c: tọa độ tâm hình học ( centroid)
 Mục đích: Robot thường navigation tới centroid thay vì đi tới từng cell.
 Đây là cách giảm dao động, path noise, local oscillation.
*/
bool FrontierDetector::isFrontierCell(const FrontierMap& map, int x, int y) const {
  if (!map.isFree(x, y)) return false; //Frontier bắt buộc phải nằm trên Free space

  bool has_unknown = false; // Kiểm tra xem cờ unknown có neighbor không

  // Duyệt 8 neighbor
  static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

  for (int d = 0; d < 8; ++d) {
    int nx = x + dx[d];
    int ny = y + dy[d];
    if (map.isValid(nx, ny) && map.isUnknown(nx, ny)) {
      has_unknown = true; //cần thấy 1 unknown là đủ để giảm tính toán quá nhiều
      break;
    }
  }

  if (!has_unknown) return false; // kiểm tra xem có phải unknown hay không

  // Lọc nhiễu: Frontier không được phép nằm sát vật cản.
  // Điều này giúp "cắt đứt" đường viền biên giới (perimeter ring) chạy dọc theo
  // các bức tường do nhiễu tia laser hoặc do lỗi tia không đến đích, 
  // qua đó chỉ tập hợp các frontier cell ở khu vực không gian thực sự mở.
  // Điều kiện bộ lọc:
  // ∀(dx,dy)∈[−2,2]^2: ¬isOccupied(x+dx,y+dy)
  int safe_radius = 3; // Cắt bỏ biên trong khoảng 3 cell (~15cm) tính từ tường
  for (int dy_s = -safe_radius; dy_s <= safe_radius; ++dy_s) { // O(n^2) dùng để quét các vùng lân cận theo trục x và y
    for (int dx_s = -safe_radius; dx_s <= safe_radius; ++dx_s) {
      int nx = x + dx_s;
      int ny = y + dy_s;
      if (map.isValid(nx, ny) && map.isOccupied(nx, ny)) { // Kiểm tra xem có gần obstacle không
        return false;
      }
    }
  }

  return true;
}

// ================================================================
//  Cluster BFS: từ 1 frontier cell khởi đầu, BFS lan rộng ra
//  các frontier cell liền kề để tạo thành 1 FrontierRegion
// ================================================================
FrontierRegion FrontierDetector::buildRegion(
    const FrontierMap& map,
    int start_x, int start_y,
    // std::vector<std::vector<bool>>& frontier_visited) const;
    std::vector<uint8_t>& frontier_visited, int W) const
{
  FrontierRegion region;
  std::queue<std::pair<int, int>> q; // queue BFS

  frontier_visited[start_y * W + start_x] = true; // đánh dấu đã đi qua
  q.push({start_x, start_y}); // bắt đầu BFS

  double sum_wx = 0.0, sum_wy = 0.0; // tích lũy centroid

  // 8-connectivity để cluster các frontier cell liền kề
  static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

  while (!q.empty() && rclcpp::ok()) { 
    auto [cx, cy] = q.front();
    q.pop(); //FIFO

    // Tính tọa độ thế giới của cell này để lấy centroid
    double wx, wy;
    map.gridToWorld(cx, cy, wx, wy);

    region.cells.emplace_back(cx, cy);
    sum_wx += wx;
    sum_wy += wy;

    // Mở rộng cluster sang các frontier cell lân cận
    for (int d = 0; d < 8; ++d) {
      int nx = cx + dx[d];
      int ny = cy + dy[d];

      if (!map.isValid(nx, ny)) continue;         // Bảo vệ boundary
      if(frontier_visited[ny * W + nx]) continue;
      if (!isFrontierCell(map, nx, ny)) continue; // Chỉ expand sang frontier khác

      // Expand cluster   
      frontier_visited[ny * W + nx] = true;
      q.push({nx, ny});
    }
  }

  // Tính centroid của region
  if (!region.cells.empty()) {
    region.centroid_x = sum_wx / region.cells.size();
    region.centroid_y = sum_wy / region.cells.size();
  }

  return region;
}

// ================================================================
//  DETECT — Wave-Front Propagation 
// ================================================================
std::vector<FrontierRegion> FrontierDetector::detect(
    const FrontierMap& map,
    double robot_x, double robot_y,
    int min_frontier_size) const
{
  if (!map.hasMap()) return {}; // kiểm tra xem có map không

  // Lấy kích thước map
  const int W = map.getWidth();
  const int H = map.getHeight();

  // Tối ưu bộ nhớ: dùng 1D vector thay vì 2D vector
  // Ma trận visited cho BFS trên free space
  std::vector<uint8_t> free_visited(H * W, 0);
  // Ma trận visited cho BFS clustering frontier cells
  std::vector<uint8_t> frontier_visited(H * W, 0);
  // Danh sách frontier cell tìm được trong lượt BFS free space
  std::vector<std::pair<int, int>> frontier_cells_found;

  // --- Bước 1: BFS từ vị trí robot trên FREE space ---
  int robot_gx, robot_gy;
  map.worldToGrid(robot_x, robot_y, robot_gx, robot_gy);

  // Nếu vị trí robot không hợp lệ, thử cell gần nhất hợp lệ
  if (!map.isValid(robot_gx, robot_gy)) return {};

  // Nếu cell robot không free (có thể do lỗi map), BFS vẫn chạy được
  // nhưng sẽ không tìm được frontier nào từ đó
  std::queue<std::pair<int, int>> bfs_queue;

  if (map.isFree(robot_gx, robot_gy)) {
    free_visited[robot_gy * W + robot_gx] = true;
    bfs_queue.push({robot_gx, robot_gy});
  } else {
    // Nếu robot nằm trên ô có vật cản/unknown (do nhiễu hoặc giãn nở vật cản),
    // quét tìm ô FREE gần nhất trong bán kính 10 ô để bắt đầu BFS
    bool found_free = false;
    int search_radius = 10;
    double min_dist_sq = 1e9; // Dùng squared distance
    int best_gx = robot_gx;
    int best_gy = robot_gy;

    for (int dy = -search_radius; dy <= search_radius; ++dy) {
      for (int dx = -search_radius; dx <= search_radius; ++dx) {
        int nx = robot_gx + dx;
        int ny = robot_gy + dy;
        if (map.isValid(nx, ny) && map.isFree(nx, ny)) {
          double dist_sq = dx * dx + dy * dy; // d^2 = dx^2 + dy^2 ( KHoảng cách Euclid )
          if (dist_sq < min_dist_sq) { // So sánh d_1 < d_2
            min_dist_sq = dist_sq;
            best_gx = nx;
            best_gy = ny;
            found_free = true;
          }
        }
      }
    }

    if (found_free) {
      free_visited[best_gy * W + best_gx] = true;
      bfs_queue.push({best_gx, best_gy});
    }
  }

  // 4-connectivity cho BFS free space (hiệu quả hơn 8-connectivity)
  static const int dx4[4] = {-1, 1, 0, 0};
  static const int dy4[4] = {0, 0, -1, 1};

  while (!bfs_queue.empty() && rclcpp::ok()) {
    auto [cx, cy] = bfs_queue.front();
    bfs_queue.pop();

    // Kiểm tra cell hiện tại có phải frontier cell không
    if (isFrontierCell(map, cx, cy) && !frontier_visited[cy * W + cx]) {
      frontier_cells_found.push_back({cx, cy});
    }

    // Mở rộng BFS sang các FREE cell lân cận (4-connectivity)
    for (int d = 0; d < 4; ++d) {
      int nx = cx + dx4[d];
      int ny = cy + dy4[d];

      if (!map.isValid(nx, ny)) continue;
      if(free_visited[ny * W + nx]) continue;
      if (!map.isFree(nx, ny)) continue;

      free_visited[ny * W + nx] = true;
      bfs_queue.push({nx, ny});
    }
  }

  // --- Bước 2: Cluster frontier cells thành FrontierRegions ---
  std::vector<FrontierRegion> regions;

  for (const auto& [fx, fy] : frontier_cells_found) {
    if(frontier_visited[fy * W + fx]) continue; // Đã thuộc 1 region rồi

    // Bắt đầu cluster mới từ frontier cell này
    FrontierRegion region = buildRegion(map, fx, fy, frontier_visited, W);

    // --- Bước 3: Lọc region quá nhỏ ---
    if (region.size() >= min_frontier_size) {
      regions.push_back(std::move(region));
    }
  }

  return regions;
}