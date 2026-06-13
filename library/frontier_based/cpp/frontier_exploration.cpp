/**
 * @file frontier_exploration.cpp
 *
 * ============================================================
 *  TOÁN HỌC — VÒNG LẶP THÁM HIỂM TỔNG QUÁT (Yamauchi 1997)
 * ============================================================
 *
 * 1. CẬP NHẬT TRẠNG THÁI (UPDATE)
 *    Nhận bản đồ M_t (OccupancyGrid) và pose x_t = (X_R, Y_R, θ_R).
 *
 * 2. QUÁ TRÌNH TÍNH TOÁN (COMPUTE)
 *    a) Nếu chưa có mục tiêu hoặc được A* báo đã đến goal:
 *       - Tìm tập R = Detect(M_t)
 *       - Chọn G_{t+1} = Select(R, x_t)
 *       - Nếu R = ∅ -> Hoàn thành thám hiểm.
 *    b) Nếu đang có mục tiêu:
 *       - Giữ nguyên, chờ A* báo goal reached.
 *
 * 3. ĐẦU RA (OUTPUT)
 *    Frontier trả về goal (x, y) cho A*.
 *    A* lập path, DWA bám path và tự tính goal_angle.
 * ============================================================
 */

#include "library/frontier_based/include/frontier_exploration.h"

#include <cmath>

// ================================================================
//  Constructor
// ================================================================
FrontierExploration::FrontierExploration(const FrontierConfig& config) : config_(config) {}

// ================================================================
//  update() — Cập nhật OccupancyGrid mới nhất từ mapCallback
// ================================================================
void FrontierExploration::update(const nav_msgs::msg::OccupancyGrid& grid) {
  map_.update(grid);
}

// ================================================================
//  reset() — Khởi động lại thám hiểm
// ================================================================
void FrontierExploration::reset() {
  has_goal_ = false;
  done_     = false;
  goal_x_   = 0.0;
  goal_y_   = 0.0;
  goal_reached_by_astar_ = false;
  goal_set_time_ = std::chrono::steady_clock::now();
  last_regions_.clear();
}

// ================================================================
//  signalGoalReached() — Được A* gọi khi đã đến frontier goal
// ================================================================
void FrontierExploration::signalGoalReached() {
  goal_reached_by_astar_ = true;
}

// ================================================================
//  compute() — Tìm frontier goal mới nếu cần (chỉ cập nhật state nội bộ)
// ================================================================
void FrontierExploration::compute(
    double robot_x, double robot_y, double robot_theta)
{
  // Chưa có map → chưa làm gì được
  if (!map_.hasMap()) {
    return;
  }

  // Đã khám phá xong
  if (done_) {
    return;
  }

  // --- Kiểm tra xem có cần tìm goal mới không ---
  bool need_new_goal = !has_goal_;

  // Nếu A* báo đã đến goal → tìm goal mới
  if (has_goal_ && goal_reached_by_astar_) {
    need_new_goal = true;
    goal_reached_by_astar_ = false;
    RCLCPP_INFO(rclcpp::get_logger("FrontierExploration"),
      "[Frontier] A* báo đã đến goal (%.2f, %.2f). Tìm frontier mới.",
      goal_x_, goal_y_);
  }

  // Timeout: nếu stuck quá lâu ở cùng 1 goal (> 30 giây) → abandon và tìm goal mới
  if (has_goal_) {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - goal_set_time_).count();
    if (elapsed > 30.0) {
      RCLCPP_WARN(rclcpp::get_logger("FrontierExploration"),
        "[Frontier] Stuck at goal (%.2f, %.2f) for %.0fs. Abandoning and finding new frontier.",
        goal_x_, goal_y_, elapsed);
      need_new_goal = true;
    }
  }

  // --- Tìm frontier goal mới nếu cần ---
  if (need_new_goal) {
    // Detect tất cả frontier regions từ vị trí robot bằng thuật toán Wave-Front
    last_regions_ = detector_.detect(map_, robot_x, robot_y, config_.min_frontier_size);

    // Nếu phát hiện thấy có frontier mới, reset trạng thái done_ về false
    if (!last_regions_.empty()) {
      done_ = false;
    }

    // Chọn frontier tốt nhất
    const FrontierRegion* best = selector_.select(last_regions_, robot_x, robot_y, config_);

    // Nếu không tìm được frontier tốt có thể danh sách empty hoặc không có frontier quá bé, xa, nhiễu,...
    // => loại hết và dừng thám hiểm
    if (best == nullptr) {
      // R = ∅ => Không còn frontier => Thám hiểm hoàn thành
      if(last_regions_.empty()) done_ = true;
      has_goal_ = false;
      return;
    }

    // Xử lý Projection (Ép tâm về vùng an toàn): Vì trung tâm hình học của 1 hình vòng cung cong chữ C...
    // đôi khi lại vào bên trong hình chữ C (vật cản). Nên ta sẽ tìm cell tự do gần nhất với tâm hình học
    // Đặt goal mới: Kéo centroid về free cell gần nhất nếu nó nằm trên ô UNKNOWN/Occupied
    double final_goal_x = best->centroid_x; // Lấy tâm hình học tọa độ x
    double final_goal_y = best->centroid_y; // Lấy tâm hình học tọa độ y
    
    int gx, gy;
    map_.worldToGrid(final_goal_x, final_goal_y, gx, gy); // Chuyển tọa độ world sang tọa độ grid
    
    if (!map_.isFree(gx, gy)) { // Nếu ô đó không phải là ô trống
      bool found_free = false;
      int search_radius = 20; // Tăng bán kính tìm kiếm với lưới là 20x20cell xung quanh tâm
      double min_dist_sq = 1e9; // Khởi tạo khoảng cách nhỏ nhất là vô cùng
      int best_free_gx = gx, best_free_gy = gy; // Khởi tạo tọa độ cell tự do tốt nhất là chính tâm
        
      //Quét ma trận vuông 41x41cell xung quanh tâm ( quét từ -20 đến 20 )
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
          int nx = gx + dx;
          int ny = gy + dy;
          if (map_.isFree(nx, ny)) { // Nếu ô đó là ô trống
            double dist_sq = dx*dx + dy*dy; // Tính khoảng cách Euclid từ tâm đến ô hiện tại
            if (dist_sq < min_dist_sq) { // Nếu khoảng cách hiện tại nhỏ hơn khoảng cách nhỏ nhất
              min_dist_sq = dist_sq; // Cập nhật khoảng cách nhỏ nhất
              best_free_gx = nx; // Cập nhật tọa độ cell tự do tốt nhất
              best_free_gy = ny; // Cập nhật tọa độ cell tự do tốt nhất
              found_free = true;
            }
          }
        }
      }
        
      // Nếu tìm được 1 cell FREE gần tâm nhất. Thì cập nhật lại tọa độ tâm mới
      if (found_free) {
        map_.gridToWorld(best_free_gx, best_free_gy, final_goal_x, final_goal_y);
      }
    }

    // Ghi tọa độ mới, reset lại timeout
    goal_x_   = final_goal_x;
    goal_y_   = final_goal_y;
    has_goal_ = true;
    goal_set_time_ = std::chrono::steady_clock::now(); // Reset timeout timer
  }

  // Frontier chỉ cập nhật state nội bộ (goal_x_, goal_y_)
  // Không trả về (v, w) — A* và DWA xử lý phần còn lại
}
