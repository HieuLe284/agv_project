/**
 * @file astar_global_planner.cpp
 *
 * ============================================================
 *  TOÁN HỌC — GLOBAL PLANNER COORDINATOR
 * ============================================================
 *
 * 1. REPLAN LOGIC (2 chiến lược)
 *    [A] Replan ngay (Triggered):
 *        - Nhận goal mới → has_pending_replan_ = true
 *        - Path hiện tại rỗng / không hợp lệ
 *    [B] Replan định kỳ (Periodic):
 *        - step_count % replan_interval == 0
 *        - Dùng khi map SLAM cập nhật liên tục (obstacle mới xuất hiện)
 *
 * 2. THEO DÕI WAYPOINT
 *    Tại mỗi bước, kiểm tra khoảng cách robot đến waypoint[current_wp_idx_]:
 *      dist = √((robot_x - wp.x)² + (robot_y - wp.y)²)
 *      Nếu dist < waypoint_tolerance → current_wp_idx_++
 *      Nếu current_wp_idx_ == total_waypoints → goal_reached_ = true
 *
 * 3. ĐẦU RA
 *    Chỉ cập nhật state nội bộ (current_path_, current_wp_idx_, goal_reached_).
 *    DWA sẽ lấy waypoint (x,y) qua getCurrentWaypointX/Y() và tự tính goal_angle.
 * ============================================================
 */

#include "library/A_star_algorithm/include/astar_global_planner.h"
#include "library/common/math_utils.h"

#include <cmath>
#include <algorithm>

// ================================================================
//  Constructor
// ================================================================
AStarGlobalPlanner::AStarGlobalPlanner(const AStarConfig& config)
  : config_(config) {}

// ================================================================
//  updateMap — cập nhật OccupancyGrid mới nhất
// ================================================================
void AStarGlobalPlanner::updateMap(const nav_msgs::msg::OccupancyGrid& grid) {
  map_.update(grid);
}

// ================================================================
//  setGoal — đặt goal mới, trigger replan ngay
// ================================================================
void AStarGlobalPlanner::setGoal(double goal_x, double goal_y) {
  goal_x_            = goal_x;        // Cập nhật điểm x
  goal_y_            = goal_y;        // Cập nhật điểm y
  has_goal_          = true;          // Cờ đã đến điểm goal
  goal_reached_      = false;         // Cờ đang trên đường đi
  has_pending_replan_ = true;         // Cờ báo hiệu cần replan ngay
  current_wp_idx_    = 0;             // Reset ở điểm cực tiểu của chuỗi điểm mới (0)
  current_path_      = AStarPath{};   // Xóa mảng rác cũ
}

// ================================================================
//  reset
// ================================================================
void AStarGlobalPlanner::reset() {
  has_goal_          = false;         // Cờ báo không có goal
  goal_reached_      = false;
  has_pending_replan_ = false;        // Cờ báo không cần replan
  current_path_      = AStarPath{};
  current_wp_idx_    = 0;
}

// ================================================================
//  runAstar — chạy A* và cập nhật current_path_
// ================================================================
bool AStarGlobalPlanner::runAstar(double robot_x, double robot_y) {
  if (!map_.hasMap() || !has_goal_) return false; // Nếu không có map hoặc không có goal thì return false

  // Chuyển tọa độ thực → grid
  int start_gx, start_gy, goal_gx, goal_gy;               // Khai báo biến
  map_.worldToGrid(robot_x, robot_y, start_gx, start_gy); 
  map_.worldToGrid(goal_x_, goal_y_,  goal_gx,  goal_gy); 

  // Chạy A*
  auto grid_path = planner_.plan(start_gx, start_gy, goal_gx, goal_gy, map_, config_);
  if (grid_path.empty()) {
    return false;  // Không tìm được đường
  }

  // Chuyển grid → world coordinates
  // Sau khi A* tìm được đường, ta cần chuyển đội lại hệ tọa độ thực (m)
  AStarPath raw_path;
  raw_path.valid = true;
  for (const auto& [gx, gy] : grid_path) {
    double wx, wy;
    map_.gridToWorld(gx, gy, wx, wy);
    // Tích lũy bằng push_back đẩy vào vector điểm path_raw thô
    raw_path.waypoints.push_back({wx, wy});
  }

  // Đơn giản hóa đường đi bằng đường đi zig-zag vuông góc theo grid, loại bỏ các node
  //thừa không cần thiết bằng cách nội suy Douglas-Peucker. Sau đó đánh index duyệt danh
  //sách từ node waypoint
  current_path_   = PathSimplifier::simplify(raw_path, config_.simplify_tolerance, config_.max_waypoint_spacing);
  current_wp_idx_ = 0;
  return true;
}

// ================================================================
//  Khoảng cách Euclidean 2D
// d = sqrt((Δx)^2+(Δy)^2)​ = sqrt((x_2​−x_1​)^2+(y_2​−y_1​)^2)
// ================================================================
double AStarGlobalPlanner::dist(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy); 
}

// ================================================================
//  Góc từ vị trí robot đến điểm target (trong robot frame)
// công thức góc tuyệt đối: α = arctan(dy,dx)
// Trong đó: 
//  - dx = x_target - x_robot
//  - dy = y_target - y_robot
//  - dưới dạng matrix: [dx] = [x_target - x_robot]
//                      [dy]   [y_target - y_robot]
// Công thức góc tương đối: θ_error = α − θ_robot
//  Trong đó:
//   - α: góc tuyệt đối của vector (dx, dy)
//   - θ_robot: góc yaw hiện tại của robot (rad)
//   - θ_error: thành phần góc của R(θ_robot)
//  vector(v_robot) = R(θ_robot) * vector(v_target)
//  R(θ_robot) = [cos(θ_robot)   sin(θ_robot)]
//               [-sin(θ_robot)  cos(θ_robot)]
// ================================================================
double AStarGlobalPlanner::angleToTarget(
    double from_x, double from_y, double theta,
    double to_x, double to_y)
{
  double dx = to_x - from_x;
  double dy = to_y - from_y;
  double angle_to_target = std::atan2(dy, dx);     // góc tuyệt đối
  return normalizeAngle(angle_to_target - theta);  // góc tương đối
}

// ================================================================
//  getCurrentWaypointX — trả về tọa độ X của waypoint hiện tại
// ================================================================
double AStarGlobalPlanner::getCurrentWaypointX() const {
  if (!hasPath() || current_wp_idx_ >= static_cast<int>(current_path_.waypoints.size())) {
    return 0.0;
  }
  return current_path_.waypoints[current_wp_idx_].first;
}

// ================================================================
//  getCurrentWaypointY — trả về tọa độ Y của waypoint hiện tại
// ================================================================
double AStarGlobalPlanner::getCurrentWaypointY() const {
  if (!hasPath() || current_wp_idx_ >= static_cast<int>(current_path_.waypoints.size())) {
    return 0.0;
  }
  return current_path_.waypoints[current_wp_idx_].second;
}

// ================================================================
//  compute — vòng lặp chính mỗi 200ms (chỉ cập nhật state nội bộ)
// ================================================================
void AStarGlobalPlanner::compute(
    double robot_x, double robot_y, double robot_theta,
    int step_count)
{
  // Nếu chưa có goal hoặc chưa có map thì không làm gì
  if (!has_goal_ || !map_.hasMap()) return;
  // Nếu đã đến đích thì không làm gì
  if (goal_reached_) return;

  // --- Kiểm tra cần replan không ---
  bool need_replan = false;

  // [A] Replan triggered: nhận goal mới hoặc path rỗng
  if (has_pending_replan_ || !current_path_.valid || current_path_.empty()) {
    need_replan         = true;
    has_pending_replan_ = false;
  }

  // [B] Replan định kỳ: mỗi replan_interval bước
  // Công thức: k ≡ 0 (mod N)
  // Trong đó:
  //  - k: số bước hiện tại (step counter)
  //  - N: số bước replan interval (config replan interval)
  // Thời gian giữa 2 lần replan là: T_replan = N * Δt
  // Trong đó:
  //  - N: số bước replan interval
  //  - Δt: thời gian của 1 chu kỳ (hiện tại đang setup ở compute() là 200ms)
  if (!need_replan && config_.replan_interval > 0) {
    if (step_count % config_.replan_interval == 0) {
      need_replan = true;
    }
  }

  // --- Thực hiện replan nếu cần ---
  if (need_replan) {
    runAstar(robot_x, robot_y); // Cập nhật current_path_ (giữ path cũ nếu A* thất bại)
  }

  // Vẫn không có path → không làm gì
  if (!hasPath()) return;

  // --- Advance waypoint: robot đến đủ gần waypoint hiện tại → chuyển sang cái tiếp ---
  while (current_wp_idx_ < static_cast<int>(current_path_.waypoints.size())) {
    const auto& wp = current_path_.waypoints[current_wp_idx_];
    double d = dist(robot_x, robot_y, wp.first, wp.second);
    if (d < config_.waypoint_tolerance) {
      current_wp_idx_++; // Chuyển sang waypoint tiếp theo
    } else {
      break;
    }
  }

  // --- Kiểm tra đến Goal chưa ---
  // Kiểm tra vị trí nếu khoảng cách tuyệt đối đến điểm đích ở cuối quỹ đạo thì 
  //xem như hoàn thành nhiệm vụ => ngắt động cơ
  // Công thức: ||p_robot - p_goal||^2 < r_goal^2
  // Trong đó:
  //  - p_robot = (x_robot, y_robot): tọa độ hiện tại của robot
  //  - p_goal = (x_goal, y_goal): tọa độ đích
  //  - r_goal: bán kính dung sai (goal_tolerance)
  double d_to_goal = dist(robot_x, robot_y, goal_x_, goal_y_);
  if (d_to_goal < config_.goal_tolerance) {
    goal_reached_ = true;
    return;
  }

  // Đã hết waypoints nhưng chưa đến goal → replan ở bước sau
  if (current_wp_idx_ >= static_cast<int>(current_path_.waypoints.size())) {
    has_pending_replan_ = true;
    return;
  }

  // A* chỉ cập nhật state nội bộ (current_wp_idx_, current_path_)
  // DWA sẽ lấy waypoint (x,y) và tự tính goal_angle
}
