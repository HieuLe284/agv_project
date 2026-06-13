#include "map_builder.h"

slam::MapBuilder::MapBuilder(double resolution, int width, int height,
                       double origin_x, double origin_y)
    : resolution_(resolution), width_(width), height_(height),
      origin_x_(origin_x), origin_y_(origin_y),
      log_odds_(width * height, 0.0f)
{}

void slam::MapBuilder::setPublisher(
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub)
{
    pub_ = pub;
}

void slam::MapBuilder::updateFromRanges(const std::vector<float>& ranges,
                                  double angle_min, double angle_inc,
                                  double px, double py, double pth)
{
    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (!std::isfinite(r) || r < 0.1f || r > 30.0f) continue;

        // Chuyển LaserScan sang tọa độ thế giới
        // với: 
        // (x_r, y_r, θ_r): pose robot
        // r_i: Khoảng cách tia laser thứ i
        // (x_e, y_e): điểm va chạm vật cản
        double angle = angle_min + i * angle_inc + pth; // θ_i = θ_min + i*Δθ + θ_r
        double ex = px + r * std::cos(angle); // x_e = x_r+r_i*cos(θ_i)
        double ey = py + r * std::sin(angle); // y_e = y_r+r_i*sin(θ_i)

        // Ô xuất phát của tia laser
        int sx, sy;
        worldToGrid(px, py, sx, sy); 

        // Ô cuối tia laser
        int ex_cell, ey_cell;
        worldToGrid(ex, ey, ex_cell, ey_cell);

        // Ray-cast: Các ô không có vật cản nằm trên đường đi của tia laser
        bresenham(sx, sy, ex_cell, ey_cell, kLogOddsFree);

        // Đánh dấu ô tại điểm cuối là có vật cản
        // CHỈ đánh dấu nếu điểm cuối ở ngoài bán kính footprint robot
        // (tránh robot tự vẽ chân mình hoặc nhiễu cự ly gần thành tường)
        if (inBounds(ex_cell, ey_cell)) {
            double dx = ex - px;
            double dy = ey - py;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist > kRobotRadius) {
                log_odds_[ex_cell + ey_cell * width_] =
                    std::min(log_odds_[ex_cell + ey_cell * width_] + kLogOddsOcc,
                             kLogOddsMax); // L_t​(m) = min(L_t​(m),L_max​)
            }
        }
    }
}

// Chuyển dữ liệu bản đồ nội bộ của SLAM thành message ( map -> rviz )
nav_msgs::msg::OccupancyGrid slam::MapBuilder::buildOccupancyGrid(rclcpp::Time stamp) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "map";
    msg.info.resolution = static_cast<float>(resolution_);

    // Thông tin kích thước map
    msg.info.width      = width_;
    msg.info.height     = height_;

    // Thông tin gốc tọa độ bản đồ
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;

    msg.info.origin.orientation.w = 1.0; // Hướng của map

    msg.data.resize(width_ * height_); // Cấp phát bộ nhớ 

    // Chuyển log-Odds -> Occupancy
    for (int i = 0; i < width_ * height_; ++i) {
        float l = log_odds_[i];
        if (l >  kThreshOcc)  msg.data[i] = 100;
        else if (l < kThreshFree) msg.data[i] = 0;
        else                  msg.data[i] = -1;
    }
    return msg;
}

void slam::MapBuilder::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_x_) / resolution_); // g_x​ = ⌊(x_w​−x_origin)/res​​⌋
    gy = static_cast<int>((wy - origin_y_) / resolution_); // g_y = ⌊(y_w​−y_origin)/res​​⌋
}

bool slam::MapBuilder::inBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void slam::MapBuilder::bresenham(int x0, int y0, int x1, int y1, float log_delta) {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        if (x0 == x1 && y0 == y1) break;   // Dừng lại trước endpoint
        if (inBounds(x0, y0)) {
            float& l = log_odds_[x0 + y0 * width_];
            // NẾU log_delta < 0 (free update) và ô đã confident occupied (l >= kThreshOcc)
            // THÌ bỏ qua, không xóa tường
            if (log_delta >= 0 || l < kThreshOcc) {
                l = std::max(-kLogOddsMax, l + log_delta); // L_t​(m) = max(−L_max​,L_t​(m))
            }
        }
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}