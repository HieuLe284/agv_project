/**
 * @file map_builder.h
 * @brief Xây dựng bản đồ lưới chiếm dụng (Occupancy Grid Mapping)
 *        sử dụng biểu diễn Log-Odds cho Graph-Based SLAM.
 *
 * Cài đặt bản đồ chiếm dụng xác suất dựa trên biểu diễn Log-Odds:
 *
 *   l_t(x) = l_{t-1}(x) + log[ p(x|z_t) / (1 - p(x|z_t)) ]
 *                        − log[ p_0 / (1 - p_0) ]
 *
 * Trong đó: p_0 = 0.5 là xác suất tiên nghiệm đồng đều
 *          (tương ứng log-odds = 0).
 *
 * Trạng thái các ô lưới theo chuẩn ROS OccupancyGrid:
 *   -1   → Chưa biết (unknown)   (|l| < free_thresh)
 *    0   → Ô trống (free)        ( l  < -free_thresh)
 *  100   → Có vật cản (occupied) ( l  > occ_thresh)
 * 
 * Mỗi tia LiDAR được dò theo thuật toán đường thẳng Bresenham:
 *   • Các ô nằm trên đường đi của tia laser trước điểm va chạm
 *     được đánh dấu là ô trống:
 *          L_t(m) = L_(t-1)(m) + L_free
 *   • Ô tại điểm cuối tia laser (điểm va chạm) được đánh dấu là ô có vật cản:
 *          L_t(m) = L_(t-1)(m) + l_occ
 */

#ifndef SLAM_GRAPH_BASED_MAP_BUILDER_H
#define SLAM_GRAPH_BASED_MAP_BUILDER_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <algorithm>
#include <cmath>

namespace slam {

class MapBuilder {
public:
    MapBuilder() = default;
    /**
     * @param resolution Độ phân giải của bản đồ lưới [m/ô]
     * @param width Chiều rộng bản đồ [số ô]
     * @param height Chiều cao bản đồ [số ô]
     * @param origin_x Tọa độ x trong hệ thế giới của ô (0,0) [m]
     * @param origin_y Tọa độ y trong hệ thế giới của ô (0,0) [m]
     */
    MapBuilder(double resolution, int width, int height,
               double origin_x, double origin_y);

    void setPublisher(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub);

    // Xóa toàn bộ bản đồ bằng cách đặt tất cả giá trị log-odds về 0
    // (trạng thái chưa biết - unknown)
    void clearMap();

    /**
     * @brief Cập nhật bản đồ lưới từ một lần quét LiDAR tại pose đã biết.
     * Với mỗi giá trị khoảng cách hợp lệ r tại góc φ:
     *      endpoint  = ( pose_x + r·cos(φ + θ), pose_y + r·sin(φ + θ))
     * Thuật toán Bresenham được sử dụng để dò tia:
     *   - Các ô nằm trên đường đi của tia được đánh dấu là ô trống.
     *   - Ô tại điểm cuối tia được đánh dấu là ô có vật cản.
     * @param ranges Dữ liệu khoảng cách LiDAR [m]
     * @param angle_min Góc bắt đầu của tia quét [rad]
     * @param angle_inc Độ tăng góc giữa hai tia liên tiếp [rad]
     * @param px, py, pth Pose của robot trong hệ tọa độ toàn cục (x, y, θ)
     */
    void updateFromRanges(const std::vector<float>& ranges,
                          double angle_min, double angle_inc,
                          double px, double py, double pth);

    // Tạo thông điệp ROS OccupancyGrid từ lưới log-odds hiện tại
    nav_msgs::msg::OccupancyGrid buildOccupancyGrid(rclcpp::Time stamp) const;

    void publishMap(rclcpp::Time stamp); // Publish bản đồ OccupancyGrid lên ROS topic

    int    width_{0}, height_{0}; // Kích thước bản đồ lưới (được sử dụng bên ngoài lớp)
    double resolution_{0.05}; // Độ phân giải bản đồ [m/ô]

    // Bán kính footprint robot [m] — các ô trong phạm vi này không được đánh occupied
    // (tránh robot tự vẽ chân mình thành tường)
    static constexpr float kRobotRadius = 0.18f;

private:
    // Tọa độ gốc của bản đồ trong hệ thế giới
    double origin_x_{0.0}, origin_y_{0.0};
    std::vector<float> log_odds_;  // Lưới log-odds có kích thước width × height

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

    // Các tham số cập nhật Log-Odds (có thể tinh chỉnh)
    static constexpr float kLogOddsFree = -0.4f;  // Mức tăng độ tin cậy rằng ô là ô trống
    static constexpr float kLogOddsOcc  =  2.0f;  // Mức tăng độ tin cậy rằng ô có vật cản (tăng lên 2.0 để tường không bị xóa bởi free ray)
    static constexpr float kLogOddsMax  =  10.0f; // Giới hạn trên/dưới của log-odds để tránh bão hòa
    static constexpr float kThreshOcc   =  0.5f;  // Ngưỡng xác định ô có vật cản
    static constexpr float kThreshFree  = -0.5f;  // Ngưỡng xác định ô trống

    // Chuyển đổi tọa độ thế giới (World Coordinate) sang chỉ số ô lưới (Grid Coordinate)
    void worldToGrid(double wx, double wy, int& gx, int& gy) const;

    // Kiểm tra xem một ô lưới có nằm trong phạm vi bản đồ hay không
    bool inBounds(int gx, int gy) const; 

    /**
     * @brief Dò tia bằng thuật toán Bresenham.
     * Đánh dấu các ô từ (x0, y0) đến (x1, y1)
     * bằng cách cộng giá trị log_delta vào log-odds.
     * Thường được sử dụng để:
     *   - Đánh dấu các ô trống dọc theo tia laser (log_delta = kLogOddsFree)
     *   - Đánh dấu ô có vật cản tại điểm cuối tia (log_delta = kLogOddsOcc)
     */
    void bresenham(int x0, int y0, int x1, int y1, float log_delta);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_MAP_BUILDER_H
