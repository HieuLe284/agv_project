#ifndef SLAM_ROBOT_H
#define SLAM_ROBOT_H

// ── ROS2 core ────────────────────────────────────────────────────────────────
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

// ── Math utilities ────────────────────────────────────────────────────────────
#include "library/common/math_utils.h"

// ── Standard library ─────────────────────────────────────────────────────────
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

// ── Graph-Based SLAM Library (lib/SLAM_Graph_Based) ──────────────────────────
#include "library/SLAM_Graph_Based/include/slam_graph.h"
#include "library/SLAM_Graph_Based/include/map_builder.h"

// ── Frontier-Based Exploration Library (lib/frontier_based) ──────────────────
#include "library/frontier_based/include/frontier_exploration.h"
#include "library/frontier_based/include/frontier_detector.h"

// ── A* Global Planner Library (lib/A_star_algorithm) ─────────────────────────
#include "library/A_star_algorithm/include/astar_global_planner.h"

// ── DWA Local Planner Library (lib/DWA) ──────────────────────────────────────
#include "library/DWA/include/dwa_planner.h"

using std::placeholders::_1;

/**
 * @struct Pose2D
 * @brief 2D pose (x, y, theta) — dùng cho frontier-based exploration
 */
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_)
    : x(x_), y(y_), theta(theta_) {}
};

// ═════════════════════════════════════════════════════════════════════════════
//  SlamRobot — ROS2 Node thực hiện Graph-Based SLAM + Frontier Exploration
//  Kiến trúc hệ thống:
//    • scanCallback()
//        - Nhận dữ liệu LiDAR
//        - Lưu scan vào bộ nhớ đệm (cache)
//        - Cập nhật / xuất bản bản đồ
//    • slamTimerCallback()
//        - Lấy pose robot từ TF
//        - Gọi graphSLAMcall()
//        - Xuất bản dữ liệu trực quan hóa Pose Graph
//    • graphSLAMcall()
//        - Hàm SLAM duy nhất của toàn hệ thống:
//            1) addOdometryNode()  (Front-End)
//            2) addLoopClosures() (Front-End)
//            3) optimizeIfNeeded() (Back-End Gauss-Newton)
//            4) Rebuild Map nếu đồ thị được tối ưu
//    • slam_exploration()
//        - Frontier-Based Exploration (Yamauchi 1997):
//            • Cập nhật OccupancyGrid từ SLAM MapBuilder
//            • Phát hiện frontier regions bằng Wave-Front BFS
//            • Chọn frontier tốt nhất theo cost function
//            • Xuất bản markers visualization lên RViz
//              + A* Global Planner + DWA Local Planner
// ═════════════════════════════════════════════════════════════════════════════
class SlamRobot : public rclcpp::Node {
public:
    SlamRobot();

private:
    /**
     * @brief Helper chuyển tf2::Quaternion sang góc yaw [rad]
     * @param q Tham chiếu tf2::Quaternion
     * @return Góc yaw [rad]
     */
    static inline double getYaw(const tf2::Quaternion& q) {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    /**
     * @brief Phát TF từ map → odom. Được sử dụng để đồng bộ hệ tọa độ bản đồ (map)
     * và hệ tọa độ odometry (odom).
     */
    void broadcastMapOdomTF(rclcpp::Time now);

    /**
     * Luồng xử lý:
     *      TF
     *       ↓
     *  graphSLAMcall()
     *       ↓
     *  Publish Pose Graph
     *       ↓
     *  Publish Visualization
     *       ↓
     *     Rviz
     */
    void slamTimerCallback();

    /**
     * @brief Cập nhật phép biến đổi map → odom.
     * @param map_x,map_y,map_theta Pose trong hệ map.
     * @param odom_x,odom_y,odom_theta Pose trong hệ odom.
     */
    void updateMapOdom(double map_x, double map_y, double map_theta,
                       double odom_x, double odom_y, double odom_theta);

    /**
     * @brief Callback nhận dữ liệu LiDAR.
     * Chức năng:
     *   • Lưu LaserScan vào cache.
     *   • Cung cấp dữ liệu cho: graphSLAMcall()
     *   • Cập nhật MapBuilder liên tục.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // ── Map publish timer ─────────────────────────────────────────────────
    void mapBuilderTimerCallback();

    /**
     * @brief Hàm xử lý SLAM duy nhất của hệ thống. 
     * Thực hiện đầy đủ một vòng Graph-Based SLAM. 
     * ────────────────────────────────────────────────────────────
     * Bước 1 — Front-End (Odometry) 
     *   Tính chuyển động tương đối: 
     *       δ = (δx, δy, δθ) từ node trước đó. 
     *   Thêm:
     *       • Node mới
     *       • Cạnh Odometry vào Pose Graph.
     * ────────────────────────────────────────────────────────────
     * Bước 2 — Front-End (Loop Closure)
     *   Tìm kiếm các node cũ.
     *   So sánh dữ liệu LiDAR bằng: Scan Correlation Nếu độ tương 
     *   đồng vượt ngưỡng: Thêm cạnh Loop Closure với trọng số thông 
     *   tin lớn hơn.
     * ────────────────────────────────────────────────────────────
     * Bước 3 — Back-End (Gauss-Newton)
     *   Nếu vừa phát hiện Loop Closure:
     *       x* = argmin Σ e_ijᵀ Ω_ij e_ij
     *   Xây dựng:
     *       HΔξ = -b
     *   Giải bằng: Gaussian Elimination Sau đó cập nhật toàn bộ pose.
     * ────────────────────────────────────────────────────────────
     * Bước 4 — Tái tạo bản đồ
     *   Nếu đồ thị vừa được tối ưu: clearMap()
     *   Sau đó: updateMapFromNode() cho toàn bộ node trong graph.

     * Bước 5 — Cập nhật map→odom TF
     * @param x,y,theta Pose hiện tại lấy từ: odom → base_link
     * * @return -1 nếu không có node mới, 0 nếu có node mới, 1 nếu có node mới + optimized
     */
    int graphSLAMcall(double x, double y, double theta);

    // ═══════════════════════════════════════════════════════════════════════
    //  Frontier-Based Exploration (Yamauchi, 1997)
    // ═══════════════════════════════════════════════════════════════════════
    /**
     * @brief Thực hiện Frontier-Based Exploration.
     *
     * Mỗi chu kỳ gọi frontier_explorer_.compute() để tìm frontier goal mới
     * dựa trên OccupancyGrid hiện tại từ SLAM. Kết quả được xuất bản dưới
     * dạng markers visualization lên RViz.
     *
     * Reference: Yamauchi — "A Frontier-Based Approach for Autonomous Exploration" (1997)
     *
     * @param cur  Pose hiện tại của robot (x, y, theta) trong map frame
     */
    void slam_exploration(Pose2D& cur);
    
    /**
     * @brief Publish frontier regions + goal visualization lên /frontier_markers
     *
     * @param regions   Danh sách FrontierRegion từ detector
     * @param best      Con trỏ đến region tốt nhất (hoặc nullptr)
     * @param robot_x   Vị trí X của robot [m]
     * @param robot_y   Vị trí Y của robot [m]
     * @param goal_x    Tọa độ X của frontier goal [m]
     * @param goal_y    Tọa độ Y của frontier goal [m]
     */
    void publishFrontierMarkers(
        const std::vector<FrontierRegion>& regions, const FrontierRegion* best,
        double robot_x, double robot_y, double goal_x, double goal_y);

    /**
     * @brief Callback cho frontier timer — cập nhật map, chạy exploration, publish markers
     */
    void frontierTimerCallback();

    // ═══════════════════════════════════════════════════════════════════════
    //  A* Global Planner (Hart, Nilsson, Raphael, 1968)
    // ═══════════════════════════════════════════════════════════════════════

    void globalPlannerTimerCallback();
    void slam_globalPlanner(double rx, double ry, double rth);
    void publishGlobalPath();
    void publishAStarWaypoints();

    // ═══════════════════════════════════════════════════════════════════════
    //  DWA Local Planner (Fox, Burgard, Thrun, 1997)
    // ═══════════════════════════════════════════════════════════════════════

    void localPlannerTimerCallback();
    void slam_localPlanner(double rx, double ry, double rth,
                           double rv, double rw);

    // TF map → odom (hiệu chỉnh SLAM)
    double map_odom_x = 0.0, map_odom_y = 0.0, map_odom_theta = 0.0; // map→odom TF (identity)

    // ── Publishers ────────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_cmd_; // Điều khiển robot
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      pub_map_; // Occupancy Grid Map
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pub_graph_nodes_; // Pose Graph nodes
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_edges_; // Pose Graph edges
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pub_loop_closure_event_; // Thông báo phát hiện Loop Closure
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       pub_scan_visualization_; // LaserScan dùng cho trực quan hóa
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frontier_markers_; // Frontier markers visualization
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_astar_waypoints_;

    // ── Subscribers ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_; // Nhận dữ liệu LiDAR

    // ── Timers ────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_; // 5 Hz SLAM timer (no viz!)
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_; // 50 Hz TF broadcast (decoupled from SLAM)
    rclcpp::TimerBase::SharedPtr map_timer_; // 2 Hz map publish timer
    rclcpp::TimerBase::SharedPtr graph_viz_timer_; // 0.5 Hz graph visualization
    rclcpp::TimerBase::SharedPtr frontier_timer_; // 2 Hz frontier exploration timer
    rclcpp::TimerBase::SharedPtr global_planner_timer_;
    rclcpp::TimerBase::SharedPtr local_planner_timer_;

    // ── Graph viz dirty flag ──────────────────────────────────────────────
    std::atomic<bool> graph_viz_dirty_{true};
    bool graph_has_changed_{false}; // set by graphSLAMcall, read+cleared by viz timer
    void graphVizTimerCallback();

    // ── TF2 ──────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;       // Bộ đệm TF
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;     // Nhận TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // Phát TF

    // ── Callback Groups ───────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr callback_group_lidar_; // Nhóm callback LiDAR
    rclcpp::CallbackGroup::SharedPtr callback_group_slam_; // Nhóm callback SLAM
    rclcpp::CallbackGroup::SharedPtr callback_group_frontier_; // Nhóm callback Frontier
    rclcpp::CallbackGroup::SharedPtr callback_group_global_planner_;
    rclcpp::CallbackGroup::SharedPtr callback_group_local_planner_;

    // ── Graph-Based SLAM ──────────────────────────────────────────────────
    slam::SlamGraph  slam_graph_; // Full SLAM pipeline 
    slam::MapBuilder map_builder_; // Log-Odds occupancy grid

    // ── Dữ liệu LiDAR được lưu tạm ────────────────────────────────────────
    std::vector<double> cached_scan_ranges_; // Chia sẻ giữa scanCallback & graphSLAMcall
    double cached_scan_angle_min_{0.0}; // Góc bắt đầu của LaserScan
    double cached_scan_angle_increment_{0.0}; // Độ phân giải góc

    // ── Map state ─────────────────────────────────────────────────────────
    bool map_initialized_{false}; // true = khởi tạo

    // ── Cached OccupancyGrid (chỉ build 1 lần, dùng cho cả frontier + A* + publish) ──
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid cached_occ_grid_; // snapshot của occupancy grid
    bool cached_grid_valid_{false};
    void refreshCachedGrid(); // build fresh + swap

    // ═══════════════════════════════════════════════════════════════════════
    //  Frontier-Based Exploration Members
    // ═══════════════════════════════════════════════════════════════════════

    std::atomic<bool> exploration_mode_{false}; // Bật/tắt chế độ thám hiểm

    FrontierExploration frontier_explorer_; // Frontier-Based Exploration coordinator
   
    // Dữ liệu frontier từ lần detect gần nhất (dùng cho visualization)
    std::vector<FrontierRegion> cached_regions_;
    double cached_robot_x_{0.0};
    double cached_robot_y_{0.0};
    double cached_goal_x_{0.0};
    double cached_goal_y_{0.0};
    double active_goal_x_{0.0};
    double active_goal_y_{0.0};
    bool   active_goal_valid_{false};
    bool   cached_has_goal_{false};

    // ═══════════════════════════════════════════════════════════════════════
    //  A* Global Planner Members
    // ═══════════════════════════════════════════════════════════════════════

    AStarGlobalPlanner global_planner_{AStarConfig()};

    std::vector<std::pair<double, double>> cached_global_path_; // Nhận path từ A*
    bool  cached_path_valid_{false};
    int   global_planner_step_{0};

    // ═══════════════════════════════════════════════════════════════════════
    //  DWA Local Planner Members
    // ═══════════════════════════════════════════════════════════════════════

    DWAPlanner dwa_planner_{DWAConfig()};
    double current_v_{0.0};
    double current_w_{0.0};

    // ═══════════════════════════════════════════════════════════════════════
    //  Stuck Detection & Recovery
    // ═══════════════════════════════════════════════════════════════════════
    bool   stuck_detected_{false};          // true = đang trong chế độ thoát kẹt
    double stuck_prev_x_{0.0};              // Pose X lần kiểm tra trước [m]
    double stuck_prev_y_{0.0};              // Pose Y lần kiểm tra trước [m]
    double stuck_movement_timer_{0.0};      // Thời gian không di chuyển [s]
    double stuck_escape_elapsed_{0.0};      // Thời gian đã thực hiện escape [s]
    int    stuck_phase_{0};                 // 0=normal, 1=lùi, 2=xoay, 3=chờ hồi phục
    static constexpr double STUCK_DIST_THRESH = 0.03;     // Ngưỡng di chuyển tối thiểu [m]
    static constexpr double STUCK_TIME_THRESH  = 1.5;     // Thời gian không move để trigger [s]
    static constexpr double STUCK_REVERSE_V    = -0.06;   // Vận tốc lùi khi kẹt [m/s]
    static constexpr double STUCK_REVERSE_TIME = 0.6;     // Thời gian lùi [s]
    static constexpr double STUCK_TURN_W       = 0.8;     // Vận tốc xoay khi kẹt [rad/s]
    static constexpr double STUCK_TURN_TIME    = 1.0;     // Thời gian xoay [s]
    static constexpr double STUCK_RECOVERY_TIME = 0.5;    // Thời gian hồi phục sau escape [s]
};

#endif  // SLAM_ROBOT_H


