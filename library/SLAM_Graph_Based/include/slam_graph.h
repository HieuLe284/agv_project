/**
 * @file slam_graph.h
 * @brief SlamGraph — Bộ điều phối (Coordinator) của toàn bộ quy trình Graph-Based SLAM.
 * ── Tổng quan quy trình xử lý (Pipeline Overview) ──────────────────────────
 * Tại mỗi thời điểm t:
 *   1. FRONT-END — addOdometryNode(x, y, θ, scan)
 *      • Tính chuyển động tương đối trong hệ tọa độ robot từ node trước đó:
 *          δx = R_{i-1}^T · (t_i − t_{i-1})
 *          δθ = normalize(θ_i − θ_{i-1})
 *      • Thêm node mới x_i vào Pose Graph.
 *      • Thêm cạnh odometry: (i−1 → i) với phép đo tương đối z_odom và ma trận thông tin Ω_odom.
 *   2. PHÁT HIỆN ĐÓNG VÒNG LẶP (LOOP CLOSURE DETECTION) — addLoopClosures(new_idx):
 *      • Tìm kiếm các node cũ hơn trong đồ thị.
 *      • So sánh dữ liệu quét LiDAR để phát hiện các Ω_loop
 *      • Nếu tìm thấy sự tương đồng đủ lớn: Thêm cạnh Loop Closure với ma trận thông tin Ω_loop lớn hơn.
 *      • Cập nhật tổng số Loop Closure đã phát hiện.
 *
 *   3. BACK-END — TỐI ƯU HÓA ĐỒ THỊ — optimizeIfNeeded():
 *      • Nếu trong bước hiện tại xuất hiện một Loop Closure mới:
 *          Chạy Gauss-Newton: x* = argmin Σ e_ij^T Ω_ij e_ij
 *      • Hàm trả về: true nếu quá trình tối ưu hóa được thực hiện.
 *        Điều này báo cho tầng gọi bên ngoài biết rằng: Bản đồ cần được xây dựng lại.
 *
 *   4. TÁI TẠO BẢN ĐỒ (MAP REBUILD) — Được thực hiện bởi slam_robot.cpp. Thông qua: getNodes()
 *      Với mỗi node đã tối ưu: mapBuilder_.updateFromRanges() sẽ được gọi để xây dựng lại 
 *      Occupancy Grid bằng dữ liệu LiDAR và pose mới.
 *
 * ── Odometry Noise Model ─────────────────────────────────────────────────────
 * Ma trận thông tin của cạnh Odometry (xấp xỉ đường chéo):
 *      Ω_odom = diag(ω_x, ω_y, ω_θ)
 * Ma trận thông tin của cạnh Loop Closure:
 *      Ω_loop = diag(ω_x_loop, ω_y_loop, ω_θ_loop) ( higher values )
 * Các trọng số Loop Closure thường lớn hơn nhằm phản ánh độ tin cậy cao hơn của ràng buộc đóng vòng lặp.
 */

#ifndef SLAM_GRAPH_BASED_SLAM_GRAPH_H
#define SLAM_GRAPH_BASED_SLAM_GRAPH_H

#include "pose_graph.h"
#include "gauss_newton_solver.h"
#include "loop_closure_detector.h"
#include "map_builder.h"
#include "jacobian.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <vector>

namespace slam {

class SlamGraph {
public:
    // ── Trạng thái công khai (Public state) ───────────────────────────────
    PoseGraph2D          pose_graph;             // Pose Graph: G = (V, E)
    int                  loop_closure_count_{0}; // Tổng số Loop Closure đã phát hiện

    // ── Trọng số ma trận thông tin của cạnh Odometry ──────────────────────
    double odom_omega_xy{50.0};    // Trọng số theo phương x và y
    double odom_omega_theta{80.0}; // Trọng số theo góc θ

    // ── Số vòng lặp Gauss-Newton mỗi lần tối ưu ───────────────────────────
    int gn_iterations{10};

    // ── Ngưỡng di chuyển tối thiểu trước khi tạo node mới ─────────────────
    double min_travel_dist{0.30};    // Khoảng cách tối thiểu [m]
    double min_travel_angle{0.15};   // Góc quay tối thiểu [rad]

    SlamGraph() = default;

    /**
     * @brief Khởi tạo Pose Graph với node neo (anchor node) tại vị trí: (0,0,0)
     * Hàm này phải được gọi một lần duy nhất trước khi xử lý dữ liệu cảm biến.
     */
    void init();

    /**
     * @brief Thiết lập đối tượng MapBuilder phục vụ việc tái tạo bản đồ sau tối ưu hóa.
     * @param mb Con trỏ tới đối tượng MapBuilder được sở hữu bởi slam_robot.
     */
    void setMapBuilder(MapBuilder* mb);

    /**
     * @brief [Front-End] Thêm một node odometry mới vào Pose Graph.
     * Một node mới chỉ được thêm khi robot đã di chuyển đủ xa(vượt quá min_travel_dist 
     * hoặc min_travel_angle) kể từ node trước đó nhằm giới hạn kích thước đồ thị.
     * Trong Graph SLAM, mỗi cạnh biểu diễn một phép đo tương đối:
     *      z_ij = [ z_x ]
     *             [ z_y ]
     *             [ z_θ ]
     * Sai số của cạnh:
     *      e_ij(x_i,x_j)  = z_ij - ẑ_ij(x_i,x_j)
     * Hàm chi phí cần tối thiểu hóa:
     *      F(x) = Σ e_ij^T Ω_ij e_ij
     * Đối với cạnh odometry giữa hai node liên tiếp:
     *      z_(i−1,i)
     * phép đo tương đối được tính:
     *      δt =  R_(i−1)^T(t_i − t_(i−1))
     *      δθ =  normalize( θ_i − θ_(i−1))
     * hay:
     *      z_(i−1,i) = [ δx ]
     *                  [ δy ]
     *                  [ δθ ]
     * trong đó:
     *      δt = [δx δy]^T
     * là độ dịch chuyển trong hệ tọa độ robot (body-frame translation).
     * @param x, y, theta Pose hiện tại của robot (TF: map → base_link)
     * @param ranges Dữ liệu khoảng cách LiDAR.
     * @param angle_min, angle_inc Góc bắt đầu và độ phân giải góc của scan. 
     * @return Chỉ số node mới được thêm, hoặc -1 nếu robot chưa di chuyển đủ xa.
     */
    int addOdometryNode(double x, double y, double theta,
                        const std::vector<double>& ranges,
                        double angle_min, double angle_inc);

    /**
     * @brief [Front-End] Phát hiện đóng vòng lặp cho node mới nhất.
     * Hàm sử dụng:
     *   • Bộ lọc khoảng cách
     *   • Bộ lọc góc quay
     *   • Độ tương đồng giữa các scan LiDAR để tìm node cũ phù hợp nhất.
     * @param new_idx Chỉ số node vừa được thêm.
     * @return Chỉ số node khớp nhất, hoặc -1 nếu không tìm thấy.
     */
    int addLoopClosures(int new_idx);

    /**
     * @brief [Back-End] Thực hiện tối ưu hóa Pose Graph bằng thuật toán Gauss-Newton.
     * Chỉ chạy khi:  xuất hiện Loop Closure mới. Hàm mục tiêu:
     *      x* = argmin_x  Σ_{<i,j>∈C}  e_ij^T Ω_ij e_ij
     * @return
     *        true: nếu tối ưu hóa đã được thực hiện.
     *        false: nếu không cần tối ưu hóa
     */
    bool optimizeIfNeeded();

    // ── Map Builder passthrough ───────────────────────────────────────────
    void clearMap(); // Xóa toàn bộ Occupancy Grid hiện tại

    void updateMapFromNode(int node_idx); // Cập nhật bản đồ từ dữ liệu của một node

    // ── Tuning proxies ────────────────────────────────────────────────────
    /**
     * @brief Trả về đối tượng LoopClosureDetector để điều chỉnh các tham số:
     *          dist_threshold
     *          angle_threshold
     *          correlation_threshold
     *          ...
     */
    LoopClosureDetector& loopDetector();

private:
    LoopClosureDetector  loop_detector_;             // Bộ phát hiện Loop Closure
    MapBuilder*          map_builder_{nullptr};      // Con trỏ tới MapBuilder
    bool                 new_loop_this_step_{false}; // Đánh dấu có Loop Closure mới trong bước hiện tại hay không
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_SLAM_GRAPH_H
