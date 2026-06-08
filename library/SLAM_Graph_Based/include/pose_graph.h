/**
 * @file pose_graph.h
 * @brief Các cấu trúc dữ liệu Pose Graph cho Graph-Based SLAM
 * ── Mô hình dữ liệu (Data Model) ──────────────────────────────────────────
 * Pose Graph:
 *      G = (V, E)
 * gồm hai thành phần chính:
 *   • Đỉnh (Vertex / Node):
 *       Mỗi node:
 *          x_i = (x_i, y_i, θ_i)^T
 *       biểu diễn một pose ước lượng của robot trong hệ tọa độ toàn cục.
 *       Ngoài pose, node còn lưu dữ liệu quét LiDAR được thu tại vị trí đó.
 *   • Cạnh (Edge):
 *       Mỗi cạnh:
 *          (i, j, z_ij, Ω_ij) biểu diễn một ràng buộc không gian giữa hai pose.
 *       Trong đó:
 *          z_ij là phép đo chuyển động tương đối giữa pose i và pose j.
 *       Độ tin cậy của phép đo được biểu diễn bởi:
 *          Ω_ij = Σ_ij⁻¹
 *       (ma trận thông tin - nghịch đảo của ma trận hiệp phương sai).
 * Các loại cạnh:
 *   • Odometry Edge:
 *       Nối giữa các node liên tiếp: i → i+1
 *       Có độ tin cậy trung bình và thường tích lũy sai số theo thời gian
 *       (drift).
 *   • Loop Closure Edge:
 *       Nối giữa các node không liên tiếp.
 *       Được tạo ra khi robot quay lại khu vực đã từng đi qua.
 *       Thường có trọng số cao hơn vì dựa trên việc so khớp dữ liệu cảm biến.
 */

#ifndef SLAM_GRAPH_BASED_POSE_GRAPH_H
#define SLAM_GRAPH_BASED_POSE_GRAPH_H

#include <vector>
#include <cmath>

namespace slam {

// ═══════════════════════════════════════════════════════════════════════════
//  Node2D — Đỉnh của Pose Graph
//  Biểu diễn pose:
//      x_i = (x, y, θ)^T
//  trong hệ tọa độ bản đồ (map frame).
//  Đồng thời lưu dữ liệu quét LiDAR tại pose này để phục vụ phát hiện Loop Closure.
// ═══════════════════════════════════════════════════════════════════════════
struct Node2D {
    double x{0.0};      // Tọa độ x trong hệ toàn cục [m]
    double y{0.0};      // Tọa độ y trong hệ toàn cục [m]
    double theta{0.0};  // Góc định hướng θ trong hệ toàn cục [rad]

    // Dữ liệu khoảng cách LiDAR ghi nhận tại pose này (dùng cho loop closure)
    std::vector<double> scan_ranges;

    double scan_angle_min{0.0}; // Góc bắt đầu của LaserScan
    double scan_angle_increment{0.0}; // Độ phân giải góc giữa hai tia liên tiếp

    Node2D() = default;
    Node2D(double x_, double y_, double t_) : x(x_), y(y_), theta(t_) {}
};

// ═══════════════════════════════════════════════════════════════════════════
//  Edge2D — Ràng buộc giữa hai node trong Pose Graph
//  Chứa phép đo tương đối:
//      z_ij = (δx, δy, δθ)^T
//  từ pose i tới pose j.
//
//  Đồng thời lưu ma trận thông tin: Ω_ij = Σ_ij^{-1}
//  Trong đó:
//       Σ_ij là ma trận hiệp phương sai của phép đo.
//  Ý nghĩa: 
//     - Phần tử đường chéo càng lớn → độ tin cậy càng cao.
//     - Phần tử đường chéo càng nhỏ  → phép đo càng nhiễu.
//  
//   Ví dụ: 
//     Odometry Edge: 
//        Ω = diag(50, 50, 100) 
//     Loop Closure Edge: 
//        Ω = diag(200, 200, 400) 
//  Loop Closure thường có trọng số lớn hơn để giúp giảm drift của toàn bộ graph.
// ═══════════════════════════════════════════════════════════════════════════
struct Edge2D {
    int from{-1};       // Chỉ số node nguồn i
    int to{-1};         // Chỉ số node đích j
    bool is_loop{false};// true nếu đây là ràng buộc Loop Closure

    // Phép đo tương đối: z_ij = (δx, δy, δθ)^T
    double z_x{0.0};     // δx
    double z_y{0.0};     // δy
    double z_theta{0.0}; // δθ

    // Ma trận thông tin Ω_ij kích thước 3×3
    //   Ω = diag(ω_x, ω_y, ω_θ) 
    double omega[3][3]{};

    Edge2D();

    /**
     * Thiết lập ma trận thông tin dạng đường chéo:
     *      Ω = [ ω_x      0       0 ]
     *          [  0      ω_y      0 ]
     *          [  0       0     ω_θ ]
     */
    void setOmegaDiagonal(double wx, double wy, double wt);
};

// ═══════════════════════════════════════════════════════════════════════════
//  PoseGraph2D — Toàn bộ đồ thị G = (V, E)
//      V = tập các node
//      E = tập các cạnh ràng buộc
// ═══════════════════════════════════════════════════════════════════════════
struct PoseGraph2D {
    std::vector<Node2D> nodes;  // Tập node: V = {x_0, x_1, ..., x_{N-1}}
    std::vector<Edge2D> edges;  // Tập cạnh: E = {e_ij | (i,j) ∈ constraints}

    int numNodes() const; // Trả về số lượng node trong graph
    int numEdges() const; // Trả về số lượng cạnh trong graph

    /**
     * Thêm một cạnh vào Pose Graph.
     * Có thể là:
     *   - Odometry Edge
     *   - Loop Closure Edge
     * @param i Node nguồn
     *
     * @param j Node đích
     *
     * @param zx, zy, zt Phép đo tương đối: z_ij = (δx, δy, δθ)
     * @param wx, wy, wt Trọng số đường chéo của ma trận thông tin Ω
     * @param loop
     *        true  → Loop Closure Edge
     *        false → Odometry Edge
     */
    void addEdge(int i, int j,
                 double zx, double zy, double zt,
                 double wx, double wy, double wt,
                 bool loop = false);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_POSE_GRAPH_H
