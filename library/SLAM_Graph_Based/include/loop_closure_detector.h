/**
 * @file loop_closure_detector.h
 * @brief Phát hiện đóng vòng lặp (Loop Closure) dựa trên độ tương quan giữa các lần quét LiDAR cho Graph-Based SLAM.
 * 
 * ── Loop Closure in Graph-Based SLAM ────────────────────────────────────────
 *
 * Một cạnh đóng vòng lặp (loop closure edge) (i, j) sẽ được thêm vào đồ thị
 * khi robot quay trở lại một khu vực đã từng được khám phá trước đó.
 * Ràng buộc z_ij được tính bằng cách so khớp (matching) dữ liệu quét LiDAR
 * tại node mới j với dữ liệu quét đã lưu của node ứng viên i, từ đó thu được
 * phép biến đổi tương đối (δx, δy, δθ) cùng với nhiễu đo tương ứng.
 *
 *  Stage 1 — Bộ lọc khoảng cách (Proximity Filter):
 *  Chỉ xem xét các cặp node (i, j) thỏa mãn:
 *      ‖t_j − t_i‖ < d_thresh
 * và
 *      |θ_j − θ_i| < θ_thresh
 * trong đó:
 *   - ‖t_j − t_i‖ là khoảng cách Euclid giữa hai pose.
 *   - |θ_j − θ_i| là độ chênh lệch góc định hướng.
 * Bước này giúp giảm số lượng cặp cần kiểm tra từ O(N²)
 * xuống còn một tập ứng viên nhỏ hơn đáng kể.
 *
 *  Stage 2 — So khớp tương quan giữa các scan:
 * Tính hệ số tương quan chéo đã chuẩn hóa (Normalized Cross-Correlation)
 * giữa hai vector dữ liệu khoảng cách:
 *      C(z_i, z_j) = Σ_k [r_i(k) · r_j(k)] / (‖z_i‖ · ‖z_j‖)
 * trong đó:
 *   - r_i(k) là giá trị khoảng cách tại tia quét thứ k của scan i.
 *   - Các giá trị NaN hoặc không hợp lệ được thay thế bằng 0.
 * Hai scan được xem là khớp nhau nếu:
 *      C > correlation_threshold
 * 
 *  Stage 3 — Ước lượng pose tương đối:
 * Đối với các cặp scan được xác nhận là khớp, phép đo tương đối z_ij
 * được tính từ pose hiện tại của các node trong đồ thị:
 *      δt_ij = R_i^T · (t_j − t_i)
 *      δθ_ij = normalize(θ_j − θ_i)
* Trong đó:
 *   - δt_ij là độ dịch chuyển tương đối trong hệ tọa độ của node i.
 *   - δθ_ij là độ thay đổi góc quay đã được chuẩn hóa.
 *
 * Phương pháp này sử dụng pose hiện tại của đồ thị làm điểm khởi tạo
 * (linearization point), phù hợp với cách tuyến tính hóa được sử dụng
 * trong bộ giải Gauss-Newton của Graph-Based SLAM.
 */

#ifndef SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H
#define SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H

#include "pose_graph.h"
#include "jacobian.h"

#include <vector>
#include <cmath>

namespace slam {

class LoopClosureDetector {
public:
    // ── Các tham số có thể điều chỉnh ( Tunable parameters ) ─────────────────
    double dist_threshold{2.0};          // Ngưỡng khoảng cách Euclid lớn nhất giữa hai node [m]
    double angle_threshold{1.2};         // Ngưỡng chênh lệch góc lớn nhất giữa hai node [rad]
    double correlation_threshold{0.80};  // Ngưỡng tương quan scan tối thiểu [0,1]
    int    min_node_gap{10};             // Khoảng cách chỉ số node tối thiểu nhằm tránh so khớp với các node quá gần về thời gian

    // ── Trọng số ma trận thông tin cho cạnh Loop Closure ─────────────────────
    // Giá trị lớn hơn cạnh odometry → thể hiện mức độ tin cậy cao hơn đối với ràng buộc đóng vòng lặp
    double omega_xy{200.0};
    double omega_theta{400.0};

    /**
     * @brief Phát hiện đóng vòng lặp (Loop Closure) cho một node mới được thêm vào đồ thị.
     * Hàm sẽ duyệt qua tất cả các node cũ hơn (cách ít nhất min_node_gap node) và áp dụng bộ lọc hai giai đoạn:
     *   Giai đoạn 1: Lọc theo khoảng cách và góc quay (Proximity Filter)
     *   Giai đoạn 2: Đánh giá độ tương đồng giữa các scan LiDAR bằng hệ số tương quan (Correlation Filter)
     * Nếu tìm thấy nhiều ứng viên phù hợp, node có điểm tương quan cao nhất sẽ được chọn.
     * @param graph Pose Graph cần kiểm tra. Mỗi node phải chứa dữ liệu scan LiDAR. 
     * @param new_idx Chỉ số của node mới được thêm vào đồ thị.
     * @return Chỉ số của node cũ khớp nhất, hoặc trả về -1 nếu không tìm thấy ứng viên đóng vòng lặp phù hợp.
     */
    int detect(PoseGraph2D& graph, int new_idx);

private:
    /**
     * @brief Tính độ tương quan chéo đã chuẩn hóa giữa hai vector 
     * dữ liệu quét khoảng cách (range scan).
     *
     * Theo định lý Cossin
     * C = Σ r_i · r_j / (‖r_i‖ · ‖r_j‖)
     * Trong đó:
     *   - r_i, r_j là hai vector khoảng cách LiDAR.
     *   - r_i · r_j là tích vô hướng (dot product).
     *   - ‖r_i‖, ‖r_j‖ là chuẩn Euclid của hai vector.
     * 
     * Giá trị C nằm trong khoảng [0, 1]:
     *   - C ≈ 1 : Hai scan rất giống nhau.
     *   - C ≈ 0 : Hai scan ít hoặc không tương đồng.
     *
     * Các giá trị không hợp lệ (NaN, Inf hoặc khoảng cách âm)
     * sẽ được thay thế bằng 0 trước khi tính toán.
     *
     * @return Hệ số tương quan đã chuẩn hóa giữa hai scan.
     */
    static double scanCorrelation(const std::vector<double>& a, const std::vector<double>& b);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_LOOP_CLOSURE_DETECTOR_H
