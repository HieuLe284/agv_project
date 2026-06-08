/**
 * @file gauss_newton_solver.h
 * @brief Bộ tối ưu hóa Pose Graph bằng thuật toán Gauss-Newton cho Graph-Based SLAM
 * ── Algorithm Overview ───────────────────────────────────────────────────────
 * Mục tiêu của quá trình tối ưu là tìm tập pose tối ưu x* sao cho tổng khoảng cách 
 * Mahalanobis bình phương của tất cả các sai số ràng buộc trong đồ thị là nhỏ nhất:
 *    x* = argmin_x  Σ_{<i,j>∈C}  e_ij(x_i, x_j)^T · Ω_ij · e_ij(x_i, x_j)
 *    với mọi cạnh <i,j> ∈ C
 * Trong đó:
 *   - e_ij là sai số (residual) của cạnh giữa node i và j
 *   - Ω_ij là ma trận thông tin (Information Matrix)
 *   - C là tập tất cả các ràng buộc trong đồ thị
 *
 * Đây là một bài toán tối ưu bình phương tối thiểu phi tuyến (Nonlinear Least Squares)
 * và được giải bằng phương pháp lặp Gauss-Newton.
 *
 * ── One Gauss-Newton Iteration ──────────────────────────────────────────────
 *
 *  Step 1 — Tuyến tính hóa Linearize với mỗi edge (i, j):
 *    Tính residual:      
 *      e_ij  ∈ R^3     
 *    Tính Jacobians:
 *      A_ij = ∂e_ij / ∂x_i
 *      B_ij = ∂e_ij / ∂x_j
 *      A_ij, B_ij ∈ R^(3×3)
 * 
 *  Step 2 — Xây dựng hệ phương trình tuyến tính
 *    H_{ii} += A_ij^T · Ω_ij · A_ij      (3×3 block of H)
 *    H_{ij} += A_ij^T · Ω_ij · B_ij
 *    H_{ji} += B_ij^T · Ω_ij · A_ij
 *    H_{jj} += B_ij^T · Ω_ij · B_ij
 *
 *    b_i    += A_ij^T · Ω_ij · e_ij      (3-vector block of b)
 *    b_j    += B_ij^T · Ω_ij · e_ij
 *  với: b_i, b_j ∈ R³
 *
 *  Step 3 — Cố định hệ quy chiếu (Gauge Fix)
 * Do Pose Graph chỉ chứa các ràng buộc tương đối, nghiệm của hệ có thể không duy nhất.
 * Để loại bỏ bậc tự do dư thừa (Gauge Freedom), node số 0 được neo cố định bằng cách 
 * cộng một giá trị lớn vào đường chéo của khối H_00.
 *
 *  Step 4 — Giải hệ tuyến tính
 *    H · Δξ = -b  (phương pháp khử Gauss - Gaussian elimination)
 *
 *  Step 5 — Cập nhật pose
 *  Trong bài toán 2D:
 *      x_i += Δξ_i[0]
 *      y_i += Δξ_i[1]
 *      θ_i += Δξ_i[2]
 * Sau đó chuẩn hóa góc:
 *      θ_i ← normalizeAngle(θ_i)
 * để đảm bảo góc luôn nằm trong miền hợp lệ.
 */

#ifndef SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H
#define SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H

#include "pose_graph.h"
#include "matrix.h"
#include "jacobian.h"

#include <vector>
#include <stdexcept>

namespace slam {

class GaussNewtonSolver {
public:
    /**
     * @brief Thực hiện một số vòng lặp Gauss-Newton cố địnhtrên Pose Graph.
     * Hàm sẽ tối ưu trực tiếp (in-place) pose của tất cả các node trong đồ thị. 
     * Node 0 được cố định để loại bỏ hiện tượng gauge freedom.
     * @param graph Pose Graph cần tối ưu hóa. Các pose của node sẽ được cập nhật trực tiếp.
     * @param max_iterations Số vòng lặp Gauss-Newton. Mặc định: 10 vòng lặp.
     */
    static void solve(PoseGraph2D& graph, int max_iterations = 10);
};

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_GAUSS_NEWTON_SOLVER_H
