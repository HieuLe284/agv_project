/**
 * @file jacobian.h
 * @brief Jacobian giải tích cho ràng buộc Pose-Pose 2D trong Graph-Based SLAM
 * ── Pose Representation (2D SE(2)) ──────────────────────────────────────────
 *   Pose của robot tại node i:
 *     x_i = (x_i, y_i, θ_i)^T
 *   Ma trận biến đổi thuần nhất:
 *         ⎡ cos θ_i  -sin θ_i  x_i ⎤
 *     X_i = ⎢ sin θ_i   cos θ_i  y_i ⎥
 *         ⎣    0          0     1  ⎦
 *
 * ── Hàm sai số của cạnh (Edge Error Function) ──────────────────────────────────────────────────────
 *
 *  Cho một ràng buộc giữa node i và node j với phép đo tương đối:
 *     z_ij = (t_ij^T, θ_ij)^T 
 *  (tức là pose tương đối của node j quan sát từ hệ tọa độ của node i)
 *  Hàm sai số được định nghĩa:
 *     e_ij(x_i, x_j) = t2v( Z_ij^{-1} · X_i^{-1} · X_j )
 *   Dạng ma trận khai triển:
 *     e_ij = ⎡ R_ij^T · (R_i^T · (t_j - t_i) - t_ij) ⎤
 *            ⎣    normalize(θ_j - θ_i - θ_ij)        ⎦
 *
 * ── Jacobians ────────────────────────────────────────────────────────────────
 *
 *   The Jacobian J_ij = [A_ij | B_ij]:
 *     A_ij = ∂e_ij / ∂x_i  (3×3)
 *     B_ij = ∂e_ij / ∂x_j  (3×3)
 *
 *   Đặt: dR_i = ∂R_i^T/∂θ_i = ⎡ -sinθ_i  cosθ_i ⎤
 *                             ⎣ -cosθ_i -sinθ_i ⎦
 *   Là đạo hàm của: R_i^T theo: θ_i
 *   Khi đó:
 *   A_ij = ⎡ -R_ij^T · R_i^T     R_ij^T · dR_i · (t_j - t_i) ⎤
 *          ⎣  0    0              -1                         ⎦
 *
 *   B_ij = ⎡  R_ij^T · R_i^T     0  ⎤
 *          ⎣  0    0              1 ⎦
 */

#ifndef SLAM_GRAPH_BASED_JACOBIAN_H
#define SLAM_GRAPH_BASED_JACOBIAN_H

#include "matrix.h"
#include <cmath>
#include "library/common/math_utils.h"

namespace slam {

// ═══════════════════════════════════════════════════════════════════════════
//  Ma trận quay 2D R(θ) kích thước 2×2
// ═══════════════════════════════════════════════════════════════════════════
inline void makeRotation2D(double theta, double R[2][2]) {
    R[0][0] =  std::cos(theta);  R[0][1] = -std::sin(theta);
    R[1][0] =  std::sin(theta);  R[1][1] =  std::cos(theta);
}

// ── dR^T/dθ  (Đạo hàm của Rᵀ theo θ) ───────────────────────────────────────
// dR_i^T / dθ_i = ⎡ -sinθ  cosθ ⎤
//                 ⎣ -cosθ -sinθ ⎦
inline void makeDRotation2D(double theta, double dR[2][2]) {
    dR[0][0] = -std::sin(theta);  dR[0][1] =  std::cos(theta);
    dR[1][0] = -std::cos(theta);  dR[1][1] = -std::sin(theta);
}

// ═══════════════════════════════════════════════════════════════════════════
// Hàm tính sai số cạnh e_ij(x_i,x_j) ∈ R³
//  Inputs:
//    x{i,j}        — pose của hai node (x, y, θ)
//    z_{x,y,theta} — phép đo tương đối (translation + rotation)
//  Output:
//    e[3]          — vector sai số
// ═══════════════════════════════════════════════════════════════════════════
void computeError(
    double xi, double yi, double ti,        // pose i
    double xj, double yj, double tj,        // pose j
    double zx, double zy, double zt,        // measurement z_ij
    double e[3]);

// ═══════════════════════════════════════════════════════════════════════════
//  Jacobian giải tích  A_ij (3×3) and B_ij (3×3)
//  A_ij = ∂e_ij/∂x_i ,   B_ij = ∂e_ij/∂x_j
// ═══════════════════════════════════════════════════════════════════════════
void computeJacobians(
    double xi, double yi, double ti,        // pose i
    double xj, double yj, double tj,        // pose j
    double zx, double zy, double zt,        // measurement z_ij
    Mat3& A, Mat3& B);

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_JACOBIAN_H
