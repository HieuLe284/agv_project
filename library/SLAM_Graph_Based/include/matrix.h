/**
 * @file matrix.h
 * Cài đặt lớp ma trận tổng quát kích thước N×N được sử dụng để xây dựng:
 *   - Ma trận thông tin toàn cục H (xấp xỉ Hessian)
 *   - Vector hệ số b
 * trong bộ tối ưu Pose Graph sử dụng thuật toán Gauss-Newton.
 * Hệ phương trình tuyến tính cần giải tại mỗi vòng lặp:
 *      H · Δξ = -b
 * trong đó:
 *      H ∈ R^(3N×3N)
 *      b ∈ R^(3N)
 * với N là số node trong Pose Graph.
 */

#ifndef SLAM_GRAPH_BASED_MATRIX_H
#define SLAM_GRAPH_BASED_MATRIX_H

#include <vector>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace slam {

// ═══════════════════════════════════════════════════════════════════════════
//  Ma trận cố định kích thước 3×3
//  Được sử dụng cho:
//      A_ij   (Jacobian theo node i)
//      B_ij   (Jacobian theo node j)
//      Ω_ij   (Information Matrix)
// ═══════════════════════════════════════════════════════════════════════════
struct Mat3 {
    double d[3][3];

    Mat3(); // Khởi tạo ma trận 0

    explicit Mat3(double diag); // Khởi tạo ma trận đường chéo:

    // Truy cập phần tử ma trận
    double& operator()(int r, int c);
    double  operator()(int r, int c) const;

    // Nhân ma trận C = A * B  (3×3)
    static Mat3 mul(const Mat3& A, const Mat3& B);

    // Chuyển vị ma trận A^T
    Mat3 T() const;

    // Tính: A^T * Ω * B  ( được sử dụng khi xây dựng Hessian H )
    static Mat3 AtOmegaB(const Mat3& A, const Mat3& Omega, const Mat3& B);

    // Nhân ma trận 3×3 với vector 3 chiều: y = M * x
    static void vecMul(const Mat3& M, const double x[3], double y[3]);

    // Cộng ma trận tại chỗ: this += rhs
    Mat3& operator+=(const Mat3& rhs);
};

// ═══════════════════════════════════════════════════════════════════════════
//  Ma trận đặc kích thước N×N
//  Được sử dụng để lưu:
//      H ∈ R^(3N×3N)
//  và hỗ trợ xây dựng:
//      b ∈ R^(3N)
//  trong quá trình tối ưu Pose Graph.
// ═══════════════════════════════════════════════════════════════════════════
class MatrixX {
public:
    int rows, cols;
    std::vector<double> data; // Dữ liệu ma trận lưu theo dạng tuyến tính

    MatrixX();
    MatrixX(int r, int c, double fill = 0.0);

    // Truy cập phần tử ma trận
    double& at(int r, int c);
    double  at(int r, int c) const;

    /**
     * Thêm một khối ma trận 3×3 vào Hessian:
     *      H(bi,bj) += M
     * với:
     *      M ∈ R^(3×3)
     * Đây là hàm được sử dụng khi cộng:
     *      H_ii
     *      H_ij
     *      H_ji
     *      H_jj
     * từ từng edge của Pose Graph.
     */
    void addBlock(int bi, int bj, const Mat3& M);

    void setZero(); // Đặt toàn bộ phần tử về 0
};

// ═══════════════════════════════════════════════════════════════════════════
//  Khử Gauss có chọn pivot từng phần (Gaussian Elimination with Partial Pivoting)
//  Giải hệ:
//      H · Δξ = -b
//  Kết quả: Δξ => được trả về dưới dạng vector một chiều.
//  Ném ngoại lệ std::runtime_error nếu:
//      - H suy biến (singular)
//      - H gần suy biến (near singular)
// ═══════════════════════════════════════════════════════════════════════════
std::vector<double> solveLinearSystem(MatrixX H, std::vector<double> rhs);

}  // namespace slam

#endif  // SLAM_GRAPH_BASED_MATRIX_H
