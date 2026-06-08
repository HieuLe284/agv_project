#include "matrix.h"
// ═══════════════════════════════════════════════════════════════════════════
//  Mat3 implementation
// ═══════════════════════════════════════════════════════════════════════════

slam::Mat3::Mat3() {
    std::memset(d, 0, sizeof(d));
}

slam::Mat3::Mat3(double diag) {
    std::memset(d, 0, sizeof(d));
    d[0][0] = d[1][1] = d[2][2] = diag;
}

double& slam::Mat3::operator()(int r, int c) {
    return d[r][c];
}

double slam::Mat3::operator()(int r, int c) const {
    return d[r][c];
}

slam::Mat3 slam::Mat3::mul(const slam::Mat3& A, const slam::Mat3& B) {
    slam::Mat3 C;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C.d[i][j] += A.d[i][k] * B.d[k][j];
    return C;
}

slam::Mat3 slam::Mat3::T() const {
    slam::Mat3 R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R.d[i][j] = d[j][i];
    return R;
}

slam::Mat3 slam::Mat3::AtOmegaB(const slam::Mat3& A, const slam::Mat3& Omega, const slam::Mat3& B) {
    return mul(mul(A.T(), Omega), B);
}

void slam::Mat3::vecMul(const slam::Mat3& M, const double x[3], double y[3]) {
    for (int i = 0; i < 3; ++i) {
        y[i] = 0.0;
        for (int j = 0; j < 3; ++j)
            y[i] += M.d[i][j] * x[j];
    }
}

slam::Mat3& slam::Mat3::operator+=(const slam::Mat3& rhs) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            d[i][j] += rhs.d[i][j];
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
//  MatrixX implementation
// ═══════════════════════════════════════════════════════════════════════════

slam::MatrixX::MatrixX() : rows(0), cols(0) {}

slam::MatrixX::MatrixX(int r, int c, double fill)
    : rows(r), cols(c), data(r * c, fill) {}

double& slam::MatrixX::at(int r, int c) {
    return data[r * cols + c];
}

double slam::MatrixX::at(int r, int c) const {
    return data[r * cols + c];
}

void slam::MatrixX::addBlock(int bi, int bj, const slam::Mat3& M) {
    int r0 = bi * 3, c0 = bj * 3;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            at(r0 + r, c0 + c) += M.d[r][c];
}

void slam::MatrixX::setZero() {
    std::fill(data.begin(), data.end(), 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Khử Gauss với phép xoay trục một phần
// ═══════════════════════════════════════════════════════════════════════════

std::vector<double> slam::solveLinearSystem(slam::MatrixX H, std::vector<double> rhs) {
    const int N = H.rows;
    // Tạo ma trận bổ xung (augmented matrix) [H | rhs]
    std::vector<int> piv(N);
    for (int i = 0; i < N; ++i) piv[i] = i;

    for (int col = 0; col < N; ++col) {
        // Tìm điểm xoay trong cột, hàng..N-1
        int maxRow = col;
        double maxVal = std::fabs(H.at(col, col));
        for (int row = col + 1; row < N; ++row) {
            if (std::fabs(H.at(row, col)) > maxVal) {
                maxVal = std::fabs(H.at(row, col));
                maxRow = row;
            }
        }
        if (maxVal < 1e-12)
            throw std::runtime_error("[GaussSolver] Singular matrix in H·Δξ=-b");

        // Đổi hàng (maxRow) <-> cho cột (col)
        if (maxRow != col) {
            for (int j = 0; j < N; ++j)
                std::swap(H.at(col, j), H.at(maxRow, j));
            std::swap(rhs[col], rhs[maxRow]);
        }

        // Khử các phần tử phía dưới đường chéo chính
        for (int row = col + 1; row < N; ++row) {
            double factor = H.at(row, col) / H.at(col, col);
            for (int j = col; j < N; ++j)
                H.at(row, j) -= factor * H.at(col, j);
            rhs[row] -= factor * rhs[col];
        }
    }

    // Giải ngược từ dưới lên
    std::vector<double> x(N, 0.0);
    for (int i = N - 1; i >= 0; --i) {
        x[i] = rhs[i];
        for (int j = i + 1; j < N; ++j)
            x[i] -= H.at(i, j) * x[j];
        x[i] /= H.at(i, i);
    }
    return x;
}