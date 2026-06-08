#include "gauss_newton_solver.h"

void slam::GaussNewtonSolver::solve(PoseGraph2D& graph, int max_iterations) {
    const int N = graph.numNodes();
    if (N < 2) return;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // ── Step 1–2: Build H ∈ R^{3N×3N} and b ∈ R^{3N} ──────────────
        const int dim = 3 * N;
        MatrixX H(dim, dim, 0.0);
        std::vector<double> b(dim, 0.0);

        for (const auto& edge : graph.edges) {
            const int i = edge.from;
            const int j = edge.to;
            if (i < 0 || j < 0 || i >= N || j >= N) continue;

            const Node2D& ni = graph.nodes[i];
            const Node2D& nj = graph.nodes[j];

            // ── Residual e_ij ──────────────────────────────────────────
            double e[3];
            computeError(ni.x, ni.y, ni.theta,
                         nj.x, nj.y, nj.theta,
                         edge.z_x, edge.z_y, edge.z_theta, e);

            // ── Jacobians A_ij, B_ij ───────────────────────────────────
            Mat3 A, B;
            computeJacobians(ni.x, ni.y, ni.theta,
                             nj.x, nj.y, nj.theta,
                             edge.z_x, edge.z_y, edge.z_theta, A, B);

            // ── Ma trận thông tin (Information matrix) Ω_ij (from edge) ───────────────────
            Mat3 Omega;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    Omega(r, c) = edge.omega[r][c];

            // ── H block contributions ──────────────────────────────────
            // H_ii += A^T Ω A
            H.addBlock(i, i, Mat3::AtOmegaB(A, Omega, A));
            // H_ij += A^T Ω B
            H.addBlock(i, j, Mat3::AtOmegaB(A, Omega, B));
            // H_ji += B^T Ω A
            H.addBlock(j, i, Mat3::AtOmegaB(B, Omega, A));
            // H_jj += B^T Ω B
            H.addBlock(j, j, Mat3::AtOmegaB(B, Omega, B));

            // ── b block contributions ──────────────────────────────────
            // b_i += A^T * Ω * e
            double Oe[3], AtOe[3];
            Mat3::vecMul(Omega, e, Oe);
            Mat3::vecMul(A.T(), Oe, AtOe);
            for (int k = 0; k < 3; ++k) b[3*i + k] += AtOe[k];

            // b_j += B^T * Ω * e
            double BtOe[3];
            Mat3::vecMul(B.T(), Oe, BtOe);
            for (int k = 0; k < 3; ++k) b[3*j + k] += BtOe[k];
        }

        // ── Step 3: Gauge fix — thêm 10^9 bằng cách thêm đường chéo lớn  ─
        const double kAnchor = 1e9;
        H.at(0, 0) += kAnchor;
        H.at(1, 1) += kAnchor;
        H.at(2, 2) += kAnchor;

        // ── Step 4: Giải H · Δξ = -b ─────────────────────────────────
        for (auto& val : b) val = -val; // phủ định của b = -b

        std::vector<double> dx;
        try {
            dx = solveLinearSystem(H, b);
        } catch (const std::runtime_error&) {
            return; 
        }

        // ── Step 5: Update x_i ← x_i ⊕ Δξ_i ───────────────────
        for (int i = 0; i < N; ++i) {
            graph.nodes[i].x     += dx[3*i + 0];
            graph.nodes[i].y     += dx[3*i + 1];
            graph.nodes[i].theta  = normalizeAngle(
                graph.nodes[i].theta + dx[3*i + 2]);
        }
    }
}