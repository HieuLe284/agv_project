#include "loop_closure_detector.h"

int slam::LoopClosureDetector::detect(PoseGraph2D& graph, int new_idx) {
    const int N = graph.numNodes();
    if (new_idx < min_node_gap) return -1;

    const Node2D& nj = graph.nodes[new_idx];
    if (nj.scan_ranges.empty()) return -1;

    int    best_id    = -1;
    double best_score = correlation_threshold;

    for (int i = 0; i < new_idx - min_node_gap; ++i) {
        const Node2D& ni = graph.nodes[i];
        if (ni.scan_ranges.empty()) continue;

        // ── Stage 1: Proximity filter ─────────────────────────────────
        // ∥t_j​ − t_i​∥ = sqrt((x_j​−x_i​)^2 + (y_j​−y_i​)^2)
        double dx = nj.x - ni.x;
        double dy = nj.y - ni.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > dist_threshold) continue;

        // |θ| = |θ_j - θ_i|
        double dtheta = std::fabs(normalizeAngle(nj.theta - ni.theta));
        if (dtheta > angle_threshold) continue;

        // ── Stage 2: Scan correlation ─────────────────────────────────
        double score = scanCorrelation(ni.scan_ranges, nj.scan_ranges);
        if (score > best_score) {
            best_score = score;
            best_id    = i;
        }
    }

    if (best_id < 0) return -1;

    // ── Stage 3: Tính toán relative pose z_ij từ graph poses hiện tại ──
    // e_ij​(x) = (R_ij^T​*(R_i^T*​(t_j​−t_i​)−t_ij​)θ_j​−θ_i​−θ_ij​​)
    // Trong đó:
    // t_i,t_j: vị trí hai node
    // R_i: ma trận quay của node i
    // t_ij: measurement lưu trong edge
    // θ_ij: góc tương đối lưu trong edge
    const Node2D& ni = graph.nodes[best_id];

    // R_i^T = Ma trận quay tại θ_i
    double ci = std::cos(ni.theta);
    double si = std::sin(ni.theta);
    double dtx = nj.x - ni.x;
    double dty = nj.y - ni.y;

    // δt_ij = R_i^T · (t_j − t_i)
    double zx =  ci * dtx + si * dty;
    double zy = -si * dtx + ci * dty;
    double zt =  normalizeAngle(nj.theta - ni.theta);

    // Tạo loop-closure edge j → best_id
    graph.addEdge(best_id, new_idx,
                  zx, zy, zt,
                  omega_xy, omega_xy, omega_theta,
                  /*loop=*/true);

    return best_id;
}

double slam::LoopClosureDetector::scanCorrelation(const std::vector<double>& a,
                                             const std::vector<double>& b)
{
    size_t n = std::min(a.size(), b.size());
    if (n == 0) return 0.0;

    double dot = 0.0, na = 0.0, nb = 0.0;
    for (size_t k = 0; k < n; ++k) {
        double ra = (std::isfinite(a[k]) && a[k] > 0.0) ? a[k] : 0.0; // Σ r_i
        double rb = (std::isfinite(b[k]) && b[k] > 0.0) ? b[k] : 0.0; // Σ r_j
        dot += ra * rb; // Σ(r_i*r_j)
        na  += ra * ra; // Σ r_i^2
        nb  += rb * rb; // Σ r_j^2
    }
    double denom = std::sqrt(na) * std::sqrt(nb); // sqrt(Σ{r_i^2}) *  sqrt(Σ{r_j^2})
    return (denom > 1e-9) ? (dot / denom) : 0.0; // C(a,b)
}
