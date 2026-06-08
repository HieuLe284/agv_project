#include "pose_graph.h"

slam::Edge2D::Edge2D() {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      omega[i][j] = 0.0;
    }
  }
}
void slam::Edge2D::setOmegaDiagonal(double wx, double wy, double wt) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      omega[i][j] = 0.0;
    }
  }
  omega[0][0] = wx;
  omega[1][1] = wy;
  omega[2][2] = wt;
}
// ═══════════════════════════════════════════════════════════════════════════
//  PoseGraph2D — the complete graph G = (V, E)
// ═══════════════════════════════════════════════════════════════════════════
int slam::PoseGraph2D::numNodes() const {
  return static_cast<int>(nodes.size());
}
int slam::PoseGraph2D::numEdges() const {
  return static_cast<int>(edges.size());
}

// thêm cạnh vào odometry i → j
void slam::PoseGraph2D::addEdge(int i, int j, double zx, double zy, double zt,
                                double wx, double wy, double wt, bool loop) {
  Edge2D e;
  e.from = i;
  e.to = j;
  e.z_x = zx;
  e.z_y = zy;
  e.z_theta = zt;
  e.setOmegaDiagonal(wx, wy, wt);
  e.is_loop = loop;
  edges.push_back(e);
}