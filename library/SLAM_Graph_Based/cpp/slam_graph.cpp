#include "slam_graph.h"

void slam::SlamGraph::init() {
  pose_graph.nodes.clear();
  pose_graph.edges.clear();
  loop_closure_count_ = 0;
  new_loop_this_step_ = false;

  Node2D anchor(0.0, 0.0, 0.0);
  pose_graph.nodes.push_back(anchor);
}

void slam::SlamGraph::setMapBuilder(MapBuilder *mb) { 
  map_builder_ = mb; 
}

int slam::SlamGraph::addOdometryNode(double x, double y, double theta,
                               const std::vector<double> &ranges,
                               double angle_min, double angle_inc) {
  new_loop_this_step_ = false;

  if (pose_graph.nodes.empty()) {
    init();
  }
  const int prev_idx = pose_graph.numNodes() - 1;
  const Node2D &prev = pose_graph.nodes[prev_idx];

  // So sánh với node trước bằng khoảng cách Eculidean
  double dx = x - prev.x;
  double dy = y - prev.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double dtheta = std::fabs(normalizeAngle(theta - prev.theta));
  // Kiểm tra ngưỡng từ Lidar => Robot
  if (dist < min_travel_dist && dtheta < min_travel_angle)
    return -1;

  // Thêm node 2D mới vào pose_graph.nodes
  Node2D node(x, y, theta);
  node.scan_ranges = ranges;
  node.scan_angle_min = angle_min;
  node.scan_angle_increment = angle_inc;
  pose_graph.nodes.push_back(node);
  const int new_idx = pose_graph.numNodes() - 1;

  //  Tính toán quãng đường di chuyển theo z_{prev → new} 
  double ci = std::cos(prev.theta);
  double si = std::sin(prev.theta);
  // δt = R_{prev}^T · (t_new − t_prev)
  double zx = ci * dx + si * dy;
  double zy = -si * dx + ci * dy;
  double zt = normalizeAngle(theta - prev.theta);

  // Thêm edge odom với information matrix Ω_ij = diag(odom_omega_xy, odom_omega_xy, odom_omega_theta)
  pose_graph.addEdge(prev_idx, new_idx, zx, zy, zt, odom_omega_xy,
                     odom_omega_xy, odom_omega_theta,
                     /*loop=*/false);

  return new_idx;
}

int slam::SlamGraph::addLoopClosures(int new_idx) {
  if (new_idx < 0)  
    return -1;
  // Duyệt các node cũ, tính scan correlation
  int matched = loop_detector_.detect(pose_graph, new_idx);
  
  // Nếu match được thì thêm edge loop-closure với Ω_loop lớn hơn
  if (matched >= 0) {
    ++loop_closure_count_;
    new_loop_this_step_ = true;
  }
  return matched;
}

bool slam::SlamGraph::optimizeIfNeeded() {
  if (!new_loop_this_step_)
    return false;

  // Sử dụng phương pháp Gauss-Newton để tối ưu hóa pose graph nếu có loop closure mới
  GaussNewtonSolver::solve(pose_graph, gn_iterations);
  new_loop_this_step_ = false;
  return true;
}