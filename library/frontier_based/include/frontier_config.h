#ifndef FRONTIER_CONFIG_H
#define FRONTIER_CONFIG_H

/**
 * @struct FrontierConfig
 * @brief Tham số cấu hình cho thuật toán Frontier-Based Exploration
 *
 * Tham khảo: Yamauchi — "A Frontier-Based Approach for Autonomous Exploration" (1997)
 */
struct FrontierConfig {
  // ================================================================
  //  Frontier Detection Parameters
  // ================================================================
  int min_frontier_size;  // Số cell tối thiểu của một frontier region hợp lệ
  double max_goal_dist;   // Khoảng cách tối đa (m) để xét một frontier

  // ================================================================
  //  Frontier Selection Cost Function
  //  cost_i = w_dist * dist(robot, centroid_i) + w_size * (1 / size_i)
  // ================================================================
  double w_dist;  // Trọng số khoảng cách (ưu tiên frontier gần)
  double w_size;  // Trọng số kích thước (ưu tiên frontier lớn)

  // ================================================================
  //  Navigation Parameters
  // ================================================================
  double v_explore;         // Vận tốc tịnh tiến khi thám hiểm [m/s]
  double w_max;             // Vận tốc góc tối đa [rad/s]
  double k_w;               // Hệ số khuếch đại góc (P-controller)
  double goal_tolerance;    // Khoảng cách để coi đã đến goal [m]
  double heading_threshold; // Ngưỡng góc để chuyển từ quay → đi thẳng [rad]

  // ================================================================
  //  Default Constructor — tham số cho AGV nhỏ
  // ================================================================
  FrontierConfig()
    : min_frontier_size(5),       
      max_goal_dist(50.0),        
      w_dist(0.7),                
      w_size(0.3),                
      v_explore(0.20),            
      w_max(1.5),                 
      k_w(1.5),                   
      goal_tolerance(0.35),       
      heading_threshold(0.45) {}  
};

#endif  // FRONTIER_CONFIG_H
