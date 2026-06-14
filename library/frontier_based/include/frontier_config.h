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
  int min_frontier_size;  // Số cell tối thiểu của một frontier region hợp lệ. Giúp lọc các điểm nhiễu li ti trên bản đồ.
  double max_goal_dist;   // Khoảng cách tối đa (m) để xét một frontier. Robot sẽ bỏ qua các vùng biên quá xa tầm này.

  // ================================================================
  //  Frontier Selection Cost Function
  //  cost_i = w_dist * dist(robot, centroid_i) + w_size * (1 / size_i)
  // ================================================================
  double w_dist;  // Trọng số khoảng cách (ưu tiên frontier gần). 
  // Nếu lớn, robot sẽ ưu tiên thám hiểm các vùng gần nó trước.

  double w_size;  // Trọng số kích thước (ưu tiên frontier lớn).
  // Nếu lớn, robot sẽ ưu tiên các vùng biên lớn (không gian rộng chưa khám phá) hơn là các hẻm nhỏ.

  // ================================================================
  //  Navigation Parameters
  // ================================================================
  double goal_tolerance;    // Sai số khoảng cách để coi đã đến goal [m]

  // ================================================================
  //  FRONTIER EXPLORATION
  // ================================================================
  double TIMEOUT;           // time out khi giá trị frontier ở 1 vị trí quá lâu

  // ================================================================
  //  Default Constructor — tham số cho AGV nhỏ
  // ================================================================
  FrontierConfig()
    : min_frontier_size(5),       
      max_goal_dist(50.0),        
      w_dist(0.7),                
      w_size(0.3),                
      goal_tolerance(0.35),       
      TIMEOUT(30) {}  
};

#endif  // FRONTIER_CONFIG_H
