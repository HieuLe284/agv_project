#ifndef DWA_CONFIG_H
#define DWA_CONFIG_H

/**
 * @struct DWAConfig
 * @brief Toàn bộ tham số cấu hình cho thuật toán Dynamic Window Approach
 *
 * Tham khảo: Fox, Burgard, Thrun — "The Dynamic Window Approach to Collision Avoidance" (1997)
 */
struct DWAConfig {
  // ================================================================
  //  Robot Kinematic Limits
  // ================================================================
  double v_max;    // Vmax: vận tốc tịnh tiến tối đa [m/s]
  double v_min;    // Vmin: vận tốc tịnh tiến tối thiểu [m/s]
  double w_max;    // Wmax: vận tốc góc tối đa [rad/s]
  
  // Gia tốc dùng để tạo Dynamic Window (Vd)
  double a_v_max;  // a_v: gia tốc tịnh tiến tối đa [m/s²]
  double a_w_max;  // a_w: gia tốc góc tối đa [rad/s²]

  // Gia tốc phanh (dùng để kiểm tra Admissible Velocities - Va)
  double v_dot_b;  // v_b: gia tốc phanh tịnh tiến [m/s²]
  double w_dot_b;  // w_b: gia tốc phanh góc [rad/s²]

  // ================================================================
  //  Robot Geometry
  // ================================================================
  double robot_radius;  // r: bán kính an toàn tối thiểu [m] (để tính khoảng cách đường tròn)

  // ================================================================
  //  Simulation Parameters
  // ================================================================
  double dt;        // Δt: bước thời gian tạo Dynamic Window [s]

  // ================================================================
  //  Sampling Resolution
  // ================================================================
  int v_samples;  // Nv: số lượng mẫu vận tốc tịnh tiến
  int w_samples;  // Nw: số lượng mẫu vận tốc góc

  // ================================================================
  //  Objective Function Weights
  //  G(v,w) = α·heading(v, w) + β·clearance(v, w) + γ·velocity(v, w)
  // ================================================================
  double alpha;  // α: trọng số cho heading (hướng tới mục tiêu)
  double beta;   // β: trọng số cho clearance (khoảng cách vật cản)
  double gamma;  // γ: trọng số cho velocity (khuyến khích tốc độ)

  // ================================================================
  //  Sensor
  // ================================================================
  double sensor_max_range;  // dmax: tầm đo tối đa của LiDAR [m]

  // Default constructor — tham số đã chỉnh cho AGV nhỏ
  DWAConfig()
    : v_max(0.30),         // Vmax: vận tốc tịnh tiến tối đa [m/s]
      v_min(-0.10),        // Hạn chế đi lùi (chỉ dùng emergency)
      w_max(1.5),          // Tăng w_max để xoay linh hoạt hơn
      a_v_max(2.0),        // Tăng gia tốc → Dynamic Window rộng hơn
      a_w_max(5.0),        // Tăng gia tốc góc
      v_dot_b(1.5),
      w_dot_b(3.0),
      // [FIX]: Phục hồi robot_radius về 0.15m đúng với thực tế vật lý.
      // Khi để 0.10m, robot nghĩ nó nhỏ hơn thực tế nên nó đi cắt góc → đâm tường.
      robot_radius(0.15),  // Vùng an toàn robot (m)
      dt(0.10),
      v_samples(20),
      w_samples(41),
      // Weights cho chế độ BÌNH THƯỜNG (không nguy hiểm).
      // Khi nguy hiểm, cơ chế Preemptive Escape trong dwa_planner.cpp
      // sẽ override và trả về lệnh xoay thoát trực tiếp.
      alpha(0.25),          // Heading: bám theo A* path
      beta(0.3),           // Clearance: né tường từ xa (kết hợp D_normalize=3.0m)
      gamma(0.25),
      sensor_max_range(8.0) {}
};

#endif  // DWA_CONFIG_H
