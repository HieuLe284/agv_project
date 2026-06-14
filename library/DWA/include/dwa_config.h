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
  //Bán kính của robot (0.15m). Dùng để tính toán khoảng cách an toàn tránh va chạm.
  double robot_radius;  // r: bán kính an toàn tối thiểu [m] (để tính khoảng cách đường tròn)

  // ================================================================
  //  Simulation Parameters
  // ================================================================
  // Bước thời gian mô phỏng quỹ đạo.
  double dt;        // Δt: bước thời gian tạo Dynamic Window [s]

  // ================================================================
  //  Sampling Resolution
  // ================================================================
  // Số lượng mẫu vận tốc để kiểm tra. Càng nhiều mẫu thì tính toán càng mịn nhưng tốn CPU hơn.
  int v_samples;  // Nv: số lượng mẫu vận tốc tịnh tiến
  int w_samples;  // Nw: số lượng mẫu vận tốc góc

  // ================================================================
  //  Objective Function Weights
  //  G(v,w) = α·heading(v, w) + β·clearance(v, w) + γ·velocity(v, w)
  // ================================================================
  //Trọng số ưu tiên việc hướng về mục tiêu (điểm tiếp theo của A*).
  double alpha;  // α: trọng số cho heading (hướng tới mục tiêu)
  // Trọng số ưu tiên việc tránh xa vật cản.
  double beta;   // β: trọng số cho clearance (khoảng cách vật cản)
  // Trọng số ưu tiên việc duy trì tốc độ cao.
  double gamma;  // γ: trọng số cho velocity (khuyến khích tốc độ)

  // ================================================================
  //  Sensor
  // ================================================================
  // Tầm xa tối đa của LiDAR mà thuật toán DWA sẽ xét đến để né vật cản.
  double sensor_max_range;  // dmax: tầm đo tối đa của LiDAR [m]

  // ================================================================
  //  DWA Planner
  // ================================================================
  double CROSS_TRACK_THRESH;    // Ngưỡng sai số lệch đường => Nếu lệch sẽ bẻ lái về
  double LOOKAHEAD_DIST;        // Khoảng cách nhìn trước
  // Quyết định robot sẽ "nhìn" xa bao nhiêu trên quãng đường phía trước để hướng theo.
 
  double LOOKAHEAD_CORR;        // Hệ số hiệu chỉnh bẻ lái
  // Điều khiển độ "gắt" khi robot bẻ lái để quay lại đường nếu bị lệch.

  double ESCAPE_TRIGGER_DIST;   // Ngưỡng kích hoạt thoát hiểm

  // ================================================================
  //  DWA scoring
  // ================================================================
  double D_normalize;  // Chuẩn hóa khoảng cách né vật cản
  // Xác định tầm xa mà robot bắt đầu "cảm thấy" lo lắng về vật cản.

  // Default constructor — tham số đã chỉnh cho AGV nhỏ
  DWAConfig()
  : v_max(0.30),         // Vmax: vận tốc tịnh tiến tối đa [m/s]
    v_min(-0.10),        // Hạn chế đi lùi (chỉ dùng emergency)
    w_max(2.1),          // Tăng w_max để xoay linh hoạt hơn
    a_v_max(3.0),        // Tăng gia tốc → Dynamic Window rộng hơn
    a_w_max(7.0),        // Tăng gia tốc góc
    v_dot_b(1.5),
    w_dot_b(3.0),
    robot_radius(0.15),  // Vùng an toàn robot (m)
    dt(0.10),
    v_samples(30),
    w_samples(50),
    alpha(0.55),          // Heading: tăng để giữ thẳng hướng tốt hơn
    beta(0.25),            // Clearance: giảm để clearance không kéo robot sang bên
    gamma(0.20),
    sensor_max_range(8.0),
    CROSS_TRACK_THRESH(0.20),  // Tăng từ 0.10→0.20m: chỉ sửa khi lệch rõ rệt (giảm false trigger)
    LOOKAHEAD_DIST(0.4),
    LOOKAHEAD_CORR(1.0),       // Tăng từ 0.7→1.0: giảm độ gắt của correction khi lệch đường
    ESCAPE_TRIGGER_DIST(0.25),
    D_normalize(2.5) {}        // Giảm từ 4.0→2.5m: chỉ lo ngại vật cản trong 2.5m
    // : v_max(0.30),         // Vmax: vận tốc tịnh tiến tối đa [m/s]
    //   v_min(-0.10),        // Hạn chế đi lùi (chỉ dùng emergency)
    //   w_max(1.85),          // Tăng w_max để xoay linh hoạt hơn
    //   a_v_max(2.0),        // Tăng gia tốc → Dynamic Window rộng hơn
    //   a_w_max(5.0),        // Tăng gia tốc góc
    //   v_dot_b(1.5),
    //   w_dot_b(3.0),
    //   robot_radius(0.15),  // Vùng an toàn robot (m)
    //   dt(0.10),
    //   v_samples(20),
    //   w_samples(41),
    //   alpha(0.25),          // Heading: bám theo A* path
    //   beta(0.3),           // Clearance: né tường từ xa (kết hợp D_normalize=3.0m)
    //   gamma(0.25),
    //   sensor_max_range(8.0),
    //   CROSS_TRACK_THRESH(0.10),
    //   LOOKAHEAD_DIST(0.4),
    //   LOOKAHEAD_CORR(0.7),
    //   ESCAPE_TRIGGER_DIST(0.25),
    //   D_normalize(3.0) {}
};

#endif  // DWA_CONFIG_H
