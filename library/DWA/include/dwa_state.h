#ifndef DWA_STATE_H
#define DWA_STATE_H

/**
 * @struct DWAState
 * @brief Trạng thái đầy đủ của robot trong không gian 2D
 *
 *   State vector: s = [x, y, θ, v, w]^T
 *     x     — vị trí theo trục X [m]
 *     y     — vị trí theo trục Y [m]
 *     theta — góc hướng (yaw) [rad], quy ước: 0 = hướng dương trục X
 *     v     — vận tốc tịnh tiến hiện tại [m/s]
 *     w     — vận tốc góc hiện tại [rad/s]
 */
struct DWAState {
  double x;      // vị trí X [m]
  double y;      // vị trí Y [m]
  double theta;  // góc hướng [rad]
  double v;      // vận tốc tịnh tiến [m/s]
  double w;      // vận tốc góc [rad/s]

  DWAState() : x(0.0), y(0.0), theta(0.0), v(0.0), w(0.0) {}

  DWAState(double x_, double y_, double theta_, double v_, double w_)
    : x(x_), y(y_), theta(theta_), v(v_), w(w_) {}
};

#endif  // DWA_STATE_H
