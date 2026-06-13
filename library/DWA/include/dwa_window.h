#ifndef DWA_WINDOW_H
#define DWA_WINDOW_H

#include "library/DWA/include/dwa_config.h"
#include "library/DWA/include/dwa_state.h"

/**
 * @class DynamicWindow
 * @brief Tính không gian vận tốc khả thi (Feasible Velocity Space)
 *
 * ================================================================
 *  TOÁN HỌC — DYNAMIC WINDOW
 * ================================================================
 *
 * Không gian vận tốc động học (Dynamic Window):
 *   Vd = { (v,w) | v ∈ [v_cur - a_v·Δt,  v_cur + a_v·Δt]
 *                  w ∈ [w_cur - a_w·Δt,  w_cur + a_w·Δt] }
 *
 * Không gian vận tốc tĩnh học (Kinematic Limits):
 *   Vs = { (v,w) | v ∈ [v_min, v_max],  w ∈ [-w_max, w_max] }
 *
 * Không gian khả thi (Feasible):
 *   Vf = Vd ∩ Vs
 *
 * Kết quả lưu vào [v_low, v_high] × [w_low, w_high]
 * @note thuật toán này tính vận tốc khả thi tức là tìm ra một hình
 * chữ nhật chứa những cặp vận tốc (v,w) mà robot có thể đạt được trong
 * khoảng thời gian dt
 */
class DynamicWindow {
 public:
  double v_low;   // v_min của Vf [m/s]
  double v_high;  // v_max của Vf [m/s]
  double w_low;   // w_min của Vf [rad/s]
  double w_high;  // w_max của Vf [rad/s]

  DynamicWindow() : v_low(0.0), v_high(0.0), w_low(0.0), w_high(0.0) {}

  /**
   * @brief Tính Vf = Vd ∩ Vs dựa trên trạng thái và cấu hình robot
   * @param state  Trạng thái hiện tại s = (x,y,θ,v,w)
   * @param config Cấu hình DWA (v_max, a_v_max, dt, ...)
   */
  void compute(const DWAState& state, const DWAConfig& config);
};

#endif  // DWA_WINDOW_H
