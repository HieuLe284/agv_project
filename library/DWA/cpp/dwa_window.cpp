#include "library/DWA/include/dwa_window.h"

#include <algorithm>

void DynamicWindow::compute(const DWAState& state, const DWAConfig& config) {
  // ================================================================
  //  Tính Dynamic Window Vd — giới hạn bởi gia tốc trong 1 bước Δt
  // ================================================================
  // Công thức không gian giới hạn tĩnh (Kinematic / Robot Specification Space)
  // V_s = {(v,ω)∣ v ∈ [v_min,v_max], ω ∈ [−ω_max,ω_max]}
  //
  //  Δv = a_v_max · Δt   (biến thiên vận tốc tịnh tiến tối đa)
  //  Δw = a_w_max · Δt   (biến thiên vận tốc góc tối đa)
  //
  // Chú ý: AGV không thể thay đổi vận tốc nhảy cóc được do bị kìm hãm bởi
  // Định luật 2 Newton F = ma nên cần có khoảng thời gian dt để thay đổi vận tốc
  // Và trong lý thuyết động học vi phân, quỹ đạo liên tục của robot tại thời điểm tiếp
  // theo (t + Δt) bị giới hạn quanh trạng thái hiện tại (v_t, w_t) nên ta có công thức DWA:
  // V_d = {(v,ω) ∣ v ∈ [v(t)−v˙_max(Δt),v(t)+v˙_max(Δt)], ω ∈ [ω(t)−ω˙_max(Δt),ω(t)+ω˙_max(Δt)]}
  //
  //  Vd_v = [v_cur - Δv,  v_cur + Δv]
  //  Vd_w = [w_cur - Δw,  w_cur + Δw]

  const double delta_v = config.a_v_max * config.dt;
  const double delta_w = config.a_w_max * config.dt;

  const double vd_low  = state.v - delta_v;
  const double vd_high = state.v + delta_v;
  const double wd_low  = state.w - delta_w;
  const double wd_high = state.w + delta_w;

  // ================================================================
  //  Giao với Vs — giới hạn vật lý của robot
  //  Vf = Vd ∩ Vs
  // Trục tịnh tiến: 
  //     v ∈ [max(v_min,v_cur−a_vΔt),min(v_max,v_cur+a_vΔt)]
  // hay v_f ∈ [max(v_min, vd_low),  min(v_max, vd_high)]
  // Trục góc: 
  //     ω ∈ [max(−ω_max,ω_cur−a_wΔt),min(ω_max,ω_cur+a_wΔt)]
  // hay w_f ∈ [max(-w_max, wd_low), min(w_max, wd_high)]
  // ================================================================

  v_low  = std::max(config.v_min, vd_low);
  v_high = std::min(config.v_max, vd_high);
  w_low  = std::max(-config.w_max, wd_low);
  w_high = std::min(config.w_max, wd_high);
}
