#pragma once
#include <cstdint>
#include <cmath>

struct CurrentMap {
  double K_T_EFF;        // [Nm/A] effective torque constant at output shaft
  double CURRENT_TICK_A; // [A/tick] resolution of Goal Current register
  int32_t MAX_ABS_TICK;  // clamp limit
};

inline int16_t torqueNmToGoalCurrentTick(double tau_nm, const CurrentMap& m) {
  // Convert torque [Nm] -> current [A]
  const double I = tau_nm / m.K_T_EFF;
  // Convert current [A] -> device ticks
  const double ticks = I / m.CURRENT_TICK_A;
  // Round to integer and clamp
  int32_t t = static_cast<int32_t>(std::lround(ticks));
  if (t >  m.MAX_ABS_TICK) t =  m.MAX_ABS_TICK;
  if (t < -m.MAX_ABS_TICK) t = -m.MAX_ABS_TICK;
  return static_cast<int16_t>(t);