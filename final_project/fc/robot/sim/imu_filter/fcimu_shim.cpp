// C shim so Python (ctypes) runs the exact driver header. Build:
//   c++ -O2 -std=c++17 -shared -fPIC -I<include> fcimu_shim.cpp -o libfcimu.so
#include "rt/fc_imu_filter.hpp"
extern "C" {
struct FcParamsC { double kp, ki, quiet_rate, spike_thresh, freeze_s, kp_frozen; };
void *fcimu_new(const FcParamsC *pc) {
  fc_imu::FilterParams p;
  if (pc) { p.kp = pc->kp; p.ki = pc->ki; p.quiet_rate = pc->quiet_rate; p.spike_thresh = pc->spike_thresh; p.freeze_s = pc->freeze_s; p.kp_frozen = pc->kp_frozen; }
  return new fc_imu::AttitudeFilter(p);
}
void fcimu_free(void *h) { delete static_cast<fc_imu::AttitudeFilter *>(h); }
// n steps. acc_new[i]!=0 -> set_accel(acc[i]) before the gyro update. flags: 1 frozen, 2 despiked, 4 quiet, 8 init
void fcimu_batch(void *h, int n, const double *dt, const double *gyro, const double *acc, const int *acc_new,
                 double *q_xyzw, double *rate, int *flags) {
  auto *f = static_cast<fc_imu::AttitudeFilter *>(h);
  for (int i = 0; i < n; i++) {
    if (acc_new[i]) f->set_accel(acc + 3 * i);
    f->update_gyro(gyro + 3 * i, dt[i]);
    f->quat_xyzw(q_xyzw + 4 * i);
    f->rate(rate + 3 * i);
    flags[i] = (f->frozen() ? 1 : 0) | (f->despiked() ? 2 : 0) | (f->quiet() ? 4 : 0) | (f->initialized() ? 8 : 0);
  }
}
void fcimu_stats(void *h, double *out6) {
  auto &s = static_cast<fc_imu::AttitudeFilter *>(h)->stats();
  out6[0] = s.samples; out6[1] = s.despiked; out6[2] = s.frozen_samples; out6[3] = s.accel_frozen_samples; out6[4] = s.accel_skipped; out6[5] = s.longest_freeze_s;
}
}
