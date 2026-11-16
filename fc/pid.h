// pid.h — PID з анти-віндапом, D по вимірюванню і фільтром на D
#pragma once
#include "fc_math.h"

namespace fc {

struct PidGains {
    float kp = 0, ki = 0, kd = 0;
    float i_limit = 1e9f;    // межа інтегральної складової (в одиницях виходу)
    float out_limit = 1e9f;  // межа виходу
    float d_lpf_hz = 0;      // 0 = без фільтра
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidGains& g) : g_(g) {}
    void setGains(const PidGains& g) { g_ = g; }
    const PidGains& gains() const { return g_; }

    void reset(float i_preload = 0) { i_ = i_preload; first_ = true; d_ = 0; }

    // integrate=false заморожує інтеграл (коли мотори в насиченні)
    float update(float setpoint, float meas, float dt, bool integrate = true) {
        float e = setpoint - meas;
        if (integrate && g_.ki != 0) i_ = clampf(i_ + g_.ki * e * dt, -g_.i_limit, g_.i_limit);
        float d_raw = first_ ? 0 : -(meas - prev_) / dt;  // D по вимірюванню — без «удару» при зміні уставки
        first_ = false;
        prev_ = meas;
        if (g_.d_lpf_hz > 0) {
            float rc = 1.0f / (2 * kPi * g_.d_lpf_hz);
            d_ += (d_raw - d_) * (dt / (rc + dt));
        } else {
            d_ = d_raw;
        }
        p_term_ = g_.kp * e;
        d_term_ = g_.kd * d_;
        return clampf(p_term_ + i_ + d_term_, -g_.out_limit, g_.out_limit);
    }
    float integral() const { return i_; }
    void decay(float k) { i_ *= k; }
    void shiftIntegral(float d) { i_ = clampf(i_ + d, -g_.i_limit, g_.i_limit); }  // повільне «стікання» інтеграла (наприклад, на землі)
private:
    PidGains g_;
    float i_ = 0, prev_ = 0, d_ = 0, p_term_ = 0, d_term_ = 0;
    bool first_ = true;
};

}  // namespace fc
