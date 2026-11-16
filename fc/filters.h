// filters.h — цифрові фільтри для контролера
#pragma once
#include "fc_math.h"

namespace fc {

// Біквадратний фільтр нижніх частот 2-го порядку (Баттерворт).
// Навіщо: шум і вібрації гіроскопа проходять через P/D регулятора кутової швидкості прямо в мотори —
// мотори «смикаються» сотні разів на секунду, на кожному смиканні розкручують і гальмують важкий пропелер
// (енергія J·ω²/2). Це гріє мотори й з'їдає батарею. Фільтр прибирає цей шум, а повільні рухи дрона
// (до кількох Гц) пропускає майже без затримки.
class Biquad {
public:
    void setup(float cutoff_hz, float sample_hz) {
        if (cutoff_hz <= 0 || cutoff_hz >= 0.45f * sample_hz) { pass_ = true; return; }
        pass_ = false;
        const float w = std::tan(kPi * cutoff_hz / sample_hz);
        const float q = 0.70710678f;
        const float n = 1.0f / (1 + w / q + w * w);
        b0_ = w * w * n; b1_ = 2 * b0_; b2_ = b0_;
        a1_ = 2 * (w * w - 1) * n;
        a2_ = (1 - w / q + w * w) * n;
        init_ = false;
    }
    float apply(float x) {
        if (pass_) return x;
        if (!init_) { x1_ = x2_ = x; y1_ = y2_ = x; init_ = true; }
        float y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_; x1_ = x; y2_ = y1_; y1_ = y;
        return y;
    }
private:
    float b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0, x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
    bool pass_ = true, init_ = false;
};

struct Biquad3 {
    Biquad f[3];
    void setup(float hz, float fs) { for (auto& b : f) b.setup(hz, fs); }
    Vec3 apply(const Vec3& v) { return {f[0].apply(v.x), f[1].apply(v.y), f[2].apply(v.z)}; }
};

}  // namespace fc
