// ai/features.h — ознаки для діагностики, які рахує ШІ-співпроцесор з сирих даних (не залежать від корпусу апарата).
//  • OrderTracker — амплітуда вібрації на частотах, кратних обертам кожного мотора (0.5×, 1×, 2×): «відстеження порядків»,
//    як у вібродіагностиці машин. 1× — дисбаланс гвинта, 0.5× і широкий спектр — люфт кріплення, 2× — перекіс/вигин.
//    Мотори розрізняються за власними обертами: при різниці обертів ≳ 0.5 Гц фільтр (τ = 1 с) розділяє їх.
//  • BandSpectrum — енергія вібрації по смугах 0…fs/2 (вхід для нейромережі-класифікатора).
//  • ServoMonitor — запізнення й люфт приводу нахилу гондоли (команда проти енкодера).
//  • BatteryRls — внутрішній опір і ЕРС батареї (рекурсивні найменші квадрати з забуванням).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ai {

constexpr float kTwoPi = 6.28318530717958647692f;

// Відстеження порядків: для кожного мотора — фаза обертання φ (інтеграл обертів), демодуляція прискорення на k·φ,
// низькочастотний фільтр I/Q. Амплітуда синусоїди a·cos(kφ + ψ) після фільтра = 2·√(I² + Q²).
class OrderTracker {
public:
    static constexpr int kMotors = 4, kOrders = 3;
    static constexpr float kOrder[kOrders] = {0.5f, 1.0f, 2.0f};
    float tau_s = 1.0f;

    // Одна вибірка IMU: прискорення (м/с², будь-які осі тіла) й поточні оберти моторів (об/хв), крок dt
    void update(const float acc[3], const float rpm[kMotors], float dt) {
        const float a = std::fmin(1.0f, dt / tau_s);
        for (int m = 0; m < kMotors; ++m) {
            phase_[m] = std::fmod(phase_[m] + kTwoPi * rpm[m] / 60.0f * dt, 2.0f * kTwoPi);   // 2·2π: період і для порядку 0.5
            for (int k = 0; k < kOrders; ++k) {
                const float c = std::cos(kOrder[k] * phase_[m]), s = std::sin(kOrder[k] * phase_[m]);
                for (int ax = 0; ax < 3; ++ax) {
                    float& I = iq_[m][k][ax][0];
                    float& Q = iq_[m][k][ax][1];
                    I += a * (acc[ax] * c - I);
                    Q += a * (-acc[ax] * s - Q);
                }
            }
        }
    }
    // Амплітуда (м/с²) вібрації мотора m на порядку k (0 — 0.5×, 1 — 1×, 2 — 2×), сумарно по осях
    float amplitude(int m, int k) const {
        float e = 0;
        for (int ax = 0; ax < 3; ++ax) e += iq_[m][k][ax][0] * iq_[m][k][ax][0] + iq_[m][k][ax][1] * iq_[m][k][ax][1];
        return 2.0f * std::sqrt(e);
    }
    void reset() { *this = OrderTracker{}; }

private:
    float phase_[kMotors] = {0, 0, 0, 0};
    float iq_[kMotors][kOrders][3][2] = {};
};

// Енергія вібрації по смугах: блоки по N вибірок, ДПФ (Герцель) по бінах 1…N/2, об'єднаних у bands смуг
class BandSpectrum {
public:
    BandSpectrum(int block = 256, int bands = 32) : n_(block), bands_(bands), buf_(size_t(block) * 3, 0.0f), out_(size_t(bands), 0.0f) {}
    // Повертає true, коли блок заповнено й смуги перераховано
    bool push(const float acc[3]) {
        for (int ax = 0; ax < 3; ++ax) buf_[size_t(fill_) * 3 + size_t(ax)] = acc[ax];
        if (++fill_ < n_) return false;
        fill_ = 0;
        std::fill(out_.begin(), out_.end(), 0.0f);
        const int bins = n_ / 2, per = std::max(1, bins / bands_);
        for (int k = 1; k <= bins; ++k) {
            const float w = kTwoPi * float(k) / float(n_), cw = 2.0f * std::cos(w);
            float p = 0;
            for (int ax = 0; ax < 3; ++ax) {   // Герцель для біна k
                float s1 = 0, s2 = 0;
                for (int i = 0; i < n_; ++i) { const float s0 = buf_[size_t(i) * 3 + size_t(ax)] + cw * s1 - s2; s2 = s1; s1 = s0; }
                p += s1 * s1 + s2 * s2 - cw * s1 * s2;
            }
            const int band = std::min(bands_ - 1, (k - 1) / per);
            out_[size_t(band)] += p * 4.0f / (float(n_) * float(n_));   // амплітуда² синусоїди
        }
        return true;
    }
    const std::vector<float>& bands() const { return out_; }
    float bandHz(int i, float fs) const { return (float(i) + 0.5f) * float(std::max(1, (n_ / 2) / bands_)) * fs / float(n_); }

private:
    int n_, bands_, fill_ = 0;
    std::vector<float> buf_, out_;
};

// Привід нахилу: модель meas[t] = cmd[t − s] + c0 + c1·d, де d = +1, коли команда росте, −1 — коли спадає. Мертва зона (люфт b)
// дає енкодеру відставання на b/2 у бік руху: c1 = −b/2. Для кожного зсуву s = 0…max_lag накопичуються суми лінійних найменших
// квадратів з забуванням (пам'ять ≈ memory_s), раз на window_s обирається s з найменшим залишком: це запізнення, а −2·c1 — люфт.
// Без члена c1 люфт «з'їдався» б зсувом і виглядав як зайве запізнення. Одразу після розвороту енкодер ще перетинає мертву зону —
// ці точки не за моделлю, тож вони пропускаються, доки команда не пройде settle_rad від точки розвороту.
class ServoMonitor {
public:
    float dt = 0.005f;            // 200 Гц
    float window_s = 2.0f, max_lag_s = 0.3f, memory_s = 20.0f;
    float settle_rad = 0.05f;     // ≈ 3°: люфт до ≈ 3° вимірюється без зсуву
    float min_samples = 40;       // у кожному напрямку руху
    float lag_s = 0, backlash_rad = 0;
    int estimates = 0;

    void update(float cmd, float meas) {
        const size_t L = size_t(max_lag_s / dt) + 2;
        if (acc_.size() != L - 1) { acc_.assign(L - 1, Acc{}); hist_.assign(L, cmd); }
        hist_.erase(hist_.begin()); hist_.push_back(cmd);   // hist_.back() — поточна команда, hist_[L−1−s] — на s кроків раніше
        const double lam = 1.0 - double(dt) / double(memory_s);
        for (size_t s = 0; s + 1 < L; ++s) {
            Acc& a = acc_[s];
            const double c = hist_[L - 1 - s], cp = hist_[L - 2 - s];
            const double v = c - cp;
            if (std::fabs(v) < 1e-6) continue;
            const double d = v > 0 ? 1.0 : -1.0, e = double(meas) - c;
            if (d != a.d_prev) { a.rev_at = cp; a.d_prev = d; }
            if (std::fabs(c - a.rev_at) < settle_rad) continue;
            a.n = lam * a.n + 1; a.sd = lam * a.sd + d; a.se = lam * a.se + e; a.sed = lam * a.sed + e * d; a.see = lam * a.see + e * e;
            (d > 0 ? a.nu : a.nd) += 1;
        }
        if (++tick_ < size_t(window_s / dt)) return;
        tick_ = 0;
        estimate();
    }

private:
    struct Acc { double n = 0, sd = 0, se = 0, sed = 0, see = 0, d_prev = 0, rev_at = 0, nu = 0, nd = 0; };
    std::vector<Acc> acc_;
    std::vector<float> hist_;
    size_t tick_ = 0;
    void estimate() {
        size_t best = 0; double best_e = 1e300, best_c1 = 0;
        for (size_t s = 0; s < acc_.size(); ++s) {
            const Acc& a = acc_[s];
            if (a.nu < min_samples || a.nd < min_samples) continue;
            const double det = a.n * a.n - a.sd * a.sd;   // Σd² = n (d = ±1)
            if (det < 1e-6 * a.n * a.n) continue;
            const double c0 = (a.se * a.n - a.sd * a.sed) / det, c1 = (a.n * a.sed - a.sd * a.se) / det;
            const double resid = (a.see - c0 * a.se - c1 * a.sed) / a.n;
            if (resid < best_e) { best_e = resid; best = s; best_c1 = c1; }
        }
        if (best_e >= 1e300) return;
        lag_s = float(best) * dt;
        backlash_rad = float(std::fmax(0.0, -2.0 * best_c1));
        ++estimates;
    }
};

// Батарея: V = E − R·I. Опір — з коливань навколо повільного середнього: R = −Σ(ΔV·ΔI) / Σ(ΔI²) з забуванням λ.
// Звичайні двопараметричні найменші квадрати тут погано обумовлені (струм майже сталий, регресори [1, −I] майже колінеарні,
// у float оцінка «розбігається»); на різницях — скалярна задача, стійка. ЕРС E = V + R·I.
class BatteryRls {
public:
    float lambda = 0.999f;        // ≈ 1000 вибірок пам'яті
    float tau_samples = 200;      // повільне середнє (2 с при 100 Гц)
    float min_excitation = 50;    // Σ ΔI² (А²) — менше: опір невідомий
    float E = 0, R = 0;
    int samples = 0;
    bool valid() const { return sxx_ > min_excitation; }
    void update(float v, float i) {
        if (!samples) { v_avg_ = v; i_avg_ = i; }
        const float a = 1.0f / tau_samples;
        v_avg_ += a * (v - v_avg_); i_avg_ += a * (i - i_avg_);
        const double dv = double(v) - v_avg_, di = double(i) - i_avg_;
        sxy_ = lambda * sxy_ + dv * di; sxx_ = lambda * sxx_ + di * di;
        if (valid()) R = float(std::fmax(0.0, -sxy_ / sxx_));
        E = v + R * i;
        ++samples;
    }

private:
    double v_avg_ = 0, i_avg_ = 0, sxy_ = 0, sxx_ = 0;
};

}  // namespace ai
