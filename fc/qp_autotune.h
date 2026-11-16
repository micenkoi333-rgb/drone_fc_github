#pragma once
// Самоналаштування регуляторів у висінні. Автопілот по черзі «смикає» апарат по крену й тангажу прямокутником
// заданої кутової швидкості (±0.6 рад/с, період 1.2 с, по 8 с на вісь; рискання — через нахил гондол, його не чіпаємо) і порівнює, яке кутове прискорення він
// просив (вихід регулятора швидкості) з тим, яке апарат справді отримав (похідна гіроскопа). Обидва сигнали — через однаковий
// фільтр 6 Гц 4-го порядку (нижче пружних коливань рами), затримка підбирається 0…60 мс. Відношення g = отримане / просили показує, наскільки модель помиляється:
// g > 1 — апарат легший за модель або мотори сильніші, g < 1 — навпаки. Інерція в регуляторі ділиться на g, і петлі швидкості
// знову мають ту смугу, під яку їх налаштовано. Велика затримка (> 30 мс) — сигнал знизити підсилення.
#include <algorithm>
#include <cmath>
#include <vector>
#include "filters.h"
#include "fc_math.h"

namespace fc {

class QpAutotune {
public:
    enum State { Idle, Run, Done };
    float amp[3] = {0.6f, 0.6f, 0.4f};
    float period = 1.2f, axis_time = 8.0f;
    State state = Idle;
    int axis = 0;
    float gain[3] = {1, 1, 1}, delay_ms[3] = {0, 0, 0}, lag_ms[3] = {0, 0, 0}, fit[3] = {0, 0, 0};   // fit — частка поясненої дисперсії (R²)
    bool applied = false;
    int rounds = 1;          // кіл виміру (одне коло — обидві осі)
    float total[3] = {1, 1, 1};   // сумарна поправка за всі кола

    void start() { state = Run; axis = 0; t_ = 0; applied = false; rounds = 1; reset(); }
    float progress() const { return state == Done ? 1.0f : state == Run ? (axis + t_ / axis_time) / 2.0f : 0.0f; }

    // кожен цикл, поки апарат висить: що додати до заданої кутової швидкості; запис того, що було просили й отримали на минулому циклі
    Vec3 step(float dt, const Vec3& acc_cmd, const Vec3& omega) {
        if (state != Run) return {0, 0, 0};
        if (!init_) { for (Biquad& b : fa_) b.setup(6.0f, 1.0f / dt); for (Biquad& b : fw_) b.setup(6.0f, 1.0f / dt); for (Biquad& b : fa2_) b.setup(6.0f, 1.0f / dt); for (Biquad& b : fw2_) b.setup(6.0f, 1.0f / dt); w_prev_ = omega; init_ = true; }
        const float a[3] = {acc_cmd.x, acc_cmd.y, acc_cmd.z}, w[3] = {omega.x, omega.y, omega.z}, wp[3] = {w_prev_.x, w_prev_.y, w_prev_.z};
        const float xa = fa2_[axis].apply(fa_[axis].apply(a[axis])), xw = fw2_[axis].apply(fw_[axis].apply((w[axis] - wp[axis]) / dt));
        if (t_ > 1.0f) { cmd_.push_back(xa); got_.push_back(xw); }   // перша секунда — розгін фільтрів
        w_prev_ = omega;
        t_ += dt;
        if (t_ >= axis_time) { solve(dt); ++axis; t_ = 0; reset(); if (axis >= 2) { state = Done; return {0, 0, 0}; } }   // рискання не смикаємо: воно через нахил гондол, повільне й нелінійне
        const float s = std::fmod(t_, period) < 0.5f * period ? 1.0f : -1.0f;
        Vec3 r{0, 0, 0};
        (axis == 0 ? r.x : axis == 1 ? r.y : r.z) = s * amp[axis];
        return r;
    }

private:
    void reset() { cmd_.clear(); got_.clear(); init_ = false; for (Biquad& b : fa_) b = Biquad{}; for (Biquad& b : fw_) b = Biquad{}; for (Biquad& b : fa2_) b = Biquad{}; for (Biquad& b : fw2_) b = Biquad{}; }
    // модель відгуку: отримане = g · (просили, затримане на d і пропущене через запізнення моторів τ), найкраще R² за d і τ
    void solve(float dt) {
        const int n = int(cmd_.size()), dmax = std::min(n / 4, int(0.04f / dt + 0.5f));
        double best_r2 = -1, best_g = 1; int best_d = 0;
        std::vector<double> xl(static_cast<size_t>(n));
        for (float tau : {0.0f, 0.01f, 0.02f, 0.03f, 0.045f, 0.06f, 0.08f, 0.1f}) {
            const double k = tau > 0 ? 1.0 - std::exp(-dt / tau) : 1.0;
            double z = cmd_.empty() ? 0.0 : cmd_[0];
            for (int i = 0; i < n; ++i) { z += (cmd_[size_t(i)] - z) * k; xl[size_t(i)] = z; }
            for (int d = 0; d <= dmax; d += 2) {
                double sxy = 0, sxx = 0, syy = 0;
                for (int i = d; i < n; ++i) { const double x = xl[size_t(i - d)], y = got_[size_t(i)]; sxy += x * y; sxx += x * x; syy += y * y; }
                if (sxx < 1e-9 || syy < 1e-9) continue;
                const double g = sxy / sxx, r2 = 1.0 - (syy - 2 * g * sxy + g * g * sxx) / syy;
                if (r2 > best_r2) { best_r2 = r2; best_g = g; best_d = d; lag_ms[axis] = tau * 1000.0f; }
            }
        }
        fit[axis] = float(std::fmax(0.0, best_r2));
        delay_ms[axis] = best_d * dt * 1000.0f;
        // поправку вносимо лише там, де модель відгуку справді пояснює дані (R² ≥ 0.8): рискання через нахил гондол — повільне й нелінійне
        gain[axis] = fit[axis] >= 0.8f ? float(std::fmin(2.0, std::fmax(0.5, best_g))) : 1.0f;
    }
    std::vector<float> cmd_, got_;
    Biquad fa_[3], fw_[3], fa2_[3], fw2_[3];
    Vec3 w_prev_{0, 0, 0};
    float t_ = 0;
    bool init_ = false;
};

}  // namespace fc
