// weather.h — випадкова погода: штиль, легкий вітер, вітряно, сильний вітер, шквали.
// Кожен «період погоди» триває 1–4 хв (часу симуляції), потім змінюється на інший. Сила і напрям змінюються
// плавно, пориви (турбулентність) — пропорційні силі вітру, як у природі (інтенсивність ~20%).
#pragma once
#include <cmath>
#include <random>
#include "quad_sim.h"
#include "portable_rng.h"

namespace sim {

class Weather {
public:
    enum Kind { Calm, Breeze, Windy, Strong, Squalls, kKinds };

    explicit Weather(unsigned seed = 1) : rng_(seed) { pick(true); speed_ = target_; dir_ = base_dir_; }

    // Викликати кожен крок фізики; оновлює середній вітер (на 10 м) і пориви
    void step(float dt, WindParams& w) {
        t_left_ -= dt;
        if (t_left_ <= 0) pick(false);
        prng::normal<float> n(0, 1);
        // шквали: раз на ~40–80 с різке посилення на 8–15 с
        float tgt = target_;
        if (kind_ == Squalls) {
            squall_t_ -= dt;
            if (squall_t_ <= 0) {
                if (squall_on_) { squall_on_ = false; squall_t_ = U(40, 80); }
                else { squall_on_ = true; squall_t_ = U(8, 15); squall_add_ = U(3, 5); }
            }
            if (squall_on_) tgt += squall_add_;
        }
        const float tau = squall_on_ ? 3.0f : 20.0f;           // шквал налітає за секунди, звичайна зміна — повільно
        speed_ += (tgt - speed_) * (dt / (tau + dt));
        // напрям «гуляє» навколо основного (±15°) і повільно повертає до нового основного
        dir_wander_ += (-dir_wander_ * (dt / 30.0f)) + n(rng_) * 0.26f * std::sqrt(2 * dt / 30.0f);
        float dd = std::remainder(base_dir_ - dir_, 2 * 3.14159265f);
        dir_ += dd * (dt / (40.0f + dt));
        const float a = dir_ + dir_wander_;
        w.mean = Vec3{speed_ * std::cos(a), speed_ * std::sin(a), 0};
        w.gust_sigma = 0.15f + 0.2f * speed_ + (squall_on_ ? 0.5f : 0.0f);
        w.shear_exp = 0.14f;
    }

    Kind kind() const { return kind_; }
    float speed() const { return speed_; }          // м/с на 10 м (без поривів)
    float direction() const { return dir_ + dir_wander_; }  // рад, куди дме (0 = на схід / +X)
    float timeLeft() const { return t_left_; }
    bool squallNow() const { return squall_on_; }

private:
    float U(float a, float b) { return prng::uniform<float>(a, b)(rng_); }
    void pick(bool first) {
        // імовірності: штиль 25%, легкий 30%, вітряно 25%, сильний 12%, шквали 8%
        static const float p[kKinds] = {0.25f, 0.30f, 0.25f, 0.12f, 0.08f};
        Kind k;
        do {
            float r = U(0, 1), acc = 0;
            k = Calm;
            for (int i = 0; i < kKinds; ++i) { acc += p[i]; if (r <= acc) { k = Kind(i); break; } }
        } while (!first && k == kind_);  // нова погода — інша
        kind_ = k;
        switch (k) {
            case Calm: target_ = U(0.0f, 1.0f); break;
            case Breeze: target_ = U(2.0f, 4.0f); break;
            case Windy: target_ = U(5.0f, 7.5f); break;
            case Strong: target_ = U(8.0f, 10.0f); break;
            case Squalls: target_ = U(3.0f, 5.0f); break;
            default: break;
        }
        squall_on_ = false;
        squall_t_ = U(10, 30);
        base_dir_ = first ? U(-3.14159f, 3.14159f) : base_dir_ + U(-0.7f, 0.7f);
        t_left_ = U(60, 240);
    }

    std::mt19937 rng_;
    Kind kind_ = Calm;
    float target_ = 0, speed_ = 0, base_dir_ = 0, dir_ = 0, dir_wander_ = 0, t_left_ = 0;
    float squall_t_ = 0, squall_add_ = 0;
    bool squall_on_ = false;
};

}  // namespace sim
