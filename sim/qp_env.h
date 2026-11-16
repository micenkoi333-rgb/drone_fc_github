// qp_env.h — середовище QuadPlane: атмосфера ISA (густина від висоти й відхилення температури) і вітер = середній зі зсувом +
// турбулентність Драйдена (MIL-F-8785C, низькі висоти; шкали й інтенсивності — формули з MathWorks-опису MIL-HDBK-1797, у футах → м).
// Фільтри Драйдена дискретизуються точно (u — AR(1)) або як каскад двох ланок (v, w); білий шум нормується на дисперсію σ².
#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include "vec3d.h"
#include "portable_rng.h"

namespace sim {

struct Atmosphere {
    double temp_offset_c = 0;     // відхилення від ISA, °C
    double field_elevation_m = 0; // висота майданчика над рівнем моря, м

    double temperatureK(double h_agl) const { return 288.15 + temp_offset_c - 0.0065 * std::clamp(h_agl + field_elevation_m, 0.0, 11000.0); }
    double pressure(double h_agl) const {
        const double h = std::clamp(h_agl + field_elevation_m, 0.0, 11000.0);
        return 101325.0 * std::pow(1 - 0.0065 * h / 288.15, 5.25588);   // барометрична формула за стандартним градієнтом (без temp_offset)
    }
    double density(double h_agl) const { return pressure(h_agl) / (287.058 * temperatureK(h_agl)); }
};

// Термічний потік (висхідний): вертикальний профіль за Алленом (NASA TM-2006-214382): середня швидкість
// w̄ = w*·(z/zi)^(1/3)·(1 − 1.1·z/zi), радіус r = max(10, 0.102·(z/zi)^(1/3)·(1 − 0.25·z/zi)·zi);
// радіальна форма — «мексиканський капелюх» w = w_пік·(1 − (ρ/r)²)·e^{−(ρ/r)²} (ядро вгору, кільце спадного потоку, сумарний потік ≈ 0) [О].
// Потік дрейфує із середнім вітром.
struct Thermal {
    double x = 0, y = 0;      // положення в момент t = 0 (на рівні землі), м
    double w_star = 2.5;      // конвективна масштабна швидкість, м/с (типово 1.5–3.5)
    double zi = 1000;         // висота шару перемішування, м
};

struct WindConfig {
    Vec3d mean20ft;               // середній вітер на 6.1 м (світ, ENU), м/с
    double shear_exp = 0.14;      // степінь профілю вітру
    double turbulence = 1.0;      // множник інтенсивності турбулентності (0 — вимкнено)
    double w20_override = 0;      // якщо > 0 — швидкість для σ_w = 0.1·W20 замість |mean20ft|
    std::vector<Thermal> thermals;   // термічні потоки (порожньо — немає)
};

class Wind {
public:
    Wind() = default;
    Wind(const WindConfig& c, unsigned seed) : cfg_(c), rng_(seed) {}
    const WindConfig& config() const { return cfg_; }

    Vec3d mean(double h_agl) const {
        const double k = std::pow(std::fmax(h_agl, 1.0) / 6.1, cfg_.shear_exp);
        return cfg_.mean20ft * k;
    }

    // Вертикальна швидкість термічних потоків у точці (світ), t — час від початку, с
    double thermalW(double x, double y, double h_agl, double t) const {
        double w = 0;
        const Vec3d drift = mean(10.0);
        for (const Thermal& th : cfg_.thermals) {
            const double zz = std::clamp(h_agl / th.zi, 0.01, 1.0);
            const double c = std::cbrt(zz);
            const double w_avg = std::fmax(0.0, th.w_star * c * (1 - 1.1 * zz));
            const double r = std::fmax(10.0, 0.102 * c * (1 - 0.25 * zz) * th.zi);
            const double dx = x - (th.x + drift.x * t), dy = y - (th.y + drift.y * t);
            const double s2 = (dx * dx + dy * dy) / (r * r);
            w += 2.5 * w_avg * (1 - s2) * std::exp(-s2);   // пік ≈ 2.5·w̄ (Аллен: w_пік/w̄ ≈ 2–3 для вузьких потоків)
        }
        return w;
    }

    // Оновити турбулентність за dt і повернути повний вітер (світ). airspeed — повітряна швидкість літака (для шкали L/V)
    Vec3d step(double dt, double h_agl, double airspeed) {
        const double h = std::clamp(h_agl, 3.0, 300.0), hf = h * 3.28084;
        const double w20 = cfg_.w20_override > 0 ? cfg_.w20_override : std::hypot(cfg_.mean20ft.x, cfg_.mean20ft.y);
        const double sw = 0.1 * w20 * cfg_.turbulence;
        const double den = 0.177 + 0.000823 * hf;
        const double su = sw / std::pow(den, 0.4);
        const double lw = h, lu = hf / std::pow(den, 1.2) * 0.3048;   // м
        const double v = std::fmax(airspeed, 5.0);
        if (sw > 1e-9) {
            // u: AR(1) з точною дисперсією σ_u²
            const double a = std::exp(-v * dt / lu);
            u_ = a * u_ + su * std::sqrt(1 - a * a) * n01();
            // v, w: (1+√3τs)/(1+τs)² = √3·p + (1−√3)·r, p' = (n−p)/τ, r' = (p−r)/τ, інтенсивність шуму q = σ²τ
            gust2(v_p_, v_r_, v_y_, su, lu / v, dt);
            const double v_old = v_y_, w_old = w_y_;
            gust2(w_p_, w_r_, w_y_, sw, lw / v, dt);
            // градієнти поривів уздовж апарата (MIL-F-8785C): p_g = ∂w/∂y (крен), q_g = ∂w/∂x (тангаж), r_g = ∂v/∂x (рискання); q_g, r_g — з «замороженого» поля
            const double b = 1.6, sigp = sw * std::sqrt(0.8 / lw) * std::pow(3.14159265358979 / (4 * b), 1.0 / 6.0) / std::cbrt(2 * lw);
            const double ap = std::exp(-dt * 3.14159265358979 * v / (4 * b));
            p_g_ = ap * p_g_ + sigp * std::sqrt(1 - ap * ap) * n01();
            q_g_ = std::clamp(-(w_y_ - w_old) / dt / v, -1.5, 1.5);
            r_g_ = std::clamp(-(v_y_ - v_old) / dt / v, -1.5, 1.5);
        } else {
            u_ = v_y_ = w_y_ = v_p_ = v_r_ = w_p_ = w_r_ = p_g_ = q_g_ = r_g_ = 0;
        }
        const Vec3d m = mean(h_agl);
        const double psi = std::hypot(cfg_.mean20ft.x, cfg_.mean20ft.y) > 0.3 ? std::atan2(cfg_.mean20ft.y, cfg_.mean20ft.x) : 0.0;
        const Vec3d gust{u_ * std::cos(psi) - v_y_ * std::sin(psi), u_ * std::sin(psi) + v_y_ * std::cos(psi), w_y_};
        last_ = m + gust;
        return last_;
    }
    Vec3d last() const { return last_; }
    Vec3d gustRates() const { return Vec3d{p_g_, q_g_, r_g_}; }   // градієнти поривів, 1/с: ∂w/∂y, ∂w/∂x, ∂v/∂x

private:
    double n01() { return nd_(rng_); }
    void gust2(double& p, double& r, double& y, double sigma, double tau, double dt) {
        const double q = sigma * sigma * tau;
        const int sub = std::max(1, int(std::ceil(dt / (0.2 * tau))));   // підкроки, коли τ малий
        const double h = dt / sub;
        for (int i = 0; i < sub; ++i) {
            p += h * (-p / tau) + std::sqrt(q * h) / tau * n01();
            r += h * (p - r) / tau;
        }
        y = 1.7320508075688772 * p + (1 - 1.7320508075688772) * r;
    }

    WindConfig cfg_;
    std::mt19937 rng_{1};
    prng::normal<double> nd_{0.0, 1.0};
    double u_ = 0, v_p_ = 0, v_r_ = 0, v_y_ = 0, w_p_ = 0, w_r_ = 0, w_y_ = 0, p_g_ = 0, q_g_ = 0, r_g_ = 0;
    Vec3d last_;
};

}  // namespace sim
