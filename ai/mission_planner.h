#pragma once
// Планувальник енергії місії (до зльоту): скільки батареї піде на маршрут з урахуванням вітру, чи лишиться запас,
// і чи не вигідніше пролетіти точки у зворотному порядку (проти вітру — поки батарея свіжа й апарат ще не втомив запас).
//  - висіння: потужність з теорії імпульсу P = (mg)^1.5 / √(2ρ·ΣA) / (FM·η) — від маси й гвинтів, які знає автопілот;
//  - літак: таблиця, яку енергоменеджер виміряв у попередніх польотах, або апріорна крива (мінімум потужності біля 0.8·V_крейс);
//  - відрізок: земна швидкість з урахуванням кута упередження; швидкість — як обрав би автопілот (або енергоменеджер);
//  - сталі витрати: зліт, переходи, посадка — частка від потужності висіння.
#include <cmath>
#include <vector>
#include "../fc/qp_mission.h"
#include "energy_manager.h"

namespace ai {

struct MissionEnergy {
    float wh = 0, minutes = 0, frac = 0;      // витрата, хвилин, частка корисної ємності
    float wh_reversed = 0;                    // те саме, якщо точки пройти навпаки
    bool fits = true;                         // вкладається в межу
    int worst_leg = -1;                       // найдорожчий відрізок
    float worst_leg_wh = 0;
};

struct MissionPlannerCfg {
    float batt_wh = 155, reserve = 0.20f;     // запас на посадку й непередбачене
    float fm_eta = 0.48f;                     // ККД висіння: figure of merit гвинта × мотор × ESC
    // апріорна крейсерська потужність 13…22 м/с, Вт, поки енергоменеджер не виміряв справжню: штиль, модель цього апарата.
    // Стрибок після 17 м/с — передні гвинти впираються в межу тяги й вмикаються задні (схема cruise_rear = 2)
    float p_table[10] = {145, 141, 139, 150, 187, 290, 387, 399, 440, 480};
    float overhead_hover_s = 85;              // зліт + переходи + посадка в «секундах висіння»
    float climb_eff = 0.45f;
};

inline float hoverPower(const fc::QpFlightConfig& c, float mass, float fm_eta) {
    const float r = c.prop_d / 2, A = 4 * 3.14159265f * r * r, T = mass * 9.80665f;
    return std::pow(T, 1.5f) / std::sqrt(2 * 1.225f * A) / fm_eta;
}

inline float cruisePower(const MissionPlannerCfg& p, const EnergyManager* em, float v) {
    if (em) { const float t = em->powerAt(v); if (t > 0) return t; }
    const float x = std::fmin(std::fmax(v - 13.0f, 0.0f), 8.999f);   // апріорна таблиця 13…22 м/с
    const int i = int(x);
    return p.p_table[i] + (x - float(i)) * (p.p_table[i + 1] - p.p_table[i]);
}

// wind — вітер на висоті крейсеру (світ, м/с); steps — місія як у QpFlight; home — точка старту
inline MissionEnergy planMissionEnergy(const std::vector<fc::QpStep>& steps, const fc::Vec3& home, const fc::Vec3& wind,
                                       const fc::QpFlightConfig& c, float mass, const MissionPlannerCfg& p, const EnergyManager* em = nullptr) {
    auto legs = [&](const std::vector<fc::QpStep>& st, MissionEnergy& out) {
        const float ph = hoverPower(c, mass, p.fm_eta);
        double wh = ph * p.overhead_hover_s / 3600.0, sec = p.overhead_hover_s;
        wh += mass * 9.81 * std::fmax(0.0f, c.cruise_alt - 30.0f) / p.climb_eff / 3600.0;
        fc::Vec3 at = home;
        int leg = 0;
        auto fly = [&](const fc::Vec3& to) {
            const float dx = to.x - at.x, dy = to.y - at.y, d = std::hypot(dx, dy);
            if (d < 1) return;
            const float ux = dx / d, uy = dy / d, wa = wind.x * ux + wind.y * uy, wc = -wind.x * uy + wind.y * ux;
            float best = 1e9f, bv = c.v_cruise, bvg = 1;
            const float vdef = std::fmin(c.v_max, std::fmax(c.v_cruise, std::hypot(wind.x, wind.y) + 8.0f));
            for (float v = 13; v <= c.v_max + 1e-3f; v += 0.5f) {
                if (std::fabs(wc) >= v) continue;
                const float vg = std::sqrt(v * v - wc * wc) + wa;
                if (vg < 3) continue;
                const float e = cruisePower(p, em, v) / vg;
                const bool use = (em && em->on) ? true : std::fabs(v - vdef) < 0.26f;
                if (use && e < best) { best = e; bv = v; bvg = vg; }
            }
            if (best > 1e8f) { best = cruisePower(p, em, c.v_max) / 1.0f; bvg = 1; }   // не долетить проти вітру
            const double lw = best * d / 3600.0;
            wh += lw; sec += d / bvg;
            if (lw > out.worst_leg_wh) { out.worst_leg_wh = float(lw); out.worst_leg = leg; }
            at = to; ++leg; (void)bv;
        };
        for (const fc::QpStep& s : st) {
            if (s.type == fc::QpStep::FlyTo || s.type == fc::QpStep::HoverAt) fly(s.p);
            if (s.type == fc::QpStep::HoverAt) { wh += ph * s.hold_s / 3600.0; sec += s.hold_s; }
            if (s.type == fc::QpStep::Land) fly(s.p.x == 0 && s.p.y == 0 ? home : s.p);
        }
        out.wh = float(wh); out.minutes = float(sec / 60.0);
    };
    MissionEnergy r;
    legs(steps, r);
    std::vector<fc::QpStep> rev;   // ті самі точки навпаки: зліт, точки з кінця, посадка
    std::vector<fc::QpStep> mid;
    for (const fc::QpStep& s : steps) if (s.type == fc::QpStep::FlyTo || s.type == fc::QpStep::HoverAt) mid.push_back(s);
    for (const fc::QpStep& s : steps) if (s.type == fc::QpStep::Takeoff) rev.push_back(s);
    for (auto it = mid.rbegin(); it != mid.rend(); ++it) rev.push_back(*it);
    for (const fc::QpStep& s : steps) if (s.type == fc::QpStep::Land) rev.push_back(s);
    MissionEnergy rv;
    legs(rev, rv);
    r.wh_reversed = rv.wh;
    const float usable = p.batt_wh * (1 - p.reserve);
    r.frac = r.wh / p.batt_wh;
    r.fits = r.wh <= usable;
    return r;
}

}  // namespace ai
