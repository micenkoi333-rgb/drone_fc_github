#pragma once
// Бортова діагностика (працює поруч із мережею сил, лише на бортових даних):
//  - гвинт пошкоджено / мотор слабкий: оберти за телеметрією ESC проти очікуваних за газом (пошкоджений гвинт легший — мотор розкручується швидше);
//  - привід гондоли повільний (знос, слабке живлення): найбільша фактична швидкість повороту проти паспортної;
//  - вітер: у польоті літаком — земна мінус повітряна швидкість; у висінні — з виміряної горизонтальної сили (опір ∝ V²);
//  - зсув центру мас: у висінні з різниці тяги передніх і задніх гвинтів;
//  - енергія: скільки хвилин лишилось і чи вистачить на повернення додому з урахуванням вітру.
#include <cmath>
#include "../fc/qp_mission.h"

namespace ai {

struct HealthReport {
    float motor_ratio[4] = {1, 1, 1, 1};     // оберти / очікувані (1 — норма)
    int prop_damaged = -1, motor_weak = -1;
    float tilt_rate[4] = {0, 0, 0, 0};       // рад/с — найбільша виміряна швидкість повороту гондоли
    int servo_slow = -1;
    float wind[2] = {0, 0}; bool wind_ok = false;
    float cg_x_mm = 0; bool cg_ok = false;   // + — центр мас попереду номіналу
    float wh_left = 0, minutes_left = 0, wh_home = 0; bool can_return = true;
};

class HealthMonitor {
public:
    float batt_wh = 155.0f;          // корисна енергія батареї, Вт·год
    float reserve = 0.15f;           // недоторканний запас
    float hover_cda = 0.09f;         // м²: опір апарата у висінні (бокова й лобова площа × Cd), для вітру з сили
    float prop_damage_ratio = 1.012f, motor_weak_ratio = 0.93f, servo_slow_frac = 0.6f;
    HealthReport r;
    float cg_calm_wind = 3.0f, cg_calm_t_ = 0, cg_as_ = 20;

    void update(float dt, const fc::QpEstimate& e, const fc::QpActuators& act, const fc::QpFlight& fl, const fc::Vec3& meas_force) {
        const fc::QpFlightConfig& c = fl.cfg;
        const fc::QpPhase ph = fl.phase();
        const bool hover = ph == fc::QpPhase::VtolHold && fl.phaseTime() > 4 && fl.vtolWeight() > 0.99f && fl.liftMotorFailed() < 0;
        const bool flying = ph != fc::QpPhase::OnGround && ph != fc::QpPhase::Landed;
        // мотори й гвинти
        if (hover && c.vtol_rps_map.valid() && e.vtol_rps[0] >= 0) {
            for (int i = 0; i < 4; ++i) {
                const float ex = c.vrps(i).fromDuty(act.duty_vtol[i], e.v_bus);
                if (ex < 20) continue;
                r.motor_ratio[i] += (e.vtol_rps[i] / ex - r.motor_ratio[i]) * dt / 3.0f;
                hi_t_[i] = r.motor_ratio[i] > prop_damage_ratio ? hi_t_[i] + dt : 0.0f;
                lo_t_[i] = r.motor_ratio[i] < motor_weak_ratio ? lo_t_[i] + dt : 0.0f;
                if (hi_t_[i] > 2.0f && r.prop_damaged < 0) r.prop_damaged = i;
                if (lo_t_[i] > 2.0f && r.motor_weak < 0) r.motor_weak = i;
            }
        }
        // приводи гондол: при великій похибці привід мусить іти на повній швидкості
        if (c.tilt) for (int i = 0; i < 4; ++i) {
            if (e.tilt_meas[i] < -5) continue;
            if (have_tilt_) {
                const float rate = (e.tilt_meas[i] - tilt_prev_[i]) / dt;
                rate_f_[i] += (rate - rate_f_[i]) * std::fmin(1.0f, dt / 0.02f);
                const float err = act.tilt[i] - e.tilt_meas[i];
                if (std::fabs(err) > 0.17f) {
                    r.tilt_rate[i] = std::fmax(r.tilt_rate[i] * (1 - dt / 20.0f), std::fabs(rate_f_[i]));
                    big_err_t_[i] += dt;
                    if (big_err_t_[i] > 0.5f && r.tilt_rate[i] < servo_slow_frac * c.tilt_rate && r.servo_slow < 0) r.servo_slow = i;
                } else big_err_t_[i] = 0;
            }
            tilt_prev_[i] = e.tilt_meas[i];
        }
        have_tilt_ = true;
        // вітер
        const fc::Euler eu = fc::eulerOf(e.q);
        if (ph == fc::QpPhase::Cruise && e.airspeed > 8) {
            const float wx = e.vel.x - e.airspeed * std::cos(eu.yaw), wy = e.vel.y - e.airspeed * std::sin(eu.yaw);
            r.wind[0] += (wx - r.wind[0]) * dt / 5.0f; r.wind[1] += (wy - r.wind[1]) * dt / 5.0f; r.wind_ok = true;
        } else if (hover) {   // сила опору (світ, гориз.) = ½ρ·CdA·|v|·v, v — повітря відносно апарата
            const fc::Vec3 fw = e.q.rotate(meas_force);
            const float fh = std::hypot(fw.x, fw.y);
            if (fh > 0.05f) {
                const float va = std::sqrt(fh / (0.5f * 1.225f * hover_cda));
                const float wx = e.vel.x + va * fw.x / fh, wy = e.vel.y + va * fw.y / fh;
                r.wind[0] += (wx - r.wind[0]) * dt / 8.0f; r.wind[1] += (wy - r.wind[1]) * dt / 8.0f; r.wind_ok = true;
            }
        }
        // центр мас: моменти тангажу від тяги врівноважені — T_пер·(x_пер − x_цм) = T_зад·(x_цм − x_зад)
        if (hover && e.vtol_rps[0] >= 0) {
            const float tf = c.rotorThrust(0, e.vtol_rps[0], 0) + c.rotorThrust(2, e.vtol_rps[2], 0);
            const float tr = c.rotorThrust(1, e.vtol_rps[1], 0) + c.rotorThrust(3, e.vtol_rps[3], 0);
            cg_as_ += (std::fmax(e.airspeed, 0.0f) - cg_as_) * std::fmin(1.0f, dt / 2.0f);
            cg_calm_t_ = cg_as_ < cg_calm_wind ? cg_calm_t_ + dt : 0.0f;
            if (tf + tr > 1 && cg_calm_t_ > 5.0f) {
                const float x = (c.af.mixer.dx() * tf - c.af.mixer.dxRear() * tr) / (tf + tr) * 1000.0f;
                r.cg_x_mm += (x - r.cg_x_mm) * dt / (r.cg_ok ? 8.0f : 1.0f); r.cg_ok = true;
            }
        }
        // енергія
        const float p = e.v_bus * std::fmax(e.bus_current, 0.0f);
        used_wh_ += p * dt / 3600.0f;
        if (flying) p_avg_ += (p - p_avg_) * dt / 20.0f;
        if (ph == fc::QpPhase::Cruise) p_cruise_ += (p - p_cruise_) * dt / 20.0f;
        if (hover) p_hover_ += (p - p_hover_) * dt / 10.0f;
        r.wh_left = std::fmax(0.0f, batt_wh * (1 - reserve) - used_wh_);
        r.minutes_left = p_avg_ > 20 ? r.wh_left / p_avg_ * 60.0f : 0.0f;
        const fc::Vec3 h = fl.home();
        const float dx = h.x - e.pos.x, dy = h.y - e.pos.y, d = std::hypot(dx, dy);
        float gs = c.v_cruise;
        if (d > 1 && r.wind_ok) gs = std::fmax(3.0f, c.v_cruise + (r.wind[0] * dx + r.wind[1] * dy) / d);   // зустрічний вітер — повільніше
        r.wh_home = (d > 150 ? d / gs * p_cruise_ : d / 5.0f * p_hover_) / 3600.0f + 90.0f * p_hover_ / 3600.0f;   // + переходи й посадка ≈ 90 с висіння
        r.can_return = r.wh_left > 1.2f * r.wh_home;
    }
    float usedWh() const { return used_wh_; }
private:
    float hi_t_[4] = {0, 0, 0, 0}, lo_t_[4] = {0, 0, 0, 0}, big_err_t_[4] = {0, 0, 0, 0}, tilt_prev_[4] = {0, 0, 0, 0}, rate_f_[4] = {0, 0, 0, 0};
    bool have_tilt_ = false;
    float used_wh_ = 0, p_avg_ = 0, p_cruise_ = 400, p_hover_ = 900;
};

}  // namespace ai
