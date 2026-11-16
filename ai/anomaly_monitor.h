#pragma once
// Передбачення поломок: апарат запам'ятовує, яким він був «здоровим» (перші хвилини польоту або профіль з попереднього польоту),
// і помічає, коли щось поступово стає гіршим — ще до відмови:
//  - вібрації: рівень «тремтіння» акселерометра відносно обертів (розбалансований чи надщерблений гвинт, розбитий підшипник, ослаблене
//    кріплення); який мотор — за збігом частоти вібрації з обертами кожного мотора (демодуляція на частоті обертання з телеметрії ESC);
//  - батарея: внутрішній опір з того, як напруга просідає під струмом (регресія напруги на струм) — старіння, пошкоджена комірка, поганий роз'єм;
//  - приводи гондол: середня розбіжність команди й датчика кута в усталеному стані — знос шестерень, люфт, ослаблена тяга.
#include <array>
#include <cmath>
#include "../fc/qp_mission.h"

namespace ai {

struct AnomalyProfile {   // «здоровий» апарат
    float vib = 0, r_batt = 0;
    std::array<float, 4> servo_err{};
    bool ready = false;
};

class AnomalyMonitor {
public:
    float learn_s = 90.0f;                 // скільки секунд польоту вважати «здоровими», якщо профілю ще немає
    float vib_alarm = 1.5f, r_alarm = 1.5f, servo_alarm = 2.5f;
    AnomalyProfile base;
    // поточні значення й висновки
    float vib = 0, r_batt = 0;
    std::array<float, 4> servo_err{}, motor_share{};
    int vib_motor = -1;
    bool vib_bad = false, batt_bad = false;
    int servo_bad = -1;
    void importProfile(const AnomalyMonitor& o) { if (o.base.ready) { base = o.base; base_own_ = false; } }   // «здоровий» профіль з попереднього польоту
    float vibRatio() const { return base.vib > 0 ? vib / base.vib : 0.0f; }
    float rRatio() const { return base.r_batt > 0 ? r_batt / base.r_batt : 0.0f; }

    void update(float dt, const fc::QpEstimate& e, const fc::QpActuators& act, const fc::QpFlight& fl) {
        const fc::QpPhase ph = fl.phase();
        const bool flying = ph != fc::QpPhase::OnGround && ph != fc::QpPhase::Landed && e.pos.z - fl.home().z > 3;
        if (!flying) return;
        fly_t_ += dt;
        // 1) вібрації: високочастотна частина акселерометра (без руху апарата) на квадрат обертів
        const fc::Vec3 a = e.accel;
        const fc::Vec3 hp{a.x - lp_.x, a.y - lp_.y, a.z - lp_.z};
        lp_ = lp_ + (a - lp_) * std::fmin(1.0f, dt / 0.02f);
        float k = 0;
        for (int i = 0; i < 4; ++i) { const float r = std::fmax(0.0f, e.vtol_rps[i]) / 100.0f; k += r * r; }
        if (k > 0.2f) {
            const float v2 = (hp.x * hp.x + hp.y * hp.y + hp.z * hp.z) / k;
            vib2_ += (v2 - vib2_) * std::fmin(1.0f, dt / 5.0f);
            vib = std::sqrt(vib2_);
            for (int i = 0; i < 4; ++i) {   // демодуляція на частоті обертання кожного мотора
                ph_[size_t(i)] += 6.2831853f * std::fmax(0.0f, e.vtol_rps[i]) * dt;
                if (ph_[size_t(i)] > 6.2831853f) ph_[size_t(i)] -= 6.2831853f;
                const float c = std::cos(ph_[size_t(i)]), s = std::sin(ph_[size_t(i)]), kk = std::fmin(1.0f, dt / 0.25f);
                const float h[3] = {hp.x, hp.y, hp.z};
                float p = 0;
                for (int ax = 0; ax < 3; ++ax) {
                    iq_[size_t(i)][size_t(2 * ax)] += (h[ax] * c - iq_[size_t(i)][size_t(2 * ax)]) * kk;
                    iq_[size_t(i)][size_t(2 * ax + 1)] += (h[ax] * s - iq_[size_t(i)][size_t(2 * ax + 1)]) * kk;
                    p += iq_[size_t(i)][size_t(2 * ax)] * iq_[size_t(i)][size_t(2 * ax)] + iq_[size_t(i)][size_t(2 * ax + 1)] * iq_[size_t(i)][size_t(2 * ax + 1)];
                }
                const float r = std::fmax(0.0f, e.vtol_rps[i]) / 100.0f;
                pw_[size_t(i)] += (p / std::fmax(r * r, 0.05f) - pw_[size_t(i)]) * std::fmin(1.0f, dt / 10.0f);
            }
            float tot = 0;
            for (float p : pw_) tot += p;
            for (int i = 0; i < 4; ++i) motor_share[size_t(i)] = tot > 0 ? pw_[size_t(i)] / tot : 0.0f;
        }
        // 2) внутрішній опір батареї: швидкі зміни струму (пориви, маневри) проти швидких змін напруги — так видно лише омічний опір,
        //    повільна поляризація й розряд відфільтровані (обидва сигнали — мінус їхнє середнє за 0.5 с)
        {
            const float I = e.bus_current, V = e.v_bus, kf = std::fmin(1.0f, dt / 0.5f), ks = std::fmin(1.0f, dt / 6.0f);
            mi_ += (I - mi_) * kf; mv_ += (V - mv_) * kf;
            const float di = I - mi_, dv = V - mv_;
            cii_ += (di * di - cii_) * ks; civ_ += (di * dv - civ_) * ks;
            if (cii_ > 0.5f) { const float r = -civ_ / cii_; if (r > 0.005f && r < 1.0f) r_batt = r_batt > 0 ? r_batt + (r - r_batt) * std::fmin(1.0f, dt / 20.0f) : r; }
        }
        // 3) приводи гондол: розбіжність команди й кута, коли команда майже стоїть
        if (fl.cfg.tilt) for (int i = 0; i < 4; ++i) {
            if (e.tilt_meas[i] < -5) continue;
            const float d = act.tilt[i] - cmd_prev_[size_t(i)];
            cmd_prev_[size_t(i)] = act.tilt[i];
            still_[size_t(i)] = std::fabs(d) < 0.02f * dt ? still_[size_t(i)] + dt : 0.0f;
            if (still_[size_t(i)] > 1.0f) servo_err[size_t(i)] += (std::fabs(act.tilt[i] - e.tilt_meas[i]) - servo_err[size_t(i)]) * std::fmin(1.0f, dt / 10.0f);
        }
        // профіль «здорового» апарата
        if (!base.ready && fly_t_ > learn_s && vib > 0) { base.vib = vib; base.r_batt = r_batt; base.servo_err = servo_err; base.ready = true; base_own_ = true; }
        if (!base.ready) return;
        if (base_own_ && fly_t_ < 2 * learn_s) { base.r_batt += (r_batt - base.r_batt) * std::fmin(1.0f, dt / 30.0f); return; }   // опір сходиться повільніше: уточнюємо ще стільки ж
        auto hold = [&](bool c, float& t) { t = c ? t + dt : 0.0f; return t > 5.0f; };
        vib_bad = hold(vib > vib_alarm * base.vib, vib_t_);
        if (vib_bad) { int m = 0; for (int i = 1; i < 4; ++i) if (motor_share[size_t(i)] > motor_share[size_t(m)]) m = i; vib_motor = motor_share[size_t(m)] > 0.45f ? m : -1; }   // мотор — лише коли частоти розійшлись (у літаку крутяться передні)
        batt_bad = base.r_batt > 0 && hold(r_batt > r_alarm * base.r_batt, r_t_);
        for (int i = 0; i < 4; ++i)
            if (base.servo_err[size_t(i)] > 0 && hold(servo_err[size_t(i)] > servo_alarm * std::fmax(base.servo_err[size_t(i)], 0.003f), sv_t_[size_t(i)]) && servo_bad < 0) servo_bad = i;
    }

private:
    fc::Vec3 lp_{0, 0, 9.81f};
    float vib2_ = 0, fly_t_ = 0, vib_t_ = 0, r_t_ = 0;
    bool base_own_ = false;
    float mi_ = 0, mv_ = 0, cii_ = 0, civ_ = 0;
    std::array<float, 4> ph_{}, pw_{}, cmd_prev_{}, still_{}, sv_t_{};
    std::array<std::array<float, 6>, 4> iq_{};
};

}  // namespace ai
