#pragma once
// Бортовий порадник «модель сил»: усе, що на реальному апараті робить польотний контролер разом із ШІ-співпроцесором.
//  1. Вимірювання зовнішньої сили лише з бортових даних: m·(акселерометр) − Σ тяга гвинтів за обертами ESC і кутами гондол.
//  2. Мережа передбачає силу наперед (за історією), самоперевірка — порівняння з тим, що потім виміряно, → довіра 0…1.
//  3. Сторож обмежує величину й швидкість поради; поправка згасає з повітряною швидкістю (де мережа неточна).
// Симулятор лише подає QpEstimate й команди — правди ця частина не бачить.
#include <cmath>
#include "../fc/filters.h"
#include "../fc/qp_mission.h"
#include "force_model.h"
#include "link.h"

namespace ai {

class ForceMeasure {
public:
    float cutoff_hz = 15.0f;
    // кожен цикл контролера: питома сила й оберти через однаковий фільтр (вібрації гвинтів ≈ 100 Гц)
    void update(float dt, const fc::QpEstimate& e, const fc::QpFlightConfig& cfg, float mass) {
        if (!init_) { acc_.setup(cutoff_hz, 1.0f / dt); for (fc::Biquad& b : rps_) b.setup(cutoff_hz, 1.0f / dt); init_ = true; }
        const fc::Vec3 a = acc_.apply(e.accel);
        const fc::Vec3 vb = e.q.rotateInv(e.vel);
        fc::Vec3 T{0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            const float n = rps_[i].apply(std::fmax(0.0f, e.vtol_rps[i]));
            const float t = e.tilt_meas[i] > -5 ? e.tilt_meas[i] : 0.0f;
            const fc::Vec3 ax{std::sin(t), 0, std::cos(t)};
            const float v_ax = e.airspeed * ax.x + vb.z * ax.z;
            T = T + ax * cfg.rotorThrust(i, n, v_ax);
        }
        f_ = a * mass - T;
        valid_ = e.vtol_rps[0] >= 0;
    }
    fc::Vec3 force() const { return f_; }   // Н, тіло
    bool valid() const { return valid_; }
    void reset() { init_ = false; }
private:
    fc::Biquad3 acc_;
    fc::Biquad rps_[4];
    fc::Vec3 f_{0, 0, 0};
    bool init_ = false, valid_ = false;
};

class ForceAdvisor {
public:
    ForceModel net;
    AiGuard guard;
    ForceMeasure meas;
    int mode = 0;                 // 0 вимкнено, 1 тінь, 2 радить
    bool self_check = true;
    bool horizontal = false;
    float fade_speed = 11.0f;
    bool trace_on = false;
    // стан для інтерфейсу й журналу
    float pred[3] = {0, 0, 0}, trust = 0.5f, check_rms = 0, check_pred[3] = {0, 0, 0}, check_meas[3] = {0, 0, 0}, fade = 0, used_acc[3] = {0, 0, 0};
    bool has_pred = false, trace_valid = false;
    ForceModel::Trace trace;

    static ForceInputs inputs(const fc::QpEstimate& e, const fc::QpActuators& act, float mass_ratio) {
        ForceInputs in;
        in.airspeed = e.airspeed;
        const fc::Vec3 vb = e.q.rotateInv(e.vel);
        in.v_body[0] = vb.x; in.v_body[1] = vb.y; in.v_body[2] = vb.z;
        const fc::Euler eu = fc::eulerOf(e.q);
        in.roll = eu.roll; in.pitch_down = eu.pitch_down;
        for (int i = 0; i < 4; ++i) { in.rps[i] = std::fmax(0.0f, e.vtol_rps[i]); in.tilt[i] = e.tilt_meas[i] > -5 ? e.tilt_meas[i] : act.tilt[i]; }
        in.elevator = act.elevator; in.aileron = act.aileron;
        in.mass_ratio = mass_ratio;
        return in;
    }

    void reset() { net.reset(); has_pred = false; frames_ = 0; }

    // кадр 50 Гц: запам'ятати вимір, передбачити далі; щоціклу 100 Гц — порада контролеру (світ, м/с²)
    fc::Vec3 tick(float now, bool frame, const ForceInputs& in, float mass_est, float mass_nom, const fc::Quat& q) {
        const float mg = mass_nom * 9.80665f;
        if (mode == 0 || !net.loaded()) { reset(); return {0, 0, 0}; }
        net.setWeight(mg);
        const fc::Vec3 m = meas.force();
        if (frame) {
            if (has_pred) {   // що передбачила кадр тому — проти щойно виміряного
                const float mv[3] = {m.x, m.y, m.z};
                double e2 = 0, m2 = 0;
                for (int k = 0; k < 3; ++k) { const double d = pred[k] - mv[k]; e2 += d * d; m2 += double(mv[k]) * mv[k]; check_pred[k] = pred[k]; check_meas[k] = mv[k]; }
                const double a = 0.01;
                err2_ = frames_ < 5 ? e2 : (1 - a) * err2_ + a * e2;
                meas2_ = frames_ < 5 ? m2 : (1 - a) * meas2_ + a * m2;
                ++frames_;
                const double rms = std::sqrt(err2_), scale = 0.15 * std::sqrt(meas2_) + 1.2;
                const double t = std::fmin(1.0, std::fmax(0.0, 1.6 - rms / scale));
                trust = float(frames_ < 50 ? std::fmin(t, 0.5) : t);
                check_rms = float(rms);
            }
            const float mv[3] = {m.x, m.y, m.z};
            net.push(in, mv);
            has_pred = net.predict(pred);
            if (trace_on) trace_valid = net.trace(trace);
        }
        AdviceFrame adv;
        if (has_pred) { adv.status = 2; adv.confidence = self_check ? trust : 1.0f; for (int i = 0; i < 3; ++i) adv.force[i] = pred[i]; }
        guard.cfg.force_max_n = 0.3f * mg;
        guard.cfg.force_rate = 3.0f * mg;
        guard.cfg.timeout_s = 0.05f;
        guard.mode = mode == 2 ? AiGuard::On : AiGuard::Shadow;
        guard.onAdvice(adv, now);
        float fo[3], mo[3];
        guard.output(now, 0.01f, fo, mo);
        // сила виміряна з масою без вантажу, тож вага вантажу в ній уже є як «зовнішня сила вниз»; ту частину ваги, яку контролер
        // і так врахував оцінкою маси, віднімаємо — інакше вантаж компенсувався б двічі (мережею й оцінкою маси) і вони розгойдували б одне одного
        const float m_e = std::fmax(mass_est, 0.1f);
        const fc::Vec3 aw = q.rotate(fc::Vec3{fo[0], fo[1], fo[2]}) * (1.0f / m_e) + fc::Vec3{0, 0, (mass_est - mass_nom) * 9.80665f / m_e} * guardGain();
        fade = std::fmin(1.0f, std::fmax(0.0f, (fade_speed - in.airspeed) / 4.0f)) * (self_check ? trust : 1.0f);
        const fc::Vec3 u{horizontal ? aw.x * fade : 0.0f, horizontal ? aw.y * fade : 0.0f, aw.z * fade};
        used_acc[0] = u.x; used_acc[1] = u.y; used_acc[2] = u.z;
        return u;
    }
private:
    // частка поради, що зараз реально йде (сторож вмикає/вимикає плавно): щоб віднімання ваги вантажу йшло разом з порадою
    float guardGain() const { return has_pred && mode == 2 ? 1.0f : 0.0f; }
    double err2_ = 0, meas2_ = 0;
    long frames_ = 0;
};

}  // namespace ai
