// qp_mission.h — режими польоту: зліт, висіння, перехід у літак, крейсер за L1+TECS, перехід у коптер, посадка.
// За зразком ArduPilot QuadPlane / tiltrotor: вперед — розгін тягою вперед до V_trans, далі гвинти плавно знімають вертикальне навантаження (blend),
// Q_ASSIST — коптерний режим підхоплює при малій швидкості, великому крені/тангажі чи низькій висоті; назад — плавне гальмування.
// Конвертоплан (cfg.tilt): «тяга вперед» і «коптерна тяга» — це ті самі 4 гвинти; кут кожної гондоли = atan2(сила вперед, сила вгору)
// (allocateTilt) — у висінні ≈ 0°, у переході 20…60°, у польоті літаком 90°. Рискання у висінні — різницевим нахилом гондол.
// Контролер працює на 500 Гц; усі команди — нормовані (duty моторів, кути гондол і поверхонь у рад).
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "qp_control.h"
#include "qp_types.h"
#include "qp_safety.h"
#include "qp_log.h"
#include "qp_autotune.h"

namespace fc {


class QpFlight {
public:
    QpFlightConfig cfg;
    QpManual manual;
    QpSafetyConfig safety;
    QpLog flog;   // бортовий журнал автопілота
    Tecs tecs;
    L1Guidance l1;

    void configure(const QpFlightConfig& c) {
        cfg = c;
        vtol_.configure(cfg.af);
        fw_.configure(cfg.af);
        tecs.cfg.thrust_max = cfg.cruise_thrust_max;
        l1.period = 17.0f;
    }
    void setMission(std::vector<QpStep> steps) { mission_ = std::move(steps); step_ = 0; phase_ = QpPhase::OnGround; phase_t_ = 0; }
    void setMass(float m, const Vec3& inertia) { cfg.af.mass = m; cfg.af.inertia = inertia; vtol_.setMass(m, inertia); tecs.setMass(m); }
    QpAutotune tune;   // самоналаштування регуляторів у висінні: startAutotune(), результат — tune.gain / delay_ms
    void startAutotune() { tune.start(); }
    const Vec3& inertiaControl() const { return cfg.af.inertia; }
    // Оцінка маси в польоті (вантаж, якого автопілот не знав): у висінні інтегратор висоти показує, скільки тяги бракує чи забагато —
    // ця різниця повільно (≈ 4 с) переноситься в масу, а інтегратор зменшується на стільки ж. Межі — 0.8…2.0 від маси без вантажу.
    float massEstimate() const { return mass_ai_ > 0 ? mass_ai_ : cfg.af.mass; }
    float massControl() const { return cfg.af.mass; }   // маса, з якою рахує регулятор
    float verticalIntegral() const { return vtol_.verticalIntegral(); }
    float massNominal() const { return mass_nom_ > 0 ? mass_nom_ : cfg.af.mass; }
    bool mass_adapt = true;
    // ШІ радить (сила виміряна з масою без вантажу): «маса з виміряної сили» = m₀ − F_z/g. Тоді оцінка маси йде за нею, а не за інтегратором —
    // інтегратор при цьому вантажу «не бачить» (вантаж компенсує порада ШІ). 0 — ШІ не радить.
    float ai_mass_meas = 0;
    QpPhase phase() const { return phase_; }
    size_t stepIndex() const { return step_; }
    bool finished() const { return phase_ == QpPhase::Landed || (step_ >= mission_.size() && phase_ == QpPhase::VtolHold); }
    float vtolWeight() const { return wv_; }
    // ШІ-співпроцесор: передбачене зовнішнє прискорення (світ, м/с²), уже пропущене через сторож (ai::AiGuard)
    void setAiAccel(const Vec3& a) { ai_acc_ = a; }
    Vec3 holdPoint() const { return hold_; }
    void setAiPitchUp(float r) { ai_pitch_up_ = r; }   // ШІ-пілот v2: поправка кута носа в утриманні точки, рад
    // виміряна зовнішня сила (тіло, Н) — бортовий вимір з акселерометра й телеметрії ESC (ai/force_advisor.h, ForceMeasure)
    void setMeasuredForce(const Vec3& f_body, bool ok) { meas_f_ = f_body; meas_ok_ = ok; }
    Vec3 aiAccel() const { return ai_acc_; }
    // Після трюку (керування перехоплювали): літакові регулятори починають з поточного стану, без накопиченого
    void resumeAfterOverride(const QpEstimate& e) {
        tecs.reset(e.pos.z, e.airspeed); fw_.reset();
        for (int i = 0; i < 4; ++i) { if (e.tilt_meas[i] > -5) tilt_est_[i] = tilt_cmd_[i] = e.tilt_meas[i]; tilt_bad_t_[i] = 0; }
    }
    float phaseTime() const { return phase_t_; }
    const Vec3& home() const { return home_; }   // точка зльоту (оцінка), світ
    const QpStep* currentStep() const { return step_ < mission_.size() ? &mission_[step_] : nullptr; }
    // поради ШІ-енергоменеджера в польоті літаком (0 — немає): повітряна швидкість, м/с, і висота над домом, м
    float energy_v_dem = 0, energy_alt = 0;
    bool armed() const { return armed_; }
    uint32_t preflightFailures() const { return pf_fail_; }      // біти PreflightBit; 0 — усе гаразд
    bool fenceBreached() const { return fence_breach_; }
    bool rcFailsafe() const { return rc_failsafe_; }
    bool assisting() const { return assist_; }
    float tiltCmd(int i) const { return tilt_cmd_[i]; }    // рад: команда нахилу гондоли
    bool gpsFailsafe() const { return gps_failsafe_; }
    bool pitotFailed() const { return pitot_failed_; }
    bool cruiseMotorFailed() const { return cruise_fail_; }
    int liftMotorFailed() const { return lift_fail_; }      // −1 — усі підйомні мотори справні
    int liftMotorWeak() const { return lift_weak_; }
    // текст аварійного стану для інтерфейсу (порожній — усе гаразд)
    std::string alertText() const {
        std::string s;
        if (lift_fail_ >= 0) s += "відмова підйомного мотора " + std::to_string(lift_fail_ + 1) + (phase_ == QpPhase::FwLand ? ": посадка літаком" : ": посадка з обертанням") + "; ";
        else if (lift_weak_ >= 0) s += "підйомний мотор " + std::to_string(lift_weak_ + 1) + " слабкий: посадка; ";
        if (gps_failsafe_) s += "GPS втрачено; ";
        if (fence_breach_) s += "вихід за геозону: повернення; ";
        if (rc_failsafe_) s += "втрачено пульт: повернення; ";
        if (phase_ == QpPhase::OnGround && pf_fail_) s += "не армується: " + preflightText(pf_fail_) + "; ";
        if (pitot_failed_) s += "труба Піто несправна; ";
        if (cruise_fail_) s += cfg.tilt ? "гвинти не тягнуть уперед; " : "маршовий мотор не працює; ";
        if (tilt_fail_ >= 0) s += "привід гондоли " + std::to_string(tilt_fail_ + 1) + " заклинив: посадка; ";
        if (rtl_) s += "низька напруга: повернення; ";
        if (lim_ < 0.98f) s += "обмеження струму; ";
        return s;
    }
    const char* statusText() const { return phaseName(phase_); }

    QpActuators update(float dt, const QpEstimate& e_raw) {
        const QpPhase p0 = phase_;
        const bool f0[5] = {gps_failsafe_, fence_breach_, rc_failsafe_, pitot_failed_, rtl_};
        {   // самоналаштування: лише в усталеному висінні, без ручного керування й відмов
            const bool ok = phase_ == QpPhase::VtolHold && phase_t_ > 4.0f && !manual.on && lift_fail_ < 0 && tilt_fail_ < 0;
            vtol_.rate_inject = ok ? tune.step(dt, vtol_.lastAcc(), e_raw.omega) : Vec3{0, 0, 0};
            if (tune.state == QpAutotune::Done && !tune.applied) {
                tune.applied = true;
                const Vec3 I = cfg.af.inertia;
                setMass(cfg.af.mass, Vec3{I.x / tune.gain[0], I.y / tune.gain[1], I.z / tune.gain[2]});
                for (int k = 0; k < 3; ++k) tune.total[k] = (tune.rounds == 1 ? 1.0f : tune.total[k]) * tune.gain[k];
                char b[160]; std::snprintf(b, sizeof b, "самоналаштування: g = %.2f / %.2f / %.2f, затримка %.0f / %.0f / %.0f мс",
                                           tune.gain[0], tune.gain[1], tune.gain[2], tune.delay_ms[0], tune.delay_ms[1], tune.delay_ms[2]);
                flog.event(clock_, b);
                // ще коло, якщо поправка велика (за одне коло підсилення впирається в межі): до 3 кіл
                if ((std::fabs(tune.gain[0] - 1) > 0.12f || std::fabs(tune.gain[1] - 1) > 0.12f) && tune.rounds < 3) { const int rr = tune.rounds + 1; tune.start(); tune.rounds = rr; }
            }
        }
        QpEstimate e_hold = e_raw;
        if (cfg.hold_gps_tau > 0 && phase_ == QpPhase::VtolHold) {
            // утримання точки без дрейфу GPS: положення = інтеграл швидкості (доплерівська швидкість GPS / оптичний потік не дрейфують),
            // а саме положення GPS лише повільно підтягує його (τ) — апарат стоїть там, де стоїть, а не «пливе» за похибкою GPS
            if (!ploc_on_) { ploc_ = e_raw.pos; ploc_on_ = true; }
            const float k = std::fmin(1.0f, dt / cfg.hold_gps_tau);
            ploc_.x += e_raw.vel.x * dt + (e_raw.pos.x - ploc_.x) * k;
            ploc_.y += e_raw.vel.y * dt + (e_raw.pos.y - ploc_.y) * k;
            e_hold.pos.x = ploc_.x; e_hold.pos.y = ploc_.y;
        } else ploc_on_ = false;
        const QpActuators out = step(dt, e_hold);
        if (phase_ == QpPhase::Landed && armed_) { armed_ = false; flog.event(clock_, "роззброєно"); }
        if (armed_ && p0 == QpPhase::OnGround) flog.event(clock_, "армовано");
        if (flog.on) {
            clock_ += dt;
            if (phase_ != p0) flog.event(clock_, std::string("фаза: ") + phaseName(phase_));
            static const char* fn[5] = {"GPS втрачено", "вихід за геозону", "втрачено пульт", "Піто несправна", "низька напруга: додому"};
            const bool f1[5] = {gps_failsafe_, fence_breach_, rc_failsafe_, pitot_failed_, rtl_};
            for (int i = 0; i < 5; ++i) if (f1[i] && !f0[i]) flog.event(clock_, fn[i]);
            QpLogRec r{};
            const Euler eu = eulerOf(e_raw.q);
            r.t = clock_; r.phase = uint8_t(phase_);
            r.flags = uint8_t((armed_ ? 1 : 0) | (gps_failsafe_ ? 2 : 0) | (fence_breach_ ? 4 : 0) | (rc_failsafe_ ? 8 : 0) | (pitot_failed_ ? 16 : 0) |
                              (lift_fail_ >= 0 ? 32 : 0) | (tilt_fail_ >= 0 ? 64 : 0) | (manual.on ? 128 : 0));
            r.px = e_raw.pos.x; r.py = e_raw.pos.y; r.pz = e_raw.pos.z; r.vx = e_raw.vel.x; r.vy = e_raw.vel.y; r.vz = e_raw.vel.z;
            r.roll = eu.roll; r.pitch = eu.pitch_down; r.yaw = eu.yaw; r.p = e_raw.omega.x; r.q = e_raw.omega.y; r.r = e_raw.omega.z;
            r.airspeed = e_raw.airspeed; r.v_bus = e_raw.v_bus; r.amps = e_raw.bus_current;
            for (int i = 0; i < 4; ++i) { r.duty[i] = out.duty_vtol[i]; r.tilt[i] = out.tilt[i]; r.rps[i] = e_raw.vtol_rps[i]; r.tilt_meas[i] = e_raw.tilt_meas[i]; }
            r.ail = out.aileron; r.ele = out.elevator; r.rud = out.rudder; r.cruise = out.duty_cruise;
            flog.record(dt, r);
        }
        return out;
    }
private:
    float clock_ = 0;
    QpActuators step(float dt, const QpEstimate& e_raw) {
        QpEstimate e = e_raw;
        {   // контроль достовірності датчиків: труба Піто показує майже нуль при великій земній швидкості → довіряємо земній швидкості
            const float gs0 = std::hypot(e.vel.x, e.vel.y);
            // лише в польоті літаком: у висінні / на переході за вітром мала повітряна швидкість при великій земній — норма, не відмова
            const bool wing_borne = phase_ == QpPhase::Cruise || (phase_ == QpPhase::TransFwd && wv_ < 0.5f);
            // на переході: маршовий гвинт тягне ≥ 6 с, а швидкість майже нуль — так не буває навіть за вітром (апарат розганяється відносно повітря)
            const bool trans_dead = phase_ == QpPhase::TransFwd && phase_t_ > 6.0f && e.airspeed < 3.0f && gs0 > 8.0f;
            pitot_bad_t_ = ((wing_borne && e.airspeed < 0.4f * gs0 && gs0 > 8.0f) || trans_dead) ? pitot_bad_t_ + dt : std::fmax(0.0f, pitot_bad_t_ - dt);
            if (pitot_bad_t_ > 2.0f) { e.airspeed = gs0; pitot_failed_ = true; }
            // GPS: пропав сигнал → без позиції керуємо лише кутами (рівно) й висотою за баро: перехід у коптер і вертикальна посадка
            gps_bad_t_ = e.gps_ok ? 0.0f : gps_bad_t_ + dt;
            if (gps_bad_t_ > 1.5f && !gps_failsafe_ && phase_ != QpPhase::OnGround && phase_ != QpPhase::Landed) {
                gps_failsafe_ = true;
                if (phase_ == QpPhase::Cruise || phase_ == QpPhase::TransFwd) { hold_ = Vec3{e.pos.x, e.pos.y, 25.0f}; enter(QpPhase::TransBack, e); }
                else if (phase_ != QpPhase::Landing) abortToLanding(e);
            }
        }
        monitorMotors(dt, e);
        monitorTilt(dt, e);
        adaptMass(dt, e);
        rc_lost_t_ = e.rc_ok ? 0.0f : rc_lost_t_ + dt;   // пульт мовчить: ручне керування скасовується, апарат летить додому
        if (manual.on && rc_lost_t_ > safety.rc_timeout_s && phase_ != QpPhase::OnGround && phase_ != QpPhase::Landed) {
            manual.on = false; rc_failsafe_ = true;
            if (home_.norm() < 1e-6f) home_ = e.pos;
            QpStep l; l.type = QpStep::Land; l.p = Vec3{home_.x, home_.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false;
        }
        if (!manual.on && !rc_failsafe_ && safety.rc_auto_action > 0 && rc_lost_t_ > safety.rc_timeout_s * 5 &&
            phase_ != QpPhase::OnGround && phase_ != QpPhase::Landed && phase_ != QpPhase::Landing && phase_ != QpPhase::FwLand) {
            rc_failsafe_ = true;
            if (safety.rc_auto_action == 1) goHome(e); else if (phase_ == QpPhase::Cruise || phase_ == QpPhase::TransFwd) { hold_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; QpStep l; l.type = QpStep::Land; l.p = Vec3{e.pos.x, e.pos.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false; enter(QpPhase::TransBack, e); } else abortToLanding(e);
        }
        if (manual.direct()) return manualDirect(dt, e);
        if (phase_ == QpPhase::Manual) leaveManual(e);
        QpActuators out;
        t_ += dt; phase_t_ += dt;
        if (phase_ != QpPhase::OnGround && phase_ != QpPhase::Landing && phase_ != QpPhase::Landed) {   // аварійний режим: низька напруга → додому й на посадку
            const float v_rest = e.v_bus + std::fmax(e.bus_current, 0.0f) * cfg.batt_r_est;   // без просідання під навантаженням: сильний вітер ≠ порожня батарея
            low_t_ = v_rest < cfg.v_low ? low_t_ + dt : std::fmax(0.0f, low_t_ - dt);
            if (low_t_ > 3.0f && !rtl_) { rtl_ = true; goHome(e); }
        }
        if (phase_ != QpPhase::OnGround && phase_ != QpPhase::Landing && phase_ != QpPhase::Landed && phase_ != QpPhase::FwLand) {
            const float d = std::hypot(e.pos.x - home_.x, e.pos.y - home_.y), h = e.pos.z - home_.z;
            const bool out_r = safety.fence_radius > 0 && d > safety.fence_radius, out_h = safety.fence_alt > 0 && h > safety.fence_alt;
            if ((out_r || out_h) && !fence_breach_) { fence_breach_ = true; goHome(e); }
            if (fence_breach_ && safety.fence_radius > 0 && d > safety.fence_radius * safety.fence_margin && phase_ != QpPhase::Cruise && phase_ != QpPhase::TransFwd) abortToLanding(e);
        }
        const Euler eu = eulerOf(e.q);
        const float alt = e.pos.z;
        const QpStep* st = step_ < mission_.size() ? &mission_[step_] : nullptr;

        const bool man = manualTick(dt, e, eu, out);
        if (!man) switch (phase_) {
            case QpPhase::OnGround:
                ground_z_ = ground_n_++ == 0 ? e.pos.z : ground_z_ + (e.pos.z - ground_z_) * std::fmin(1.0f, dt / 0.5f);   // висота землі — усереднена (як калібрування баро перед зльотом)
                pf_fail_ = safety.preflight ? preflightCheck(e, safety, cfg.v_low, cfg.tilt, manual.on) : 0u;
                pf_ok_t_ = pf_fail_ ? 0.0f : pf_ok_t_ + dt;
                if (phase_t_ > 1.5f && st && pf_ok_t_ >= safety.arm_ok_s) { armed_ = true; home_ = Vec3{e.pos.x, e.pos.y, ground_z_}; hold_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; yaw_ = eu.yaw; enter(QpPhase::Takeoff, e); }
                break;
            case QpPhase::Takeoff: {
                const float target = (st && st->type == QpStep::Takeoff) ? st->p.z : 30.0f;   // перший крок не Takeoff — стандартні 30 м
                tg_.zmode = alt < target - 3.0f ? VtolTargets::ClimbRate : VtolTargets::Altitude;
                tg_.climb_rate = cfg.takeoff_climb; tg_.pos = Vec3{hold_.x, hold_.y, target};
                tg_.hmode = VtolTargets::Position; tg_.yaw = yaw_;
                if (alt > target - 1.0f && std::hypot(e.vel.x, e.vel.y) < 1.0f && std::fabs(e.vel.z) < 0.7f) { hold_ = Vec3{hold_.x, hold_.y, target}; nextStep(e); }
                break;
            }
            case QpPhase::VtolHold: {
                tg_.zmode = VtolTargets::Altitude; tg_.hmode = VtolTargets::Position; tg_.pos = hold_; tg_.yaw = yaw_;
                tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad;
                if (!st) break;
                if (st->type == QpStep::FlyTo) {
                    from_ = Vec3{e.pos.x, e.pos.y, e.pos.z};
                    enter(QpPhase::TransFwd, e);
                } else if (st->type == QpStep::HoverAt && std::hypot(st->p.x - e.pos.x, st->p.y - e.pos.y) > 150.0f) {
                    from_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; hold_.z = e.pos.z;
                    enter(QpPhase::TransFwd, e);
                } else if (st->type == QpStep::HoverAt) {
                    hold_ = Vec3{st->p.x, st->p.y, st->p.z};
                    // над точкою: висіти hold_s (відлік — з моменту, коли апарат над точкою й майже нерухомий)
                    if (!hold_sent_ && std::hypot(e.pos.x - hold_.x, e.pos.y - hold_.y) < 3.0f && std::hypot(e.vel.x, e.vel.y) < 0.8f) { hold_sent_ = true; hold_t0_ = phase_t_; }
                    if (hold_sent_ && phase_t_ - hold_t0_ >= st->hold_s) nextStep(e);
                } else if (st->type == QpStep::Land && std::hypot(st->p.x - e.pos.x, st->p.y - e.pos.y) > 150.0f) {
                    from_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; hold_.z = e.pos.z;
                    enter(QpPhase::TransFwd, e);
                } else if (st->type == QpStep::Land) {
                    hold_ = Vec3{st->p.x, st->p.y, hold_.z};
                    if (std::hypot(e.pos.x - st->p.x, e.pos.y - st->p.y) < 3.0f) enter(QpPhase::Landing, e);
                } else if (st->type == QpStep::Takeoff) {
                    hold_.z = st->p.z; if (std::fabs(alt - st->p.z) < 1.0f) nextStep(e);
                }
                break;
            }
            case QpPhase::TransFwd: {
                // коптер тримає висоту й курс, маршовий гвинт розганяє; тангаж узгоджено з керми за розкладом α(V); blend на V ≥ v_trans
                const Vec3 wp = st ? st->p : hold_;
                const float bearing = std::atan2(wp.y - e.pos.y, wp.x - e.pos.x);
                if (phase_t_ < 0.05f) yaw_ = bearing;
                const float v = std::fmax(e.airspeed, 0.0f);
                const float a_trim = tecs.alphaTrim(std::fmax(v, 9.0f));
                const float ramp = clampf((v - 4.0f) / (cfg.v_trans - 4.0f), 0.0f, 1.0f);
                tg_.zmode = VtolTargets::Altitude; tg_.pos = Vec3{hold_.x, hold_.y, hold_.z}; tg_.yaw = bearing;
                // вище висоти переходу (крило у вітрі «піднімає») — ніс опускаємо: зайва висота переходить у швидкість, а не в набір
                const float over = clampf(alt - hold_.z - 2.0f, 0.0f, 15.0f);
                tg_.att_override = true; tg_.roll_o = 0; tg_.pitch_down_o = -a_trim * ramp + clampf(0.025f * over, 0.0f, a_trim * ramp + 0.05f);
                tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad;
                if (!blending_ && v >= cfg.v_trans) blending_ = true;
                if (blending_ && v < cfg.v_trans - 2.5f) { blending_ = false; blend_t_ = 0; }   // гістерезис: не гойдатися на порозі
                if (blending_) {
                    if (blend_t_ <= 0.0f) { tecs.reset(alt, v); blend_h_ = alt; }
                    blend_t_ += dt; wv_ = clampf(1.0f - blend_t_ / cfg.blend_time, 0.0f, 1.0f);
                    thrust_cruise_ = fwThrust(e, cfg.v_cruise, dt, blend_h_);   // тримати висоту, що була в момент blend, і розганятись до крейсеру
                    tg_.pos.z = blend_h_;
                    tg_.pitch_down_o = -tecs.pitchUpDemand();
                } else {
                    blend_t_ = 0; wv_ = 1.0f;
                    // розгін: майже повний газ маршового (як ArduPilot у переході), щоб швидко пройти небезпечний діапазон 6…12 м/с
                    thrust_cruise_ = clampf(std::fmax(tecs.thrustTrim(std::fmax(v, 9.0f)) + cfg.af.mass * 1.6f, 0.9f * cfg.cruise_thrust_max), 0.0f, cfg.cruise_thrust_max);
                }
                if (wv_ <= 0.0f) { tg_.att_override = false; enter(QpPhase::Cruise, e); }
                if (phase_t_ > 45.0f) { abortToLanding(e); break; }   // тайм-аут переходу (Q_TRANS_FAIL): датчик швидкості відмовив або тяги не вистачає
                break;
            }            case QpPhase::Cruise: {
                if (!st) {   // місія закінчилась у польоті: летіти додому (L1) і сісти
                    QpStep l; l.type = QpStep::Land; l.p = Vec3{home_.x, home_.y, 0}; mission_ = {l}; step_ = 0; from_ = Vec3{e.pos.x, e.pos.y, e.pos.z};
                    break;
                }
                const Vec3 wp = st->p;
                const float dist = std::hypot(wp.x - e.pos.x, wp.y - e.pos.y);
                if (st->type == QpStep::FlyTo && dist < cfg.accept_radius) { from_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; hold_ = wp; nextStep(e); break; }
                if (st->type == QpStep::Takeoff) { hold_ = Vec3{e.pos.x, e.pos.y, 40.0f}; enter(QpPhase::TransBack, e); break; }   // виконується лише з висіння
                const float gs_now = std::hypot(e.vel.x, e.vel.y);
                // вітер уздовж шляху (земна − повітряна): за вітром гальмувати раніше (інакше проскакує на 80+ м), проти вітру — пізніше (інакше зупиняється далеко й повзе)
                const float w_along = clampf(gs_now - e.airspeed, -12.0f, 12.0f);
                const float d_brake = std::fmax(cfg.brake_distance * 0.8f + 2.0f * gs_now + 8.0f * w_along, 30.0f);
                if ((st->type == QpStep::HoverAt || st->type == QpStep::Land) && dist < d_brake) { hold_ = Vec3{wp.x, wp.y, st->type == QpStep::HoverAt ? wp.z : 25.0f}; enter(QpPhase::TransBack, e); break; }
                cruiseControl(dt, e, wp, -1, false, wp.z > 1 && (st->type == QpStep::FlyTo || st->type == QpStep::HoverAt) ? home_.z + (st->type == QpStep::HoverAt ? std::fmax(wp.z, 25.0f) : wp.z) : -1.0f);   // проміжна точка маршруту — зі своєю висотою
                {   // оцінка вітру в установленому польоті: V_земна − V_повітряна·курс (без ковзання)
                    const float wx = e.vel.x - e.airspeed * std::cos(eu.yaw), wy = e.vel.y - e.airspeed * std::sin(eu.yaw);
                    wind_est_.x += (wx - wind_est_.x) * dt / 4.0f; wind_est_.y += (wy - wind_est_.y) * dt / 4.0f;
                }
                // Q_ASSIST: низька швидкість, великий крен/тангаж, низька висота → коптерні мотори підхоплюють
                {   // як Q_ASSIST_ANGLE в ArduPilot: похибка кута, а не абсолютний кут, тримається 0.5 с (Q_ASSIST_DELAY)
                    roll_ref_ += clampf(roll_fw_ - roll_ref_, -1.05f * dt, 1.05f * dt);   // крен, який літак реально встигає відпрацювати (60°/с): стрибок цілі — не «погана орієнтація»
                    const bool att_bad = std::fabs(eu.roll - roll_ref_) > cfg.assist_angle || std::fabs(-eu.pitch_down - tecs.pitchUpDemand()) > cfg.assist_angle;
                    assist_cnt_ = (e.airspeed < cfg.assist_speed || att_bad || alt < cfg.assist_alt) ? assist_cnt_ + dt : 0.0f;
                    assist_ = assist_cnt_ > 0.5f;
                }
                if (wv_ < 0.001f) vtol_.reset();   // коптерні інтеграли не копичаться, поки мотори не потрібні
                if (assist_) wv_ = std::min(1.0f, wv_ + dt / 1.0f); else wv_ = std::fmax(0.0f, wv_ - dt / cfg.blend_time);
                tg_.att_override = false; tg_.hmode = VtolTargets::Velocity; tg_.zmode = VtolTargets::Altitude; tg_.pos.z = alt;
                tg_.vel_xy = Vec3{e.vel.x, e.vel.y, 0}; tg_.yaw = eu.yaw;
                break;
            }
            case QpPhase::TransBack: {
                // A: літак гальмує (TECS: V_dem → v_back, висота плаває), на V < v_back+2 гвинти плавно вмикаються (2 с), кути узгоджено з керми;
                // B: коптер гальмує швидкістю-ціллю до зависання над точкою (нахил носа вгору обмежено), курс фіксується
                const Vec3 wp = hold_;
                const float bearing = std::atan2(wp.y - e.pos.y, wp.x - e.pos.x);
                const float gs = std::hypot(e.vel.x, e.vel.y);
                if (!back_vtol_) {
                    cruiseControl(dt, e, wp, cfg.v_back, true);
                    if (e.airspeed < cfg.v_back + 2.0f || phase_t_ > 25.0f) wv_ = std::min(1.0f, wv_ + dt / 2.0f);
                    tg_.att_override = true; tg_.roll_o = roll_fw_; tg_.pitch_down_o = -tecs.pitchUpDemand();
                    tg_.zmode = VtolTargets::Altitude; tg_.pos = Vec3{wp.x, wp.y, back_h_}; tg_.yaw = eu.yaw;
                    tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad;
                    if (wv_ >= 1.0f) { back_vtol_ = true; tg_.att_override = false; v_ref_ = gs; yaw_ = eu.yaw; back_t0_ = phase_t_; }
                    if (std::hypot(wind_est_.x, wind_est_.y) > 2.5f) intoWind_ = true;
                    break;
                }
                if (phase_t_ > 150.0f) { abortToLanding(e); break; }   // не вдалось зависнути над точкою (вітер): сісти де є
                if (gps_failsafe_) { abortToLanding(e); break; }   // без позиції гальмувати нема за чим: сідаємо вертикально
                v_ref_ = std::fmax(0.0f, std::min(v_ref_, gs) - cfg.back_decel * dt);   // конвертоплан гальмує нахилом гондол назад — швидше
                tg_.att_blend = clampf(1.0f - (phase_t_ - back_t0_) / 1.5f, 0.0f, 1.0f);
                const float d = std::hypot(wp.x - e.pos.x, wp.y - e.pos.y);
                if (intoWind_) yaw_ = std::atan2(-wind_est_.y, -wind_est_.x);   // носом проти вітру: крило допомагає, флюгер стабільний
                tg_.zmode = VtolTargets::Altitude; tg_.pos = Vec3{wp.x, wp.y, hold_.z}; tg_.yaw = yaw_;
                tg_.max_tilt = 18 * kDeg2Rad; tg_.pitch_up_limit = 14 * kDeg2Rad;
                tg_.v_xy_max = clampf(1.5f + 0.08f * d, 2.5f, 7.0f);   // зупинився далеко від точки (вітер) — підлітати швидше, а не повзти 2.5 м/с
                if (gs > tg_.v_xy_max + 1.5f) pos_mode_ = false; else if (gs < tg_.v_xy_max - 0.3f) pos_mode_ = true;   // гістерезис швидкість ↔ позиція
                if (!pos_mode_) { tg_.hmode = VtolTargets::Velocity; tg_.vel_xy = Vec3{v_ref_ * std::cos(bearing), v_ref_ * std::sin(bearing), 0}; }
                else tg_.hmode = VtolTargets::Position;
                thrust_cruise_ = 0;
                if (gs < 2.0f && d < 8.0f) {
                    hold_ = Vec3{wp.x, wp.y, st && st->type == QpStep::HoverAt ? st->p.z : hold_.z};
                    const bool land = !st || st->type == QpStep::Land;
                    if (st && st->type == QpStep::HoverAt && st->hold_s > 0) enterPhaseAfterBack(e);   // висіти над точкою hold_s — у VtolHold
                    else { nextStep(e); if (land) enter(QpPhase::Landing, e); else enterPhaseAfterBack(e); }
                }
                break;
            }            case QpPhase::Landing: {
                const float dz = alt;
                tg_.hmode = VtolTargets::Position; tg_.pos = Vec3{hold_.x, hold_.y, 0}; tg_.yaw = yaw_;
                if (gps_failsafe_) { tg_.att_override = true; tg_.roll_o = 0; tg_.pitch_down_o = 0; }   // без GPS — тримаємо кути рівно
                tg_.zmode = VtolTargets::ClimbRate;
                tg_.climb_rate = -(dz > 6.0f ? 1.5f : (dz > 1.5f ? cfg.landing_rate : 0.35f));
                if (alt - home_.z < 0.6f && std::fabs(e.vel.z) < 0.4f) landed_t_ += dt; else landed_t_ = std::fmax(0.0f, landed_t_ - 2.0f * dt);   // стійко до шуму датчиків
                if (landed_t_ > 0.3f) {   // торкнулися землі: рівно й зменшуємо тягу (land maybe), щоб порив не підкинув і не потягнув по землі
                    tg_.att_override = true; tg_.roll_o = 0; tg_.pitch_down_o = 0;
                    tg_.climb_rate = -1.2f;
                }
                if (landed_t_ > 1.5f) { enter(QpPhase::Landed, e); }
                break;
            }
            case QpPhase::Landed:
                break;
            case QpPhase::FwLand:
                fwLandTick(dt, e, eu);
                break;
            case QpPhase::Manual:   // обробляється в manualDirect()
                break;
        }

        // висіння у вітрі: маршовий гвинт замість нахилу носом униз (Q_FWD_THR_USE) і розворот носом проти вітру (Q_WVANE)
        const float agl = alt - home_.z;
        const bool hover_like = !man && !vtol_.motorFailed() &&
            ((phase_ == QpPhase::Takeoff && agl > 3.0f) || phase_ == QpPhase::VtolHold || (phase_ == QpPhase::Landing && agl > 2.0f) || (phase_ == QpPhase::TransBack && back_vtol_));
        tg_.fwd_pitch_lim = (hover_like && !cruise_fail_) ? cfg.fwd_pitch_lim : -1.0f;
        tg_.fwd_neg_max = cfg.tilt ? cfg.tilt_brake_n : 0.0f;
        // --- коптерні виходи (з вагою blend)
        if (phase_ != QpPhase::OnGround && phase_ != QpPhase::Landed) {
            vtol_.holdIntegrators(lim_ < 0.98f);   // обмежувач струму зрізає тягу — інтегратор висоти не має «накручуватись»
            {   // поправка ШІ — лише коли гвинти несуть усю вагу (w ≈ 1): у переході частку крила вже закладає blend (тяга × w), і поправка рахувала б підйомну силу двічі
                const float g_ai = (!man && !vtol_.motorFailed()) ? clampf((wv_ - 0.95f) / 0.05f, 0.0f, 1.0f) : 0.0f;
                vtol_.a_dist = ai_acc_ * g_ai;
                {
                    const float sw = phase_ == QpPhase::VtolHold ? clampf((e.airspeed - 5.0f) / 4.0f, 0.0f, 1.0f) : 0.0f;
                    vtol_.setVelGainScale(1 + cfg.wind_kp_boost * sw, 1 + cfg.wind_ki_boost * sw);
                    tg_.wind_pitch_up = phase_ == QpPhase::VtolHold ? cfg.wind_wing_up * clampf((e.airspeed - 6.0f) / 4.0f, 0.0f, 1.0f) + ai_pitch_up_ : 0.0f;
                }
                if (meas_ok_ && phase_ == QpPhase::VtolHold && e.airspeed > 3.0f) {
                    // порив: підйомна сила крила ∝ V². Швидкі зміни V² (з ПВТ) і швидкі зміни виміряної вертикальної сили — регресія в польоті дає,
                    // скільки м/с² вгору дає 1 м²/с² напору; далі зміну V² переводимо в поправку тяги одразу, не чекаючи, поки апарат зрушить по висоті
                    const float v2 = e.airspeed * e.airspeed, fz = e.q.rotate(meas_f_).z / massNominal();
                    const float k3 = std::fmin(1.0f, dt / 3.0f), k20 = std::fmin(1.0f, dt / 20.0f);
                    if (!gust_init_) { v2_lp_ = v2; fz_lp_ = fz; gust_init_ = true; }
                    v2_lp_ += (v2 - v2_lp_) * k3; fz_lp_ += (fz - fz_lp_) * k3;
                    const float hv = v2 - v2_lp_, hf = fz - fz_lp_;
                    g_vv_ += (hv * hv - g_vv_) * k20; g_fv_ += (hf * hv - g_fv_) * k20;
                    gust_k_ = g_vv_ > 4.0f ? clampf(g_fv_ / g_vv_, 0.0f, 0.1f) : gust_k_;
                    hv_f_ += (hv - hv_f_) * std::fmin(1.0f, dt / 0.05f);
                    vtol_.a_dist.z += cfg.gust_ff * gust_k_ * hv_f_ * g_ai;
                } else { gust_init_ = false; hv_f_ = 0; }
                if (meas_ok_ && phase_ == QpPhase::VtolHold) {
                    // знахідка ШІ-пілота (tools/ai_rl.cpp): виміряну зовнішню силу (акселерометр − тяга гвинтів) — частково наперед, не чекаючи інтеграторів.
                    // Горизонталь — лише в сильному вітрі (у слабкому інтегратори встигають, а шум виміру лише заважає), вертикаль — завжди (вантаж, низхідні потоки)
                    const Vec3 a_ext = e.q.rotate(meas_f_) / massNominal();
                    dob_ = dob_ + (a_ext - dob_) * std::fmin(1.0f, dt / cfg.dist_ff_tau);
                    const float kh = cfg.dist_ff_xy * clampf((e.airspeed - 5.0f) / 4.0f, 0.0f, 1.0f);
                    const float kz = cfg.dist_ff_z * clampf((e.airspeed - 5.0f) / 4.0f, 0.0f, 1.0f);
                    vtol_.a_dist = vtol_.a_dist + Vec3{kh * dob_.x, kh * dob_.y, kz * dob_.z} * g_ai;
                } else dob_ = Vec3{0, 0, 0};
            }
            VtolCmd vc = vtol_.update(dt, e, tg_);
            if (hover_like && cfg.fwd_pitch_lim >= 0) thrust_cruise_ = cruise_fail_ ? 0.0f : clampf(vc.fwd_thrust, -tg_.fwd_neg_max, cfg.cruise_thrust_max);
            // флюгер — коли тримаємо точку (гальмування після переходу — не вітер); на зльоті, у висінні й на посадці — і тоді, коли вітер уже зносить:
            // боком до сильного вітру крену (≤ 25°) не вистачає, і без розвороту апарат так і зносило б (конвертоплан: бічна площа V-хвоста й гондол)
            const float gs_wv = std::hypot(e.vel.x, e.vel.y);
            if (hover_like && cfg.weathervane && (gs_wv < 1.5f || (phase_ != QpPhase::TransBack && gs_wv < 8.0f))) {
                wv_roll_f_ += (vc.roll_sp - wv_roll_f_) * dt / 2.0f;
                wv_pitch_f_ += (vc.pitch_down_sp - wv_pitch_f_) * dt / 2.0f;
                float rate = 0;
                if (std::fabs(wv_roll_f_) > 2 * kDeg2Rad) rate = -cfg.wvane_gain * wv_roll_f_;   // крен ліворуч (−) → вітер зліва → курс ліворуч (+)
                if (wv_pitch_f_ < -2 * kDeg2Rad) rate += (wv_roll_f_ <= 0 ? 1.0f : -1.0f) * cfg.wvane_gain * -wv_pitch_f_;   // ніс угору → вітер ззаду → розвертатись
                yaw_ = wrapPi(yaw_ + clampf(rate, -0.35f, 0.35f) * dt);
            } else { wv_roll_f_ = 0; wv_pitch_f_ = 0; }
            const float w = (vtolActive() && !manual_idle_) ? wv_ : 0.0f;
            // нормований blend: тяга й моменти ∝ вага (мікшер перераховується з ваговими величинами)
            if (cfg.tilt) {
                float fz[4] = {0, 0, 0, 0}, tau_tilt = 0;
                if (vtol_.motorFailed()) {   // без одного мотора: тяги вже розподілено керуванням з обертанням, гондоли вертикально
                    if (w > 0.5f) for (int i = 0; i < 4; ++i) fz[i] = vc.thrust_i[i];
                } else if (w > 0.001f) {
                    const float k_react = cfg.tilt_yaw_vector ? 0.3f : 1.0f;   // рискання: 30% — реактивний момент, 70% — нахил гондол
                    const MixerResult m = mixQuadX(vc.thrust * w, Vec3{vc.torque.x * w, vc.torque.y * w, vc.torque.z * w * k_react}, cfg.af.mixer);
                    for (int i = 0; i < 4; ++i) fz[i] = m.thrust[i];
                    tau_tilt = vc.torque.z * w * (1 - k_react);
                }
                allocateTilt(dt, e, fz, tau_tilt, w, vtol_.motorFailed() && w > 0.5f, out);
            } else if (vtol_.motorFailed()) {   // без одного мотора: тяги моторів уже розподілено керуванням з обертанням (blend неможливий)
                for (int i = 0; i < 4; ++i) out.duty_vtol[i] = (w > 0.5f && vc.thrust_i[i] > 0.01f) ? cfg.vmap(i).toDuty(vc.thrust_i[i], e.v_bus) : 0.0f;
            } else {
                const MixerResult m = mixQuadX(vc.thrust * w, Vec3{vc.torque.x * w, vc.torque.y * w, vc.torque.z * w}, cfg.af.mixer);
                for (int i = 0; i < 4; ++i) out.duty_vtol[i] = (w > 0.001f) ? cfg.vmap(i).toDuty(m.thrust[i], e.v_bus) : 0.0f;
            }
            // Керми літака: активні, коли швидкість достатня (вага blend їх не зменшує, окрім низької швидкості)
            // без мотора й з обертанням керми лише заважали б; у вітрі (флюгер, обертання немає) — тримають рівно вісь, якою мотори не керують
            const bool spinning = vtol_.motorFailed() && phase_ != QpPhase::FwLand && std::fabs(e.omega.z) > 1.5f;
            if (e.airspeed > 6.0f && !spinning) {
                const bool fw = phase_ == QpPhase::Cruise || phase_ == QpPhase::FwLand || (phase_ == QpPhase::TransBack && !back_vtol_);
                float roll_sp = fw ? roll_fw_ : vc.roll_sp;
                float pitch_up = phase_ == QpPhase::FwLand ? fw_pitch_ : (fw ? tecs.pitchUpDemand() : -vc.pitch_down_sp);
                const FwCmd fc = fw_.update(dt, e, roll_sp, pitch_up);
                const float k = clampf((e.airspeed - 6.0f) / 3.0f, 0.0f, 1.0f);
                const float ks = k * (1.0f - 0.6f * wv_);   // коптерні мотори вже тримають кут: керми лише допомагають (без подвійного підсилення)
                out.aileron = fc.aileron * ks; out.elevator = fc.elevator * ks; out.rudder = fc.rudder * ks;
                rud_dt_ = fc.rudder * k;
            } else {
                fw_.reset(); rud_dt_ = 0;
            }
            out.duty_cruise = (!cfg.tilt && thrust_cruise_ > 0.01f) ? cfg.cruise_map.toDuty(thrust_cruise_, std::fmax(e.airspeed, 0.0f), e.v_bus) : 0.0f;
        }
        if (man && manual_idle_) { for (float& d : out.duty_vtol) d = 0; }
        if (cfg.tilt && (phase_ == QpPhase::OnGround || phase_ == QpPhase::Landed)) {   // на землі гондоли вертикально
            for (int i = 0; i < 4; ++i) { out.tilt[i] = tilt_cmd_[i] = 0; tilt_est_[i] += clampf(-tilt_est_[i], -cfg.tilt_rate * dt, cfg.tilt_rate * dt); }
        }
        if (phase_ == QpPhase::Takeoff && phase_t_ < 0.75f) {   // розкрутка моторів на зльоті (як Q_M_SPOOL_TIME): без стрибка струму 150 А
            const float cap = 0.15f + 0.85f * phase_t_ / 0.75f;
            for (float& d : out.duty_vtol) d = std::fmin(d, cap);
        }
        {   // обмежувач струму: при I > i_max множник lim_ швидко спадає (≈ 0.5 с до мінімуму), потім повільно повертається;
            // так само працює захист ESC/BMS — не дає пікам 250–300 А «посадити» напругу й розкачати регулятор
            // після відмови підйомного мотора тяга важливіша за батарею — обмежувач вимкнено
            if (e.bus_current > cfg.i_max && lift_fail_ < 0) lim_ = std::fmax(0.35f, lim_ - 3.0f * dt * (e.bus_current / cfg.i_max - 1.0f + 0.1f));
            else lim_ = std::fmin(1.0f, lim_ + (lift_fail_ >= 0 ? 5.0f : 0.25f) * dt);
            for (float& d : out.duty_vtol) d *= lim_;
            out.duty_cruise *= lim_;
        }
        last_ = out;
        return out;
    }

private:
    VtolController vtol_;
    Vec3 ai_acc_{0, 0, 0};
    FwController fw_;
    VtolTargets tg_;
    QpPhase phase_ = QpPhase::OnGround;
    std::vector<QpStep> mission_;
    size_t step_ = 0;
    float t_ = 0, phase_t_ = 0, wv_ = 1.0f, blend_t_ = 0, v_ref_ = 0, thrust_cruise_ = 0, roll_fw_ = 0, yaw_ = 0, hold_t0_ = 0, landed_t_ = 0;
    bool blending_ = false;
    float blend_h_ = 0, back_h_ = 0, back_t0_ = 0;
    bool pos_mode_ = false, rtl_ = false, gps_failsafe_ = false, pitot_failed_ = false;
    bool armed_ = false, fence_breach_ = false, rc_failsafe_ = false;
    uint32_t pf_fail_ = 0;
    float pf_ok_t_ = 0, rc_lost_t_ = 0, ground_z_ = 0;
    int ground_n_ = 0;
    float pitot_bad_t_ = 0, gps_bad_t_ = 0, lim_ = 1.0f;
public:
    float currentLimit() const { return lim_; }   // 1 — обмеження немає
private:
    float low_t_ = 0, assist_cnt_ = 0, roll_ref_ = 0;
    // контроль підйомних моторів за телеметрією обертів
    float rps_exp_[4] = {0, 0, 0, 0}, dead_t_[4] = {0, 0, 0, 0}, weak_t_[4] = {0, 0, 0, 0};
    int lift_fail_ = -1, lift_weak_ = -1;
    bool cruise_fail_ = false;
    float cruise_bad_t_ = 0;
    // посадка літаком
    int fw_stage_ = 0;
    // конвертоплан: команда й очікуване положення гондол (модель приводу — для тяги вздовж фактичної осі)
    float tilt_cmd_[4] = {0, 0, 0, 0}, tilt_est_[4] = {0, 0, 0, 0}, tilt_bad_t_[4] = {0, 0, 0, 0};
    int tilt_fail_ = -1;
    float mass_nom_ = 0, mass_prev_ = 0, mass_mid_ = 0, mass_t_ = 0, mass_ai_ = 0;
    void adaptMass(float dt, const QpEstimate& e);
    float tilt_stuck_ = 0;   // рад: де заклинило
    float fw_pitch_ = 0, roll_t_ = 0, wv_roll_f_ = 0, wv_pitch_f_ = 0;
    Vec3 fw_a_, fw_dir_;
    Vec3 wind_est_;
    bool intoWind_ = false;
    bool hold_sent_ = false, assist_ = false, tecs_active_ = false, back_vtol_ = false;
    Vec3 home_, hold_, from_;
    Vec3 meas_f_{0, 0, 0}, dob_{0, 0, 0};
    bool meas_ok_ = false, gust_init_ = false, ploc_on_ = false;
    float rud_dt_ = 0;
    Vec3 ploc_{0, 0, 0};
    float ai_pitch_up_ = 0;
    float v2_lp_ = 0, fz_lp_ = 0, g_vv_ = 0, g_fv_ = 0, gust_k_ = 0, hv_f_ = 0;
  public:
    float gustGain() const { return gust_k_; }
  private:
    QpActuators last_;

    bool manual_idle_ = false, manual_prev_ = false;

    // Ручний режим: швидкості-цілі в осях курсу, швидкість підйому, рискання; на землі без команди підйому мотори вимкнено.
    // З літакових фаз (перехід, крейсер) ручне керування не підхоплює — лише зліт/висіння/посадка.
    bool manualTick(float dt, const QpEstimate& e, const Euler& eu, QpActuators& out);

    // Повністю ручне керування: виходи на мотори й рулі без місії (лише обмежувач струму й керування з обертанням після відмови мотора)
    QpActuators manualDirect(float dt, const QpEstimate& e);
    // Вихід із ручного керування в повітрі: на швидкості — літак і повернення додому з посадкою; повільно — висіння тут і продовження місії; на землі — «на землі»
    void leaveManual(const QpEstimate& e);

    bool vtolActive() const { return phase_ != QpPhase::Cruise || wv_ > 0.001f; }

    void enter(QpPhase p, const QpEstimate& e) {
        phase_ = p; phase_t_ = 0;
        if (p == QpPhase::TransFwd) { v_ref_ = std::max(0.0f, std::hypot(e.vel.x, e.vel.y)); blend_t_ = 0; blending_ = false; wv_ = 1.0f; back_vtol_ = false; }
        if (p == QpPhase::Cruise || p == QpPhase::TransBack) vtol_.reset();
        if (p == QpPhase::Cruise) { wv_ = 0.0f; back_vtol_ = false; roll_fw_ = 0; }
        if (p == QpPhase::TransBack) { intoWind_ = false; back_vtol_ = false; pos_mode_ = false; tg_.att_blend = 0; v_ref_ = std::hypot(e.vel.x, e.vel.y); back_h_ = e.pos.z; }
        // посадка одразу після переходу назад: прибрати змішування кутів і обмеження нахилу гальмування — інакше апарат лишався з носом,
        // «замороженим» угору (att_blend не встигав згаснути), і на посадці у вітрі його зносило
        if (p == QpPhase::Landing) { hold_ = Vec3{hold_.x, hold_.y, e.pos.z}; wv_ = 1.0f; landed_t_ = 0; tg_.att_override = false; tg_.att_blend = 0; tg_.v_xy_max = 5.0f; tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad; }
        if (p == QpPhase::VtolHold) { wv_ = 1.0f; tg_.pos = hold_; }
    }
    void goHome(const QpEstimate& e) {   // як RTL: летіти додому й сісти
        QpStep l; l.type = QpStep::Land; l.p = Vec3{home_.x, home_.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false; from_ = Vec3{e.pos.x, e.pos.y, e.pos.z};
        if (phase_ == QpPhase::Takeoff) abortToLanding(e);
    }
    void abortToLanding(const QpEstimate& e) {
        thrust_cruise_ = 0;   // маршовий гвинт не має штовхати апарат під час вертикальної посадки
        QpStep l; l.type = QpStep::Land; l.p = Vec3{e.pos.x, e.pos.y, 0}; mission_ = {l}; step_ = 0;
        tg_.att_override = false; tg_.att_blend = 0; tg_.v_xy_max = 5.0f; tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad;
        hold_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; vtol_.reset(); enter(QpPhase::Landing, e);
    }
    void enterPhaseAfterBack(const QpEstimate& e) { phase_ = QpPhase::VtolHold; phase_t_ = 0; wv_ = 1.0f; hold_sent_ = false; tg_.att_blend = 0; tg_.v_xy_max = 5.0f; tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad; (void)e; }

    void nextStep(const QpEstimate& e) {
        ++step_; hold_sent_ = false;
        if (phase_ == QpPhase::Takeoff) { phase_ = QpPhase::VtolHold; phase_t_ = 0; wv_ = 1.0f; }
        else if (phase_ == QpPhase::Cruise) { /* лишаємось у літаку для наступного кроку */ phase_t_ = 0; }
        (void)e;
    }

    // Контроль підйомних моторів: оберти з телеметрії ESC проти очікуваних за duty (з інерцією ротора ≈ 80 мс).
    // Повна відмова — оберти < 60% очікуваних 30 мс (≈ 0.09 с після відмови: кожні 0.1 с затримки — це вже ≈ 5° нахилу, який потім не виправити);
    // слабкий мотор — < 85% протягом 1 с.
    // Мотор, що не крутиться (крейсер), перевірити неможливо — відмова виявляється, коли мотори знову потрібні.
    void monitorMotors(float dt, const QpEstimate& e);
    // Контроль приводів нахилу за датчиком положення сервоприводу: фактичний кут розходиться з очікуваним (модель приводу) > 15° протягом 0.4 с —
    // привід заклинив. Гондола лишається там, де стала: у висінні/переході — переривання й посадка коптером (сили рахуються з фактичним кутом);
    // у польоті літаком з гондолою вперед (> 60°) — посадка літаком (вертикально з однією гондолою вперед не сісти).
    void monitorTilt(float dt, const QpEstimate& e);
    void onLiftMotorDead(int i, const QpEstimate& e);
    void onLiftMotorWeak(int i, const QpEstimate& e);

    // Посадка літаком (підйомні мотори не використовуються): захід проти вітру, глісада fw_glide_deg, вирівнювання з 3 м, пробіг на ніжках.
    void enterFwLand(const QpEstimate& e) {
        phase_ = QpPhase::FwLand; phase_t_ = 0; fw_stage_ = 0; wv_ = 0; assist_ = false; assist_cnt_ = 0;
        tg_.att_override = false; tg_.att_blend = 0; back_vtol_ = false; blending_ = false;
        const float wn = std::hypot(wind_est_.x, wind_est_.y);
        Vec3 u = wn > 1.5f ? Vec3{-wind_est_.x / wn, -wind_est_.y / wn, 0} : Vec3{home_.x - e.pos.x, home_.y - e.pos.y, 0};
        const float un = std::hypot(u.x, u.y);
        u = un > 1e-3f ? Vec3{u.x / un, u.y / un, 0} : Vec3{1, 0, 0};
        fw_dir_ = u;
        const float h0 = 30.0f, len = h0 / std::tan(cfg.fw_glide_deg * kDeg2Rad);
        // точка прицілювання глісади — за 50 м до дому: вирівнювання й видержування з’їдають ≈ 40–60 м
        fw_a_ = Vec3{home_.x - u.x * (len + 50.0f), home_.y - u.y * (len + 50.0f), h0};
        from_ = Vec3{e.pos.x, e.pos.y, e.pos.z};
        if (e.airspeed > 6.0f) tecs.reset(e.pos.z, e.airspeed);
    }
    void fwLandTick(float dt, const QpEstimate& e, const Euler& eu) {
        wv_ = 0;
        const float v = std::fmax(e.airspeed, 5.0f);
        const float agl = e.pos.z - home_.z;
        if (fw_stage_ == 0) {   // до початку глісади на 30 м
            const float a_lat = gps_failsafe_ ? 0.0f : l1.update(e.pos, e.vel, from_, fw_a_);
            roll_fw_ = clampf(-std::atan(a_lat / kGravity), -30 * kDeg2Rad, 30 * kDeg2Rad);
            thrust_cruise_ = fwThrust(e, cfg.v_cruise, dt, fw_a_.z + home_.z);
            fw_pitch_ = tecs.pitchUpDemand();
            const float d = std::hypot(fw_a_.x - e.pos.x, fw_a_.y - e.pos.y);
            const float hdg_err = std::fabs(wrapPi(std::atan2(e.vel.y, e.vel.x) - std::atan2(fw_dir_.y, fw_dir_.x)));
            if (d < 40.0f || (d < 120.0f && hdg_err < 0.5f)) { fw_stage_ = 1; tecs.reset(e.pos.z, e.airspeed); }
        } else if (fw_stage_ == 1) {   // глісада: висота-ціль за відстанню до точки торкання
            const Vec3 b{home_.x + fw_dir_.x * 200.0f, home_.y + fw_dir_.y * 200.0f, 0};
            const float a_lat = gps_failsafe_ ? 0.0f : l1.update(e.pos, e.vel, fw_a_, b);
            roll_fw_ = clampf(-std::atan(a_lat / kGravity), -20 * kDeg2Rad, 20 * kDeg2Rad);
            const float s = (home_.x - e.pos.x) * fw_dir_.x + (home_.y - e.pos.y) * fw_dir_.y - 50.0f;   // до точки прицілювання вздовж осі
            const float h_dem = home_.z + clampf(s * std::tan(cfg.fw_glide_deg * kDeg2Rad), 0.0f, 30.0f);
            thrust_cruise_ = fwThrust(e, cfg.fw_land_speed, dt, h_dem);
            fw_pitch_ = tecs.pitchUpDemand();
            if (agl < 3.0f) fw_stage_ = 2;
        } else if (fw_stage_ == 2) {   // вирівнювання: тяга 0, крила рівно, тангаж тримає зниження ≈ 0.5 м/с
            thrust_cruise_ = 0;
            roll_fw_ = clampf(0.5f * roll_fw_, -5 * kDeg2Rad, 5 * kDeg2Rad);
            const float alpha = tecs.alphaTrim(std::fmax(v, 9.0f));
            fw_pitch_ = clampf(alpha + std::atan2(-0.5f, v) + 0.08f * (-0.5f - e.vel.z), -2 * kDeg2Rad, 12 * kDeg2Rad);
            if (agl < 0.35f && std::fabs(e.vel.z) < 0.6f) { fw_stage_ = 3; roll_t_ = 0; }
        } else {   // пробіг: керми нейтрально, чекаємо зупинки
            thrust_cruise_ = 0; roll_fw_ = 0; fw_pitch_ = 0;
            roll_t_ += dt;
            if (std::hypot(e.vel.x, e.vel.y) < 0.5f && roll_t_ > 1.0f) enter(QpPhase::Landed, e);
        }
        (void)eu;
    }

    // Конвертоплан: бажані сили кожного гвинта — вертикальна fz[i] (мікшер коптера × вага blend) і поздовжня (тяга вперед thrust_cruise_,
    // плюс різницева для рискання tau_yaw) → кут гондоли θ = atan2(Fx, Fz) і тяга гвинта вздовж ОЧІКУВАНОЇ осі (привід ≈ 90°/с: поки гондола
    // повертається, вертикальна складова має пріоритет — висота не провалюється). spin — керування з обертанням після відмови: гондоли вертикально.
    void allocateTilt(float dt, const QpEstimate& e, const float fz[4], float tau_yaw, float w, bool spin, QpActuators& out) {
        static const bool kFront[4] = {true, false, true, false};
        bool alive[4];
        for (int i = 0; i < 4; ++i) alive[i] = i != lift_fail_;
        // частки тяги вперед
        float share[4] = {0, 0, 0, 0};
        const float F = thrust_cruise_;
        const bool plane = w < 0.05f;
        int nf = 0, nr = 0;
        for (int i = 0; i < 4; ++i) { if (alive[i]) (kFront[i] ? nf : nr) += 1; }
        float f_front = F, f_rear = 0;
        if (!plane || cfg.cruise_rear == 1 || nf == 0) { f_front = F * nf / float(std::max(nf + nr, 1)); f_rear = F - f_front; }
        else if (cfg.cruise_rear == 2 && F > cfg.front_cruise_cap * nf) { f_front = cfg.front_cruise_cap * nf; f_rear = F - f_front; }
        if (nr == 0) { f_front = F; f_rear = 0; }
        if (plane && lift_fail_ >= 0) {   // мотор відмовив: тягне лише симетрична пара (інакше несиметрична тяга крутить літак по рисканню сильніше, ніж утримає кермо)
            if (nf == 2) { f_front = F; f_rear = 0; } else { f_front = 0; f_rear = F; }
        }
        for (int i = 0; i < 4; ++i) if (alive[i]) share[i] = kFront[i] ? f_front / float(std::max(nf, 1)) : f_rear / float(std::max(nr, 1));
        // рискання: сила вперед ∝ −y (момент −y·Fx), лише в режимі коптера
        float sy2 = 0;
        for (int i = 0; i < 4; ++i) if (alive[i]) sy2 += cfg.motor_y[i] * cfg.motor_y[i];
        const bool yaw_tilt = cfg.tilt_yaw_vector && !plane && !spin && sy2 > 1e-4f;
        // керма напряму немає (yaw_diff_thrust): у польоті літаком демпфер рискання — різницею тяги лівих і правих моторів (як RUDD_DT_GAIN)
        const float tau_dt = (plane && cfg.yaw_diff_thrust && lift_fail_ < 0 && sy2 > 1e-4f) ? cfg.yaw_dt_gain * rud_dt_ : 0.0f;
        for (int i = 0; i < 4; ++i) {
            const float T_max = cfg.vmap(i).maxThrust();
            float fx = share[i] + (yaw_tilt && alive[i] ? -tau_yaw * cfg.motor_y[i] / sy2 : 0.0f) - tau_dt * cfg.motor_y[i] / std::fmax(sy2, 1e-4f);
            if (tau_dt != 0.0f) fx = std::fmax(fx, 0.0f);
            float cmd;
            if (spin) { cmd = 0; fx = 0; }
            else if (std::hypot(fx, fz[i]) > 0.2f) cmd = std::atan2(fx, fz[i]);
            else cmd = plane ? kPi / 2 : tilt_cmd_[i];   // сил майже немає: у польоті літаком — уперед, у висінні — як було
            cmd = clampf(cmd, cfg.tilt_min, cfg.tilt_max);
            tilt_cmd_[i] = cmd;
            const float d = (cmd - tilt_est_[i]) * (1 - std::exp(-dt / cfg.tilt_tau));
            tilt_est_[i] += clampf(d, -cfg.tilt_rate * dt, cfg.tilt_rate * dt);
            if (i == tilt_fail_) tilt_est_[i] = tilt_stuck_;   // заклинена гондола: тяга — вздовж фактичної осі
            const float c = std::cos(tilt_est_[i]), sn = std::sin(tilt_est_[i]);
            float T = fz[i] * c + fx * sn;
            if (fz[i] > 0.05f && c > 0.25f) T = std::fmax(T, std::fmin(fz[i] / c, T_max));
            T = alive[i] ? clampf(T, 0.0f, T_max) : 0.0f;
            const float v_ax = std::fmax(e.airspeed * sn, 0.0f);   // потік уздовж осі гвинта
            out.duty_vtol[i] = T > 0.01f ? cfg.vmap2(i).toDuty(T, v_ax, e.v_bus) : 0.0f;
            out.tilt[i] = cmd;
        }
    }

    float fwThrust(const QpEstimate& e, float v_dem, float dt, float h_dem) {
        tecs.update(dt, cfg.af.mass, e.pos.z, e.vel.z, std::fmax(e.airspeed, 5.0f), h_dem, v_dem);
        return tecs.thrustDemand();
    }

    // літакова навігація: L1 → крен, TECS → тангаж і тяга
    void cruiseControl(float dt, const QpEstimate& e, const Vec3& wp, float v_dem = -1, bool braking = false, float h_override = -1) {
        // у сильному вітрі — швидше, щоб проти вітру земна швидкість лишалась ≥ 8 м/с (запас керованості й часу польоту)
        float vd = v_dem > 0 ? v_dem : clampf(std::hypot(wind_est_.x, wind_est_.y) + 8.0f, cfg.v_cruise, std::fmax(cfg.v_max, cfg.v_cruise));
        if (v_dem <= 0 && !braking && energy_v_dem > 0) vd = clampf(energy_v_dem, std::fmax(cfg.assist_speed + 1.5f, 1.3f * cfg.af.v_stall), cfg.v_max);
        float h_dem = h_override > 0 ? h_override : (braking ? e.pos.z : cfg.cruise_alt);
        if (h_override <= 0 && !braking && energy_alt > 0) h_dem = home_.z + energy_alt;
        const float a_lat = gps_failsafe_ ? 0.0f : l1.update(e.pos, e.vel, from_, wp);
        const float vg = std::fmax(std::hypot(e.vel.x, e.vel.y), 5.0f);
        roll_fw_ = clampf(-std::atan(a_lat / kGravity), -35 * kDeg2Rad, 35 * kDeg2Rad);   // a_left > 0 → крен ліворуч (φ < 0)
        (void)braking; (void)vg;
        thrust_cruise_ = fwThrust(e, vd, dt, h_dem);
        tecs_active_ = true;
    }
};

}  // namespace fc

#include "qp_monitors.h"
#include "qp_manual_ctl.h"
