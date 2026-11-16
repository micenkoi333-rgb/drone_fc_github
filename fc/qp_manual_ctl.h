#pragma once
// qp_manual_ctl.h — ручне керування (частина fc::QpFlight): QLOITER/QSTABILIZE/QACRO/FBWA/MANUAL і повернення автопілоту.
// Підключається в кінці qp_mission.h; окремо не використовується.

namespace fc {

inline bool QpFlight::manualTick(float dt, const QpEstimate& e, const Euler& eu, QpActuators& out) {
    if (!manual.on) { manual_prev_ = false; manual_idle_ = false; return false; }
    const bool ok = phase_ == QpPhase::OnGround || phase_ == QpPhase::Takeoff || phase_ == QpPhase::VtolHold || phase_ == QpPhase::Landing || phase_ == QpPhase::Landed;
    if (!ok) return false;
    if (!manual_prev_) { manual_prev_ = true; yaw_ = eu.yaw; phase_ = QpPhase::VtolHold; phase_t_ = 0; wv_ = 1.0f; vtol_.reset(); tg_.att_override = false; tg_.att_blend = 0; home_ = e.pos; }
    yaw_ = wrapPi(yaw_ + manual.yaw_rate * 1.2f * dt);
    const float cy = std::cos(yaw_), sy = std::sin(yaw_), v = manual.speed_max;
    tg_.hmode = VtolTargets::Velocity; tg_.vel_xy = Vec3{(manual.fwd * cy - manual.left * sy) * v, (manual.fwd * sy + manual.left * cy) * v, 0};
    tg_.zmode = VtolTargets::ClimbRate; tg_.climb_rate = manual.climb * manual.climb_max; tg_.yaw = yaw_;
    tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad; tg_.v_xy_max = v;
    thrust_cruise_ = 0;
    manual_idle_ = e.pos.z < 0.3f && manual.climb <= 0.05f;
    if (manual_idle_) vtol_.reset();
    (void)out;
    return true;
}

inline QpActuators QpFlight::manualDirect(float dt, const QpEstimate& e) {
    QpActuators out;
    if (phase_ != QpPhase::Manual) { phase_ = QpPhase::Manual; phase_t_ = 0; vtol_.reset(); fw_.reset(); if (home_.norm() < 1e-6f) home_ = e.pos; }
    phase_t_ += dt; t_ += dt;
    const QpManual& m = manual;
    const bool plane = m.mode == QpManual::Fbwa || m.mode == QpManual::Manual;
    float fz[4] = {0, 0, 0, 0};   // конвертоплан: вертикальні сили гвинтів (коптерні режими)
    bool spin = false;
    if (!plane) {
        if (vtol_.motorFailed()) {   // без мотора — лише керування з обертанням: ручки задають напрям тяги
            tg_.hmode = VtolTargets::Velocity; tg_.vel_xy = e.vel; tg_.zmode = VtolTargets::ClimbRate; tg_.climb_rate = (m.throttle - 0.5f) * 4.0f;
            const VtolCmd vc = vtol_.update(dt, e, tg_);
            for (int i = 0; i < 4; ++i) { out.duty_vtol[i] = vc.thrust_i[i] > 0.01f ? cfg.vmap(i).toDuty(vc.thrust_i[i], e.v_bus) : 0.0f; fz[i] = vc.thrust_i[i]; }
            spin = true;
        } else {
            const VtolCmd vc = vtol_.manualUpdate(dt, e, m.mode == QpManual::QAcro, m.shaped(m.roll), m.shaped(m.pitch), m.shaped(m.yaw), m.throttle,
                                                  m.max_angle_deg * kDeg2Rad, m.acro_rate_dps * kDeg2Rad);
            for (int i = 0; i < 4; ++i) { out.duty_vtol[i] = vc.thrust_i[i] > 0.01f ? cfg.vmap(i).toDuty(vc.thrust_i[i], e.v_bus) : 0.0f; fz[i] = vc.thrust_i[i]; }
        }
        out.duty_cruise = cfg.tilt ? 0.0f : clampf(m.pusher, 0.0f, 1.0f);
        // конвертоплан: «маршовий газ» — нахил гондол уперед; від'ємний — назад (гальмування, до tilt_brake_n)
        thrust_cruise_ = m.pusher >= 0 || !cfg.tilt ? clampf(m.pusher, 0.0f, 1.0f) * cfg.cruise_thrust_max : clampf(m.pusher, -1.0f, 0.0f) * cfg.tilt_brake_n;
        if (e.airspeed > 8.0f) {   // на швидкості керми допомагають тримати кути (як QSTABILIZE в ArduPilot)
            const FwCmd fc = fw_.update(dt, e, m.max_angle_deg * kDeg2Rad * m.shaped(m.roll), -m.max_angle_deg * kDeg2Rad * m.shaped(m.pitch));
            out.aileron = 0.5f * fc.aileron; out.elevator = 0.5f * fc.elevator; out.rudder = -0.3f * m.yaw;
        } else fw_.reset();
    } else {
        vtol_.reset();
        if (m.mode == QpManual::Fbwa) {
            const FwCmd fc = fw_.update(dt, e, m.fbwa_bank_deg * kDeg2Rad * m.shaped(m.roll), -m.fbwa_pitch_deg * kDeg2Rad * m.shaped(m.pitch));
            out.aileron = fc.aileron; out.elevator = fc.elevator; out.rudder = clampf(fc.rudder - 0.35f * m.shaped(m.yaw), -0.43f, 0.43f);
        } else {
            const float dmax = 25 * kDeg2Rad;
            out.aileron = dmax * m.shaped(m.roll); out.elevator = dmax * m.shaped(m.pitch); out.rudder = -dmax * m.shaped(m.yaw);   // elevator > 0 — ніс униз; rudder > 0 — ніс ліворуч
        }
        out.duty_cruise = cfg.tilt ? 0.0f : clampf(m.throttle, 0.0f, 1.0f);
        thrust_cruise_ = clampf(m.throttle, 0.0f, 1.0f) * cfg.cruise_thrust_max;
    }
    if (cfg.tilt) allocateTilt(dt, e, fz, 0.0f, plane ? 0.0f : 1.0f, spin, out);   // ручний мікшер уже розклав рискання на реактивний момент
    if (e.bus_current > cfg.i_max && lift_fail_ < 0) lim_ = std::fmax(0.35f, lim_ - 3.0f * dt * (e.bus_current / cfg.i_max - 1.0f + 0.1f));
    else lim_ = std::fmin(1.0f, lim_ + 0.25f * dt);
    for (float& d : out.duty_vtol) d *= lim_;
    out.duty_cruise *= lim_;
    wv_ = plane ? 0.0f : 1.0f;
    last_ = out;
    return out;
}

inline void QpFlight::leaveManual(const QpEstimate& e) {
    const Euler eu = eulerOf(e.q);
    yaw_ = eu.yaw; vtol_.reset(); fw_.reset();
    tg_.att_override = false; tg_.att_blend = 0; tg_.v_xy_max = 5.0f; tg_.max_tilt = 25 * kDeg2Rad; tg_.pitch_up_limit = 25 * kDeg2Rad;
    if (e.pos.z - home_.z < 0.5f) { phase_ = QpPhase::OnGround; phase_t_ = 0; return; }
    if (e.airspeed > cfg.assist_speed + 1.0f) {
        QpStep l; l.type = QpStep::Land; l.p = Vec3{home_.x, home_.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false;
        from_ = Vec3{e.pos.x, e.pos.y, e.pos.z};
        tecs.reset(e.pos.z, e.airspeed); thrust_cruise_ = 0;
        enter(QpPhase::Cruise, e);
    } else {
        hold_ = Vec3{e.pos.x, e.pos.y, e.pos.z}; thrust_cruise_ = 0;
        enter(QpPhase::VtolHold, e);
    }
}

}  // namespace fc
