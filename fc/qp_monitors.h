#pragma once
// qp_monitors.h — нагляд у польоті (частина fc::QpFlight): відмова й слабкість підйомних моторів, заклинення гондол, оцінка маси.
// Підключається в кінці qp_mission.h; окремо не використовується.

namespace fc {

inline void QpFlight::adaptMass(float dt, const QpEstimate& e) {
    if (mass_nom_ <= 0) { mass_nom_ = cfg.af.mass; mass_prev_ = mass_mid_ = cfg.af.mass; }
    mass_t_ += dt;
    if (mass_t_ > 1.5f) { mass_t_ = 0; mass_prev_ = mass_mid_; mass_mid_ = cfg.af.mass; }   // оцінка 1.5…3 с тому
    if (!mass_adapt || manual.on || lift_fail_ >= 0 || lift_weak_ >= 0 || vtol_.motorFailed()) return;
    // рівний набір висоти на зльоті або усталене висіння; не під час гальмування, зниження й посадки (там інтегратор зайнятий іншим)
    const bool ok_phase = (phase_ == QpPhase::Takeoff && phase_t_ > 2.0f && e.pos.z - home_.z > 3.0f && std::fabs(e.vel.z) < 2.5f)
                       || (phase_ == QpPhase::VtolHold && phase_t_ > 3.0f && std::fabs(e.vel.z) < 0.6f);
    if (!ok_phase || wv_ < 0.99f || e.airspeed > 5.0f || e.omega.norm() > 0.6f) return;   // різкі рухи (порив, відмова) — не про масу
    if (ai_mass_meas > 0) {   // маса регулятора застигає (вантаж уже компенсує порада ШІ), оцінка для показу й журналу — з виміряної сили
        const float mt = clampf(ai_mass_meas, 0.8f * mass_nom_, 2.0f * mass_nom_);
        mass_ai_ = mass_ai_ > 0 ? mass_ai_ + (mt - mass_ai_) * std::fmin(1.0f, dt / 4.0f) : cfg.af.mass;
        return;
    }
    const float I = vtol_.verticalIntegral(), m = cfg.af.mass;
    // на наборі висоти гвинти тягнуть трохи слабше, ніж у таблиці (потік уздовж осі) — там лише груба поправка (зона нечутливості 35%: на наборі інтегратор і без вантажу показує +20…30%),
    // точно — в усталеному висінні
    const float dead = phase_ == QpPhase::Takeoff ? 0.35f * kGravity : 0.0f;
    const float Ie = std::fabs(I) > dead ? I - std::copysign(dead, I) : 0.0f;
    const float mn = clampf(m + m * 0.35f * dt * clampf(Ie / kGravity, -0.3f, 0.3f), 0.8f * mass_nom_, 2.0f * mass_nom_);
    if (std::fabs(mn - m) < 1e-7f) return;
    vtol_.shiftVerticalIntegral(-kGravity * (mn - m) / mn);
    setMass(mn, cfg.af.inertia);
    mass_ai_ = mn;
}

inline void QpFlight::monitorMotors(float dt, const QpEstimate& e) {
    if (cfg.cruise_rps_map.valid() && e.cruise_rps >= 0 && !cruise_fail_) {   // маршовий: оберти < 60% очікуваних (за duty й швидкістю) при duty > 0.35 протягом 0.5 с
        // очікувані оберти — для поточної повітряної швидкості: мертвий гвинт авторотує, і статична карта виявляла б відмову надто пізно
        const float ex = cfg.cruise_rps_map.fromDuty(last_.duty_cruise, std::fmax(e.airspeed, 0.0f), e.v_bus);
        cruise_bad_t_ = (last_.duty_cruise > 0.35f && e.cruise_rps < 0.6f * ex) ? cruise_bad_t_ + dt : 0.0f;
        if (cruise_bad_t_ > 0.5f) {
            cruise_fail_ = true;
            // без маршового гвинта літаком не долетіти: перехід у коптер і посадка поблизу
            if (phase_ == QpPhase::Cruise || phase_ == QpPhase::TransFwd) {
                QpStep l; l.type = QpStep::Land; l.p = Vec3{e.pos.x, e.pos.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false;
                hold_ = Vec3{e.pos.x, e.pos.y, home_.z + 30.0f};
                if (phase_ == QpPhase::Cruise) enter(QpPhase::TransBack, e); else abortToLanding(e);
            } else if (phase_ == QpPhase::VtolHold || phase_ == QpPhase::Takeoff) {
                abortToLanding(e);
            }
        }
    }
    if (!cfg.vtol_rps_map.valid() || e.vtol_rps[0] < 0) return;
    for (int i = 0; i < 4; ++i) {
        const float rps_min = 0.25f * cfg.vrps(i).fromDuty(1.0f, e.v_bus);
        const float ex = cfg.vrps(i).fromDuty(last_.duty_vtol[i], e.v_bus);
        rps_exp_[i] += (ex - rps_exp_[i]) * dt / (0.08f + dt);
        // очікування — менше з миттєвого й відфільтрованого: при скиданні газу мотор з активним гальмуванням сповільнюється швидше за модель
        const float ex_lo = std::fmin(ex, rps_exp_[i]);
        if (ex_lo < rps_min || i == lift_fail_) { dead_t_[i] = 0; weak_t_[i] = std::fmax(0.0f, weak_t_[i] - dt); continue; }
        const float ratio = e.vtol_rps[i] / ex_lo;
        dead_t_[i] = ratio < 0.6f ? dead_t_[i] + dt : 0.0f;
        weak_t_[i] = ratio < 0.85f ? weak_t_[i] + dt : std::fmax(0.0f, weak_t_[i] - dt);
        if (dead_t_[i] > 0.03f && lift_fail_ < 0) onLiftMotorDead(i, e);
        else if (weak_t_[i] > 1.0f && lift_weak_ < 0 && lift_fail_ < 0) onLiftMotorWeak(i, e);
    }
}

inline void QpFlight::monitorTilt(float dt, const QpEstimate& e) {
    if (!cfg.tilt || tilt_fail_ >= 0) return;
    for (int i = 0; i < 4; ++i) {
        if (e.tilt_meas[i] < -5) continue;
        tilt_bad_t_[i] = std::fabs(e.tilt_meas[i] - tilt_est_[i]) > 0.26f ? tilt_bad_t_[i] + dt : 0.0f;
        if (tilt_bad_t_[i] > 0.4f) {
            tilt_fail_ = i; tilt_stuck_ = e.tilt_meas[i];
            if (phase_ == QpPhase::OnGround || phase_ == QpPhase::Landed) return;
            const bool plane = phase_ == QpPhase::Cruise || (phase_ == QpPhase::TransFwd && wv_ < 0.5f) || (phase_ == QpPhase::TransBack && !back_vtol_);
            if (plane && tilt_stuck_ > 60 * kDeg2Rad) enterFwLand(e);
            else if (phase_ != QpPhase::Landing && phase_ != QpPhase::FwLand) abortToLanding(e);
            return;
        }
    }
}

inline void QpFlight::onLiftMotorDead(int i, const QpEstimate& e) {
    lift_fail_ = i;
    if (mass_prev_ > 0) setMass(mass_prev_, cfg.af.inertia);   // відмова сама «накрутила» інтегратор — повернути оцінку маси з-перед неї
    vtol_.setFailedMotor(i);
    const bool fast = e.airspeed > cfg.assist_speed;
    if (phase_ == QpPhase::OnGround || phase_ == QpPhase::Landed || e.pos.z - home_.z < 0.5f) { enter(QpPhase::Landed, e); return; }   // на землі — зліт скасовано
    if (fast && (phase_ == QpPhase::Cruise || phase_ == QpPhase::TransFwd || (phase_ == QpPhase::TransBack && !back_vtol_))) { enterFwLand(e); return; }
    if (phase_ != QpPhase::FwLand) abortToLanding(e);   // висіння/мала швидкість: вертикальна посадка з обертанням просто тут
}

inline void QpFlight::onLiftMotorWeak(int i, const QpEstimate& e) {
    lift_weak_ = i;
    if (phase_ == QpPhase::Takeoff || phase_ == QpPhase::VtolHold || (phase_ == QpPhase::TransBack && back_vtol_)) abortToLanding(e);
    else if (phase_ != QpPhase::Landing && phase_ != QpPhase::Landed && phase_ != QpPhase::OnGround && phase_ != QpPhase::FwLand) {
        QpStep l; l.type = QpStep::Land; l.p = Vec3{home_.x, home_.y, 0}; mission_ = {l}; step_ = 0; hold_sent_ = false;   // додому й сісти
    }
}

}  // namespace fc
