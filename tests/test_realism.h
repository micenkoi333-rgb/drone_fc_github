#pragma once
// Реалістичні датчики й приводи (sim/qp_servo.h, SensorNoise::realistic) і безпека польоту (fc/qp_safety.h, qp_log.h, qp_params.h).
#include <cstdio>
#include "../fc/qp_params.h"
#include "../sim/qp_flight.h"
#include "../sim/qp_tricks.h"

static void testRealSensors() {
    SECTION("Реалістичні приводи й датчики: машинки під навантаженням, Піто, ESC, енкодери гондол");
    {   // машинка: без навантаження — повна швидкість; під моментом 70% від зупинки — повільніше; понад момент — провертається назад
        ServoSpec sp; sp.rate = 1.57; sp.tau = 0.05; sp.stall_nm = 2.0; sp.deadband = 0.002;
        auto timeTo = [&](double load) {
            ServoActuator s(sp); s.reset(0);
            for (int i = 0; i < 4000; ++i) { s.step(0.001, 1.0, load); if (s.pos() > 0.95) return i * 0.001; }
            return 99.0;
        };
        const double t0 = timeTo(0), t7 = timeTo(-1.4);
        ServoActuator s(sp); s.reset(0.5);
        for (int i = 0; i < 1000; ++i) s.step(0.001, 0.5, 3.0);   // навантаження 1.5× від моменту зупинки
        std::printf("  привід гондоли: поворот на 1 рад без навантаження %.2f с, під 70%% моменту %.2f с; перевантаження 150%% — вихід зсунуло на %.1f°\n", t0, t7, (s.pos() - 0.5) * 57.3);
        CHECK(t0 < 0.8 && t7 > 1.8 * t0 && t7 < 99, "швидкість приводу має падати з навантаженням: %.2f → %.2f с", t0, t7);
        CHECK(s.pos() - 0.5 > 0.1, "перевантажений привід має провертатись");
        ServoActuator h(sp); h.reset(0);
        for (int i = 0; i < 60000; ++i) h.step(0.001, 0.0, 1.6);   // хвилина під 80% моменту
        std::printf("  нагрів під 80%% моменту за 60 с: %.0f °C\n", h.temperature());
        CHECK(h.temperature() > 45 && h.temperature() < 200, "машинка під навантаженням має грітись");
        ServoActuator d(sp); d.reset(0);
        d.step(0.001, 0.001, 0);
        for (int i = 0; i < 200; ++i) d.step(0.001, 0.001, 0);
        CHECK(std::fabs(d.pos()) < 1e-9, "команда в межах мертвої зони не рухає машинку");
    }
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    {   // Піто: у висінні показ ненадійний (шум тиску), у польоті — точний; ESC — затримка й дискретність
        QpFlightRig r(a, tb, 7);
        using S = fc::QpStep; S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {1500, 0, 60}; hv.hold_s = 5;
        r.setMission({tk, hv});
        double hov_max = 0, cr_e2 = 0, rps_e2 = 0, rps_n = 0; int cr_n = 0; double tl_max = 0;
        r.run(160, [&](const QpFlightRig& g) {
            const fc::QpEstimate& e = g.lastEstimate();
            if (g.ctl.phase() == fc::QpPhase::VtolHold || (g.ctl.phase() == fc::QpPhase::Takeoff && g.sim.time() > 10)) hov_max = std::fmax(hov_max, double(e.airspeed));
            if (g.ctl.phase() == fc::QpPhase::Cruise && g.ctl.phaseTime() > 5) { const double d = e.airspeed - g.sim.airspeedVec(Vec3d{}).x; cr_e2 += d * d; ++cr_n; }
            if (g.ctl.phase() == fc::QpPhase::VtolHold) { const double d = e.vtol_rps[0] - g.sim.vtolRps(0); rps_e2 += d * d; rps_n += 1; }
            for (int i = 0; i < 4; ++i) tl_max = std::fmax(tl_max, std::fabs(g.sim.tiltServo(i).load()));
            return false;
        });
        const double cr = std::sqrt(cr_e2 / std::max(cr_n, 1)), rp = std::sqrt(rps_e2 / std::fmax(rps_n, 1));
        std::printf("  Піто у висінні показує до %.1f м/с (самий шум тиску), у крейсері похибка RMS %.2f м/с; ESC: похибка RMS %.2f об/с; момент на осі гондоли до %.2f Н·м\n", hov_max, cr, rp, tl_max);
        CHECK(!r.sim.crashed() && cr_n > 100, "політ із реалістичними датчиками має доходити до крейсера");
        CHECK(hov_max > 0.5 && hov_max < 6.0, "Піто у висінні: шум, але не вигадана швидкість (%.1f м/с)", hov_max);
        CHECK(cr < 0.8, "Піто в крейсері: похибка %.2f м/с", cr);
        CHECK(tl_max > 0.01 && tl_max < 2.9, "навантаження приводу гондоли в межах моменту");
    }
    {   // ослаблений привід гондоли (25% моменту) — політ і посадка все одно безпечні
        QpFlightRig r(a, tb, 9);
        for (int i = 0; i < 4; ++i) r.sim.faults.tilt_strength[i] = 0.25;
        WindConfig w; w.mean20ft = Vec3d{4, 1.5, 0}; w.turbulence = 1.0; r.setWind(w, 5);
        using S = fc::QpStep; S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {600, 0, 40}; hv.hold_s = 5; S ld; ld.type = S::Land;
        r.setMission({tk, hv, ld});
        r.run(300, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
        std::printf("  приводи гондол на 25%% моменту: %s, t %.0f с\n", r.sim.crashed() ? "АВАРІЯ" : r.ctl.statusText(), r.sim.time());
        CHECK(!r.sim.crashed() && r.ctl.phase() == fc::QpPhase::Landed, "слабкі приводи гондол не мають валити апарат");
    }
}

static void testSafety() {
    SECTION("Безпека: армування, геозона, втрата пульта, захист трюків, бортовий журнал, параметри з файлу");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    {   // гондола заклинила під 30° ще на землі — автопілот не армується
        QpFlightRig r(a, tb, 3);
        double t[4] = {0, 0.52, 0, 0}; r.sim.presetTilts(t); r.sim.faults.tilt_stuck[1] = true;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; r.setMission({tk});
        const double z_init = r.sim.state().pos.z;
        r.run(10);
        std::printf("  гондола №2 під 30%s на землі: %s, «%s»\n", "°", r.ctl.armed() ? "АРМОВАНО" : "не армується", r.ctl.alertText().c_str());
        CHECK(!r.ctl.armed() && r.ctl.phase() == fc::QpPhase::OnGround && (r.ctl.preflightFailures() & fc::kPfTiltPos), "несправна гондола має блокувати армування");
        CHECK(r.sim.state().pos.z < z_init + 0.05, "мотори не мають підіймати апарат без армування");
    }
    {   // геозона 250 м: місія на 900 м → повернення додому й посадка
        QpFlightRig r(a, tb, 4);
        r.ctl.safety.fence_radius = 250;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {900, 0, 50}; hv.hold_s = 5; S ld; ld.type = S::Land;
        r.setMission({tk, hv, ld});
        double dmax = 0;
        r.run(300, [&](const QpFlightRig& g) { dmax = std::fmax(dmax, std::hypot(g.sim.state().pos.x, g.sim.state().pos.y)); return g.ctl.phase() == fc::QpPhase::Landed; });
        const double dh = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        std::printf("  геозона 250 м: найдалі %.0f м, сів за %.1f м від дому (%s)\n", dmax, dh, r.ctl.statusText());
        CHECK(r.ctl.fenceBreached() && r.ctl.phase() == fc::QpPhase::Landed && !r.sim.crashed() && dh < 20 && dmax < 450, "вихід за геозону має повертати додому");
    }
    {   // ручне висіння, пульт пропав → автопілот вертає й садить
        QpFlightRig r(a, tb, 5);
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 25}; r.setMission({tk});
        r.run(25);
        r.ctl.manual.on = true; r.ctl.manual.mode = fc::QpManual::Assisted; r.ctl.manual.fwd = 0.8f;
        r.run(10);
        const double d0 = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        r.sfaults.rc_lost = true;
        r.run(120, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
        const double dh = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        std::printf("  пульт зник за %.0f м від дому → %s за %.1f м від дому\n", d0, r.ctl.statusText(), dh);
        CHECK(r.ctl.rcFailsafe() && !r.ctl.manual.on && r.ctl.phase() == fc::QpPhase::Landed && dh < 10 && d0 > 20, "втрата пульта має повертати додому");
    }
    {   // FS_RC_AUTO = 1: пульт зник посеред автоматичної місії → додому й посадка
        QpFlightRig r(a, tb, 8);
        r.ctl.safety.rc_auto_action = 1;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {1500, 0, 50}; hv.hold_s = 5; S ld; ld.type = S::Land;
        r.setMission({tk, hv, ld});
        r.run(60);
        const double d0 = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        r.sfaults.rc_lost = true;
        double dmax = d0;
        r.run(300, [&](const QpFlightRig& g) { dmax = std::fmax(dmax, std::hypot(g.sim.state().pos.x, g.sim.state().pos.y)); return g.ctl.phase() == fc::QpPhase::Landed; });
        const double dh = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        std::printf("  автоматична місія, пульт зник за %.0f м (FS_RC_AUTO=1): далі до %.0f м, %s за %.1f м від дому\n", d0, dmax, r.ctl.statusText(), dh);
        CHECK(r.ctl.rcFailsafe() && r.ctl.phase() == fc::QpPhase::Landed && !r.sim.crashed() && dh < 20 && dmax < 1300, "втрата пульта в авто з FS_RC_AUTO=1 має повертати додому");
    }
    {   // трюк: межа висоти спрацьовує, керування повертається автопілоту, апарат летить далі
        TrickTrainer tr(a, tb);
        auto base = tr.cruiseRig();
        QpFlightRig r = *base;
        fc::TrickPilot pilot;
        pilot.v_cruise = r.ctl.cfg.af.v_cruise; pilot.v_stall = r.ctl.cfg.af.v_stall; pilot.cfg = &r.ctl.cfg;
        pilot.guard.ground_z = r.ctl.home().z; pilot.guard.min_agl = 10.0f;
        pilot.start(fc::Trick::Tumble, defaultTrickParams(fc::Trick::Tumble));
        r.override_act = [&](const fc::QpEstimate& e, fc::QpActuators& act) { return pilot.update(0.002f, e, act); };
        double t_ab = -1;
        bool armed_floor = false;
        for (int i = 0; i < 20000 && pilot.active(); ++i) {   // посеред кульбіта «підлога» раптом вище апарата — перервати просто в обертанні
            r.step();
            if (!armed_floor && pilot.phase() >= 2) { armed_floor = true; pilot.guard.min_agl = float(r.lastEstimate().pos.z - r.ctl.home().z) + 1.0f; }
            if (!pilot.active()) t_ab = i * 0.001;
        }
        r.override_act = nullptr;
        r.ctl.resumeAfterOverride(r.lastEstimate());
        const double zab = r.sim.state().pos.z;
        r.run(25);
        std::printf("  кульбіт, межа висоти посеред трюку: «%s» за %.1f с, далі %s, висота %.0f → %.0f м\n", pilot.abortReason(), t_ab, r.sim.crashed() ? "АВАРІЯ" : r.ctl.statusText(), zab, r.sim.state().pos.z);
        CHECK(pilot.aborted() && !r.sim.crashed() && r.sim.state().pos.z > 20, "захист трюку має переривати його й рятувати апарат");
    }
    {   // бортовий журнал і параметри
        QpFlightRig r(a, tb, 6);
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 20}; S ld; ld.type = S::Land;
        r.setMission({tk, ld});
        r.run(80, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
        bool armed = false, disarmed = false;
        for (const fc::QpLogEvent& ev : r.ctl.flog.events()) { armed |= ev.text == "армовано"; disarmed |= ev.text == "роззброєно"; }
        const bool csv = r.ctl.flog.writeCsv("fc_log_test.csv");
        std::printf("  журнал автопілота: %zu записів, %zu подій (армовано: %s, роззброєно: %s)\n", r.ctl.flog.size(), r.ctl.flog.events().size(), armed ? "так" : "ні", disarmed ? "так" : "ні");
        CHECK(r.ctl.flog.size() > 500 && armed && disarmed && csv, "бортовий журнал має писати стан і події");
        std::remove("fc_log_test.csv");
        fc::QpFlight fl = r.ctl;
        const size_t n = fc::qpParamList(fl).size();
        CHECK(fc::saveQpParams(fl, "fc_test.param"), "збереження параметрів");
        fc::QpFlight f2 = r.ctl;
        f2.cfg.v_cruise = 1; f2.safety.fence_radius = 1;
        { FILE* f = std::fopen("fc_test.param", "ab"); std::fprintf(f, "LAND_RATE = 0.55\nNO_SUCH = 3\n"); std::fclose(f); }
        std::vector<std::string> unk;
        const int got = fc::loadQpParams(f2, "fc_test.param", &unk);
        std::printf("  параметри: %zu у списку, прочитано %d, невідомих %zu\n", n, got, unk.size());
        CHECK(got == int(n) + 1 && unk.size() == 1 && f2.cfg.v_cruise == fl.cfg.v_cruise && f2.safety.fence_radius == fl.safety.fence_radius && std::fabs(f2.cfg.landing_rate - 0.55f) < 1e-6f,
              "параметри з файлу мають застосовуватись");
        std::remove("fc_test.param");
    }
}

static void testHealth() {
    SECTION("Бортова діагностика: пошкоджений гвинт, повільний привід гондоли, центр мас, вітер, енергія на повернення");
    using S = fc::QpStep;
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    auto fly = [&](int sc, ai::HealthReport& mid, double& wx, double& wy) {
        AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
        if (sc == 3) for (MassItem& it : a.items) if (it.name == "battery") it.x += 0.03;   // батарею зсунули на 3 см уперед
        auto r = std::make_unique<QpFlightRig>(a, tb, 5);
        WindConfig w; w.mean20ft = Vec3d{5, 2, 0}; w.turbulence = 1.0; r->setWind(w, 3);
        if (sc == 1) r->sim.faults.prop_scale[2] = 0.85;
        if (sc == 2) r->sim.tiltServo(1).spec().rate *= 0.4;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {1200, 0, 50}; hv.hold_s = 15; S ld; ld.type = S::Land;
        r->setMission({tk, hv, ld});
        bool got = false; double sx = 0, sy = 0, n = 0;
        r->run(400, [&](const QpFlightRig& g) {
            if (g.ctl.phase() == fc::QpPhase::Cruise && g.ctl.phaseTime() > 20) { const Vec3d ww = g.sim.windWorld(); sx += ww.x; sy += ww.y; n += 1; if (!got && g.ctl.phaseTime() > 30) { got = true; mid = g.health.r; } }
            return g.ctl.phase() == fc::QpPhase::Landed || (sc == 2 && g.health.r.servo_slow >= 0);
        });
        wx = n > 0 ? sx / n : 0; wy = n > 0 ? sy / n : 0;
        return r;
    };
    ai::HealthReport m0, m1, m2, m3; double wx = 0, wy = 0, t = 0;
    auto r0 = fly(0, m0, wx, wy);
    auto r1 = fly(1, m1, t, t);
    auto r2 = fly(2, m2, t, t);
    const ai::HealthReport &h0 = r0->health.r, &h1 = r1->health.r, &h2 = r2->health.r;
    std::printf("  справний: оберти/очікувані %.3f…%.3f, гвинт %d, мотор %d, привід %d; гвинт №3 −15%% тяги: %.3f → виявлено гвинт %d; привід гондоли №2 ×0.4: %.2f рад/с → повільний %d\n",
                std::fmin(std::fmin(h0.motor_ratio[0], h0.motor_ratio[1]), std::fmin(h0.motor_ratio[2], h0.motor_ratio[3])),
                std::fmax(std::fmax(h0.motor_ratio[0], h0.motor_ratio[1]), std::fmax(h0.motor_ratio[2], h0.motor_ratio[3])), h0.prop_damaged, h0.motor_weak, h0.servo_slow,
                h1.motor_ratio[2], h1.prop_damaged, h2.tilt_rate[1], h2.servo_slow);
    CHECK(h0.prop_damaged < 0 && h0.motor_weak < 0 && h0.servo_slow < 0, "справний апарат — без хибних тривог");
    CHECK(h1.prop_damaged == 2, "пошкоджений гвинт має виявлятись (№%d)", h1.prop_damaged + 1);
    CHECK(h2.servo_slow == 1, "повільний привід гондоли має виявлятись");
    for (const char* file : {"aircraft/tiltrotor.ini", "aircraft/qp5.ini"}) {   // центр мас — у тиху погоду (у вітрі момент від крила й хвоста маскує зсув, оцінка завмирає)
        double cg[2] = {0, 0}; bool ok[2] = {false, false};
        const AircraftFile a0 = AircraftFile::load(file);
        const fc::MixerConfig nominal = QpFlightRig(a0, tb, 5).ctl.cfg.af.mixer;
        for (int k = 0; k < 2; ++k) {
            AircraftFile a = a0;
            if (k) for (MassItem& it : a.items) if (it.name == "battery") it.x += 0.03;   // батарею зсунули на 3 см уперед, автопілот про це не знає
            QpFlightRig r(a, tb, 5);
            r.ctl.cfg.af.mixer = nominal;
            WindConfig w; w.mean20ft = Vec3d{1.5, 0.6, 0}; w.turbulence = 1.0; r.setWind(w, 3 + unsigned(k));
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {0, 0, 30}; hv.hold_s = 30;
            r.setMission({tk, hv});
            r.run(70);
            cg[k] = r.health.r.cg_x_mm; ok[k] = r.health.r.cg_ok;
        }
        std::printf("  центр мас (%s): номінал %+.1f мм, батарея +3 см → %+.1f мм (очікувано зсув ≈ +11)\n", file, cg[0], cg[1]);
        CHECK(ok[0] && ok[1] && std::fabs(cg[0]) < 6 && cg[1] - cg[0] > 7 && cg[1] - cg[0] < 16, "оцінка зсуву центру мас");
    }
    {   // у сильному вітрі оцінка центру мас не оновлюється
        CHECK(!h0.cg_ok, "у вітрі 5 м/с оцінка центру мас має мовчати");
    }
    const double we = std::hypot(m0.wind[0] - wx, m0.wind[1] - wy);
    std::printf("  вітер у крейсері: оцінка (%.1f, %.1f), середній справжній (%.1f, %.1f) м/с; енергія: лишилось %.0f Вт·год (%.0f хв), додому треба %.0f → %s\n",
                m0.wind[0], m0.wind[1], wx, wy, m0.wh_left, m0.minutes_left, m0.wh_home, m0.can_return ? "вистачає" : "НЕ вистачає");
    CHECK(m0.wind_ok && we < 1.5, "оцінка вітру: похибка %.1f м/с", we);
    CHECK(m0.can_return && m0.minutes_left > 10 && m0.wh_home > 2 && m0.wh_home < m0.wh_left, "оцінка енергії на повернення");
}

#include "../fc/frames.h"
static void testFrames() {
    SECTION("Системи координат: ENU/FLU (проєкт) ↔ NED/FRD (ArduPilot, MAVLink)");
    double worst = 0;
    for (float r : {-0.6f, 0.0f, 0.4f}) for (float p : {-0.5f, 0.0f, 0.3f}) for (float y : {-2.5f, 0.0f, 1.0f}) {
        const fc::Quat q = fc::Quat::fromEuler(r, p, y);
        const fc::Euler e = fc::eulerOf(q);
        const fc::EulerNed n = fc::eulerNed(q);
        float dy = (1.5707963f - e.yaw) - n.yaw; while (dy > 3.14159f) dy -= 6.2831853f; while (dy < -3.14159f) dy += 6.2831853f;
        worst = std::fmax(worst, std::fabs(n.roll - e.roll) + std::fabs(n.pitch + e.pitch_down) + std::fabs(dy));
        const fc::Vec3 v_flu{1.0f, 2.0f, 3.0f};
        const fc::Vec3 a = fc::enuToNed(q.rotate(v_flu)), b = fc::qEnuFluToNedFrd(q).rotate(fc::fluToFrd(v_flu));
        worst = std::fmax(worst, double(std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z)));
        const fc::Quat back = fc::qNedFrdToEnuFlu(fc::qEnuFluToNedFrd(q));
        worst = std::fmax(worst, double(std::fabs(std::fabs(back.w * q.w + back.x * q.x + back.y * q.y + back.z * q.z) - 1)));
    }
    std::printf("  найбільша розбіжність кутів/векторів після перетворень: %.2e\n", worst);
    CHECK(worst < 1e-4, "перетворення ENU/FLU ↔ NED/FRD мають бути узгоджені");
}

static void testEnergyAi() {
    SECTION("ШІ-енергоменеджер: сам вивчає криву потужності й вітер за висотою, обирає швидкість і висоту крейсеру");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    double tot[2] = {0, 0};
    for (double dir : {90.0, 180.0}) {
        double wh[2] = {0, 0}, t[2] = {0, 0}; bool ok = true; int known = 0;
        for (int on = 0; on < 2; ++on) {
            QpFlightRig r(a, tb, 5);
            const double rad = dir * 3.14159265 / 180;
            WindConfig w; w.mean20ft = Vec3d{8 * std::cos(rad), 8 * std::sin(rad), 0}; w.turbulence = 1.0; r.setWind(w, 8);
            r.energy.on = on;
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {3000, 0, 50}; hv.hold_s = 5; S ld; ld.type = S::Land;
            r.setMission({tk, hv, ld});
            r.run(1500, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
            wh[on] = r.sim.battery().usedWh(); t[on] = r.sim.time();
            ok = ok && !r.sim.crashed() && r.ctl.phase() == fc::QpPhase::Landed;
            if (on) known = r.energy.bins_known;
        }
        std::printf("  місія 3 км туди й назад, вітер 8 м/с %s: %.1f → %.1f Вт·год (%+.0f%%), час %.0f → %.0f с, виміряно швидкостей %d\n",
                    dir == 90 ? "збоку" : "назустріч", wh[0], wh[1], 100 * (wh[1] / wh[0] - 1), t[0], t[1], known);
        CHECK(ok, "з енергоменеджером — політ і посадка без аварії");
        CHECK(wh[1] < 1.02 * wh[0] && known >= 3, "енергоменеджер не має витрачати більше: %.1f проти %.1f Вт·год", wh[1], wh[0]);
        tot[0] += wh[0]; tot[1] += wh[1];
    }
    // одна місія у поривах — надто випадкова (на іншому компіляторі порив інший): економію рахуємо на обох разом
    CHECK(tot[1] < 0.95 * tot[0], "енергоменеджер має заощаджувати щонайменше 5%% енергії на двох місіях: %+.0f%%", 100 * (tot[1] / tot[0] - 1));
}

static void testRoute() {
    SECTION("Маршрут з кількох точок: літаком через точки зі своїми висотами, висіння над останньою, додому");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    QpFlightRig r(a, tb, 5);
    WindConfig w; w.mean20ft = Vec3d{4, 2, 0}; w.turbulence = 1.0; r.setWind(w, 8);
    const float pts[3][3] = {{800, 400, 70}, {1300, -300, 50}, {600, -700, 0}};
    S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30};
    std::vector<S> m{tk};
    for (int i = 0; i < 2; ++i) { S f; f.type = S::FlyTo; f.p = {pts[i][0], pts[i][1], pts[i][2]}; m.push_back(f); }
    S hv; hv.type = S::HoverAt; hv.p = {pts[2][0], pts[2][1], 40}; hv.hold_s = 5; m.push_back(hv);
    S ld; ld.type = S::Land; m.push_back(ld);
    r.setMission(m);
    double dmin[3] = {1e9, 1e9, 1e9}, zat[2] = {0, 0};
    r.run(900, [&](const QpFlightRig& g) {
        const QpState& s = g.sim.state();
        for (int i = 0; i < 3; ++i) {
            const double d = std::hypot(s.pos.x - pts[i][0], s.pos.y - pts[i][1]);
            if (d < dmin[i]) { dmin[i] = d; if (i < 2) zat[i] = s.pos.z - g.sim.derived().cg_z; }
        }
        return g.ctl.phase() == fc::QpPhase::Landed;
    });
    const double dh = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
    std::printf("  3 точки: пройшов за %.0f / %.0f / %.0f м від них, висота на точках %.0f і %.0f м (задано 70 і 50), сів за %.1f м від дому, %.0f с\n",
                dmin[0], dmin[1], dmin[2], zat[0], zat[1], dh, r.sim.time());
    CHECK(!r.sim.crashed() && r.ctl.phase() == fc::QpPhase::Landed && dh < 10, "маршрут: посадка вдома без аварії");
    CHECK(dmin[0] < 60 && dmin[1] < 60 && dmin[2] < 10, "маршрут: апарат проходить кожну точку");
    CHECK(std::fabs(zat[0] - 70) < 12 && std::fabs(zat[1] - 50) < 12, "маршрут: висота кожної точки витримується");
}

#include "../ai/mission_planner.h"
static void testQp5() {
    SECTION("QP-5 TiltQuad v3 (aircraft/qp5.ini): маси, звалювання, тяга, енергія й політ — проти інженерної документації");
    const AircraftFile a = AircraftFile::load("aircraft/qp5.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    const AircraftDerived d = deriveAircraft(a);
    std::printf("  маса %.3f кг (док. 2.370), ЦМ x %+.1f мм від креслення, Ixx/Iyy/Izz %.3f/%.3f/%.3f (док. 0.039/0.207/0.244), звалювання %.2f м/с (док. 11.7)\n",
                d.mass_total, 1000 * d.cg_x, d.inertia.xx, d.inertia.yy, d.inertia.zz, d.v_stall);
    CHECK(std::fabs(d.mass_total - 2.370) < 0.01 && std::fabs(d.v_stall - 11.7) < 0.6, "маса й швидкість звалювання як у документації");
    CHECK(std::fabs(d.inertia.yy / 0.207 - 1) < 0.15 && std::fabs(d.inertia.zz / 0.244 - 1) < 0.15 && std::fabs(d.inertia.xx / 0.039 - 1) < 0.30, "моменти інерції близькі до документації");
    QpFlightRig r0(a, tb, 1);
    const float tf = r0.ctl.cfg.vmap(0).maxThrust() / 9.80665f * 1000, trr = r0.ctl.cfg.vmap(1).maxThrust() / 9.80665f * 1000;
    std::printf("  найбільша тяга: передній %.0f г (док. 1422), задній %.0f г (док. 1016); висіння %.0f Вт (док. 352)\n", tf, trr, r0.hover_w);
    CHECK(std::fabs(tf / 1422 - 1) < 0.15 && std::fabs(trr / 1016 - 1) < 0.15, "тяга моторів як у документації (±15%%)");
    CHECK(r0.hover_w > 0.8 * 352 && r0.hover_w < 1.35 * 352, "потужність висіння в межах точності моделі (−20…+35%% до розрахунку документації)");
    for (size_t i = 0; i < r0.trim_v.size(); ++i) if (std::fabs(r0.trim_v[i] - 14.0f) < 0.1f) {
        std::printf("  крейсер 14 м/с: %.0f Вт від батареї (док. 14.6 м/с — 78 Вт)\n", r0.trim_w[i]);
        CHECK(r0.trim_w[i] > 0.8 * 78 && r0.trim_w[i] < 1.35 * 78, "крейсерська потужність у межах точності моделі");
    }
    using S = fc::QpStep;
    for (double wv : {0.0, 10.0}) {
        QpFlightRig r(a, tb, 5);
        r.noise = SensorNoise::realistic();
        WindConfig w; w.mean20ft = Vec3d{-wv, wv * 0.3, 0}; w.turbulence = 1.0; r.setWind(w, 8);
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 40}; S f1; f1.type = S::FlyTo; f1.p = {1200, 0, 50}; S hv; hv.type = S::HoverAt; hv.p = {1200, 300, 50}; hv.hold_s = 10; S ld; ld.type = S::Land;
        r.setMission({tk, f1, hv, ld});
        r.run(900, [&](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed || g.sim.crashed(); });
        const auto& p = r.sim.state().pos;
        std::printf("  місія 1.2 км, вітер %.0f м/с: %s, %.0f с, посадка %.2f м від дому, %.1f Вт·год\n", wv, r.sim.crashed() ? "АВАРІЯ" : (r.ctl.phase() == fc::QpPhase::Landed ? "сів" : "не сів"),
                    r.sim.time(), std::hypot(p.x, p.y), r.sim.battery().usedWh());
        CHECK(!r.sim.crashed() && r.ctl.phase() == fc::QpPhase::Landed && std::hypot(p.x, p.y) < 4, "QP-5 проходить місію: зліт, перехід, літак, висіння, посадка");
    }
}

static void testWingHover() {
    SECTION("Висіння на крилі в сильному зустрічному вітрі: менше енергії, точка тримається не гірше");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    for (double wv : {9.0, 11.0}) {
        double wh[2], pos[2]; bool crash = false;
        for (int on = 0; on < 2; ++on) {
            QpFlightRig r(a, tb, 7);
            r.noise = SensorNoise::realistic();
            if (!on) r.ctl.cfg.wind_wing_up = 0;
            WindConfig w; w.mean20ft = Vec3d{-wv, 0, 0}; w.turbulence = 1.0; r.setWind(w, 17);
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {0, 0, 30}; hv.hold_s = 1e6f;
            r.setMission({tk, hv});
            double t0 = -1, wh0 = 0, e2 = 0; long n = 0;
            r.run(90, [&](const QpFlightRig& g) {
                if (g.ctl.phase() == fc::QpPhase::VtolHold && t0 < 0) { t0 = g.sim.time(); wh0 = g.sim.battery().usedWh(); }
                if (t0 > 0 && g.sim.time() > t0 + 15) { const auto& p = g.sim.state().pos; const fc::Vec3 h = g.ctl.holdPoint(); e2 += (p.x - h.x) * (p.x - h.x) + (p.y - h.y) * (p.y - h.y) + (p.z - h.z) * (p.z - h.z); ++n; }
                return g.sim.crashed() || (t0 > 0 && g.sim.time() > t0 + 60);
            });
            crash = crash || r.sim.crashed();
            wh[on] = r.sim.battery().usedWh() - wh0; pos[on] = std::sqrt(e2 / std::max(n, 1L));
        }
        std::printf("  вітер %.0f м/с (біля землі), 60 с висіння: енергія %.2f → %.2f Вт·год (%+.0f%%), положення RMS %.2f → %.2f м\n", wv, wh[0], wh[1], 100 * (wh[1] / wh[0] - 1), pos[0], pos[1]);
        CHECK(!crash && wh[1] < 0.85 * wh[0] && pos[1] < 1.15 * pos[0] + 0.05, "висіння на крилі: щонайменше на 15%% менше енергії, точка не гірше");
    }
}

static void testStructureRotors() {
    SECTION("Фізика: слід гвинтів на інших гвинтах, пружність рами й автопошук її частоти");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    struct R { double wh, t, d, rear_in, front_in; bool crash; float n0, n1; };
    auto fly = [&](double flex, bool detect, bool interf, double fr, double fp) {
        QpFlightRig r(a, tb, 5);
        r.noise = SensorNoise::realistic();
        r.sim.setRotorInterference(interf);
        r.sim.setFlexScale(flex);
        if (fr > 0) r.sim.setFlexModes(fr, fp);
        r.nav.struct_detect = detect;
        WindConfig w; w.mean20ft = Vec3d{-5, 0, 0}; w.turbulence = 1.0; r.setWind(w, 8);
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {1000, 0, 50}; hv.hold_s = 10; S ld; ld.type = S::Land;
        r.setMission({tk, hv, ld});
        R o{0, 0, 0, 0, 0, false, 0, 0};
        r.run(500, [&](const QpFlightRig& g) {
            for (int i = 0; i < 4; ++i) { const double v = g.sim.interferenceInflow(i); (i % 2 == 0 ? o.front_in : o.rear_in) = std::fmax(i % 2 == 0 ? o.front_in : o.rear_in, v); }
            return g.ctl.phase() == fc::QpPhase::Landed || g.sim.crashed();
        });
        o.wh = r.sim.battery().usedWh(); o.t = r.sim.time(); o.d = std::hypot(r.sim.state().pos.x, r.sim.state().pos.y);
        o.crash = r.sim.crashed() || r.ctl.phase() != fc::QpPhase::Landed; o.n0 = r.nav.struct_notch_hz[0]; o.n1 = r.nav.struct_notch_hz[1];
        return o;
    };
    const R base = fly(0, true, true, 0, 0);
    std::printf("  жорстка рама: %.1f Вт·год, %.0f с; слід крізь диски: передні %.2f, задні %.2f м/с; режектор %.1f/%.1f Гц\n", base.wh, base.t, base.front_in, base.rear_in, base.n0, base.n1);
    CHECK(!base.crash && base.n0 < 1 && base.n1 < 1, "жорстка рама: долетів, хибного режектора немає");
    CHECK(base.rear_in > 0.5 && base.rear_in > 3 * base.front_in, "у переходах слід передніх гвинтів проходить крізь задні (а не навпаки)");
    const R no = fly(0, true, false, 0, 0);
    std::printf("  без взаємного сліду гвинтів: %.1f Вт·год\n", no.wh);
    CHECK(!no.crash && std::fabs(no.wh / base.wh - 1) < 0.05, "взаємний слід гвинтів у цій схемі змінює витрату менш ніж на 5%%");
    const R soft = fly(8, false, true, 0, 0);
    std::printf("  м'яка рама без режектора: %.1f Вт·год, %.0f с, %s\n", soft.wh, soft.t, soft.crash ? "не сів / аварія" : "сів");
    CHECK(soft.crash || soft.wh > 1.3 * base.wh, "м'яка рама без режектора: коливання замикаються через регулятор");
    for (double f0 : {16.0, 12.0}) {
        const double f1 = f0 == 16.0 ? 22.0 : 30.0;
        const R sd = fly(8, true, true, f0 == 16.0 ? 0 : f0, f1);
        std::printf("  м'яка рама %.0f/%.0f Гц з автопошуком: %.1f Вт·год, режектор %.1f/%.1f Гц, %s\n", f0, f1, sd.wh, sd.n0, sd.n1, sd.crash ? "аварія" : "сів");
        CHECK(!sd.crash && std::fabs(sd.n0 - f0) < 1.5 && std::fabs(sd.n1 - f1) < 2.0 && sd.wh < 1.2 * base.wh, "автопошук знаходить частоти рами і прибирає коливання");
    }
}

static void testPlannerTuneAnomaly() {
    SECTION("ШІ: енергія маршруту до зльоту, самоналаштування регуляторів, передбачення поломок");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    using S = fc::QpStep;
    {   // прогноз енергії: у штиль — точно, у вітер — з запасом (не менше справжнього)
        for (double wind : {0.0, 8.0}) {
            QpFlightRig r(a, tb, 5);
            WindConfig w; w.mean20ft = Vec3d{-wind, 0, 0}; w.turbulence = 1.0; r.setWind(w, 8);
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {3000, 0, 50}; hv.hold_s = 5; S ld; ld.type = S::Land;
            const std::vector<S> m{tk, hv, ld};
            r.setMission(m);
            const float k = float(std::pow(60 / 6.1, 0.14));
            const ai::MissionEnergy pe = ai::planMissionEnergy(m, fc::Vec3{0, 0, 0}, fc::Vec3{float(-wind) * k, 0, 0}, r.ctl.cfg, float(r.sim.mass()), ai::MissionPlannerCfg{});
            r.run(1500, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
            const double wh = r.sim.battery().usedWh();
            std::printf("  прогноз енергії, вітер %.0f м/с: %.1f Вт·год, %.1f хв — у польоті %.1f Вт·год, %.1f хв\n", wind, pe.wh, pe.minutes, wh, r.sim.time() / 60);
            if (wind == 0) CHECK(std::fabs(pe.wh / wh - 1) < 0.10, "у штиль прогноз енергії точний до 10%%");
            else CHECK(pe.wh >= 0.95 * wh && pe.wh < 1.5 * wh, "у вітер прогноз — з запасом, не менший за справжню витрату");
        }
    }
    {   // самоналаштування: з помилкою інерції в регуляторі ×0.5 і ×2 приходить до тієї самої ефективної інерції
        double res[2][2]; bool ok = true;
        const double ks[2] = {0.5, 2.0};
        for (int c = 0; c < 2; ++c) {
            QpFlightRig r(a, tb, 5);
            WindConfig w; w.mean20ft = Vec3d{4, 1.5, 0}; w.turbulence = 1.5; r.setWind(w, 3);
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {0, 0, 30}; hv.hold_s = 200;
            r.setMission({tk, hv});
            const fc::Vec3 I0 = r.ctl.cfg.af.inertia;
            r.ctl.setMass(r.ctl.cfg.af.mass, I0 * float(ks[c]));
            r.run(30);
            r.ctl.startAutotune();
            r.run(90);
            res[c][0] = r.ctl.inertiaControl().x / I0.x; res[c][1] = r.ctl.inertiaControl().y / I0.y;
            ok = ok && !r.sim.crashed() && r.ctl.tune.state == fc::QpAutotune::Done;
            std::printf("    ×%.1f: кіл %d, R² %.2f/%.2f, останнє підсилення %.2f/%.2f, режектор %.1f/%.1f, %s\n", ks[c], r.ctl.tune.rounds, r.ctl.tune.fit[0], r.ctl.tune.fit[1], r.ctl.tune.gain[0], r.ctl.tune.gain[1], r.nav.struct_notch_hz[0], r.nav.struct_notch_hz[1], r.ctl.tune.state == fc::QpAutotune::Done ? "готово" : "не завершено");
        }
        std::printf("  самоналаштування: інерція в регуляторі ×0.5 → крен ×%.2f, тангаж ×%.2f; ×2.0 → ×%.2f, ×%.2f від номіналу\n", res[0][0], res[0][1], res[1][0], res[1][1]);
        CHECK(ok, "самоналаштування завершується без аварії");
        CHECK(std::fabs(res[0][0] / res[1][0] - 1) < 0.25 && std::fabs(res[0][1] / res[1][1] - 1) < 0.25, "з різних помилок самоналаштування приходить до однієї інерції");
    }
    {   // передбачення поломок: розбалансований гвинт, «втомлена» батарея, люфт приводу — і жодної хибної тривоги на справному
        const char* nm[4] = {"справний", "гвинт ×3 вібрацій", "опір батареї ×2", "люфт приводу ×6"};
        bool got[4] = {false, false, false, false}, any_false = false;
        for (int f = 0; f < 4; ++f) {
            QpFlightRig r(a, tb, 5);
            WindConfig w; w.mean20ft = Vec3d{4, 1.5, 0}; w.turbulence = 1.0; r.setWind(w, 3);
            S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S hv; hv.type = S::HoverAt; hv.p = {1500, 0, 50}; hv.hold_s = 30; S ld; ld.type = S::Land;
            r.setMission({tk, hv, ld});
            r.run(200);
            if (f == 1) r.sensors.motor_vib[2] = 3.0;
            if (f == 2) r.sim.faults.battery_r_scale = 2.0;
            if (f == 3) r.sim.tiltServo(1).spec().backlash *= 6;
            r.run(250, [](const QpFlightRig& g) { return g.ctl.phase() == fc::QpPhase::Landed; });
            const ai::AnomalyMonitor& an = r.anomaly;
            got[f] = f == 1 ? an.vib_bad : f == 2 ? an.batt_bad : f == 3 ? an.servo_bad == 1 : false;
            if (f == 0) any_false = an.vib_bad || an.batt_bad || an.servo_bad >= 0;
            std::printf("  %-18s вібрації ×%.2f, опір ×%.2f, приводи %d → %s\n", nm[f], an.vibRatio(), an.rRatio(), an.servo_bad + 1, f == 0 ? (any_false ? "ХИБНА ТРИВОГА" : "тихо") : (got[f] ? "виявлено" : "НЕ виявлено"));
        }
        CHECK(!any_false, "на справному апараті — без тривог");
        CHECK(got[1] && got[2] && got[3], "розбалансований гвинт, втомлена батарея й люфт приводу мають виявлятись");
    }
}
