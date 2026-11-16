#pragma once

static void testPhysicsDetail() {
    SECTION("Фізика: гвинт у косому потоці (елемент лопаті), момент Мунка, поправка на Re, люфт приводів, нагрів мотора");
    const AircraftFile a = AircraftFile::load("aircraft/tiltrotor.ini");
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    const BemRotor bem(a, "vtol_prop");
    const Propeller prop(a, "vtol_prop");
    const double n = 100, D = prop.diameter(), rho = 1.225;
    auto converge = [&](double v_ax, double v_ip, int spin) { BemRotor::Out o; double vi = 0; for (int k = 0; k < 12; ++k) { o = bem.eval(n, v_ax, v_ip, rho, spin, vi, 4); vi = o.vi; } return o; };
    double worst = 0;
    for (double J : {0.0, 0.2, 0.4}) {
        const BemRotor::Out o = converge(J * n * D, 0, 1);
        const double ct = o.thrust / (rho * n * n * D * D * D * D);
        worst = std::fmax(worst, std::fabs(ct / prop.ct(J) - 1));
    }
    std::printf("  елемент лопаті проти таблиць UIUC в осьовому потоці (J 0…0.4): найбільша розбіжність CT %.0f%%\n", 100 * worst);
    CHECK(worst < 0.2, "геометрія лопаті має відтворювати виміряні CT в осьовому потоці (±20%%): %.0f%%", 100 * worst);
    const BemRotor::Out h0 = converge(0, 0, 1), h9 = converge(0, 9, 1), h9m = converge(0, 9, -1);
    std::printf("  висіння → 9 м/с збоку: тяга %.2f → %.2f Н, сила H %.2f Н (проти руху), моменти асиметрії %.3f / %.3f Н·м\n", h0.thrust, h9.thrust, h9.fx, h9.mx, h9.my);
    CHECK(h9.thrust > h0.thrust && h9.fx < 0 && h9.my < 0, "косий потік: тяга росте, H проти руху, передня частина диска тягне більше");
    CHECK(std::fabs(h9.mx + h9m.mx) < 1e-9 && std::fabs(h9.my - h9m.my) < 1e-9, "протилежне обертання дзеркалить бічний момент");
    {
        QuadPlaneAero aero(a, tb, deriveAircraft(a));
        AeroInputs in; in.v_body = Vec3d{15 * std::cos(0.1), 0, -15 * std::sin(0.1)}; in.rho = 1.225;
        const double m_with = aero.eval(in).moment.y;
        AircraftFile b = a; b.values["drag.munk_volume_m3"] = {"0"};
        QuadPlaneAero aero2(b, tb, deriveAircraft(b));
        const double m_without = aero2.eval(in).moment.y;
        std::printf("  момент Мунка корпусу й гондол на α ≈ 6°, 15 м/с: %.3f Н·м (ніс угору)\n", m_without - m_with);
        CHECK(m_with < m_without, "момент Мунка має бути дестабілізуючим (ніс угору при α > 0)");
        AeroInputs slow = in; slow.v_body = in.v_body * (8.0 / 15.0);
        auto cdOf = [&](const QuadPlaneAero& ae, const AeroInputs& x) { const AeroResult r = ae.eval(x); const double V = x.v_body.norm(); return -(r.wing_force.dot(x.v_body / V)) / (0.5 * 1.225 * V * V); };
        AircraftFile c = a; c.values.erase("aero_model.wing_table_re"); c.values.erase("aero_model.tail_table_re");
        QuadPlaneAero aero3(c, tb, deriveAircraft(c));
        const double k = cdOf(aero, slow) / cdOf(aero3, slow);
        std::printf("  опір крила на 8 м/с (Re ≈ 1.5·10⁵) з поправкою на Re: ×%.2f\n", k);
        CHECK(k > 1.0 && k < 1.3, "на меншому Re профільний опір більший");
    }
    {
        QuadPlaneSim sim(a, tb, 3);
        QpControls u; u.tilt[0] = u.tilt[1] = u.tilt[2] = u.tilt[3] = 0;
        double t_before = 0, t_after = 0;
        for (int k = 0; k < 900; ++k) { u.tilt[0] = k < 600 ? 0.5 : 0.497; sim.step(0.001, u); if (k == 599) t_before = sim.tilt(0); t_after = sim.tilt(0); }
        std::printf("  люфт приводу нахилу (0.3°): команда 0.500 → 0.497 рад, гондола %.4f → %.4f рад\n", t_before, t_after);
        CHECK(std::fabs(t_after - t_before) < 1e-4 && t_before > 0.49, "розворот команди в межах люфту не рухає гондолу");
    }
    {
        MotorEsc m(a, "vtol_motor", 2e-5);
        double t0 = m.temperature();
        for (int k = 0; k < 60000; ++k) m.step(0.001, 0.6, 24, 0.12);
        std::printf("  нагрів мотора 60 с під навантаженням: %.0f → %.0f °C\n", t0, m.temperature());
        CHECK(m.temperature() > t0 + 3 && m.temperature() < 150, "мотор під навантаженням гріється");
    }
}
