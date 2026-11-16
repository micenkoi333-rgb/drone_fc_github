// main.cpp — консольний симулятор.
//   ./drone_sim                       — прогнати всі вбудовані сценарії
//   ./drone_sim hover --csv log.csv   — один сценарій + лог
//   ./drone_sim custom --wind 6,2,0 --gust 2 --mass 3 --payload 1.5 --motor 1:0.8 --time 20 --alt 3 --analog --csv out.csv
//   ./drone_sim custom --gps --wind 8,0,0 --fairing 0.05 --wing 0.5,1.6,8 --fin 0.04 --pusher 25 --fly 12
#include <cstdlib>
#include <cstring>
#include "sim/aircraft_params.h"
#include "sim/qp_analysis.h"
#include "sim/console_utf8.h"
#include "sim/scenarios.h"

using namespace sim;

// drone_sim qp trim | modes | mission — аналіз моделі конвертоплана (aircraft/tiltrotor.ini, aircraft/aero_tables.json)
static void printEig(const char* title, const std::vector<std::complex<double>>& ev) {
    std::printf("  %s:\n", title);
    std::vector<std::complex<double>> e = ev;
    std::sort(e.begin(), e.end(), [](auto a, auto b) { return std::abs(a) > std::abs(b); });
    for (size_t i = 0; i < e.size(); ++i) {
        const auto z = e[i];
        if (std::fabs(z.imag()) < 1e-6) {
            std::printf("    λ = %+9.4f   (дійсний, стала часу %.2f с, %s)\n", z.real(), 1.0 / std::fabs(z.real()), z.real() < 0 ? "стійкий" : "РОЗБІЖНИЙ");
        } else if (z.imag() > 0) {
            const double wn = std::abs(z), zeta = -z.real() / wn;
            std::printf("    λ = %+8.4f ± %7.4fj  (ωn %.3f рад/с, ζ %.3f, період %.2f с, %s)\n", z.real(), z.imag(), wn, zeta, 2 * 3.14159265358979 / z.imag(),
                        z.real() < 0 ? "стійкий" : "РОЗБІЖНИЙ");
        }
    }
}

static int qpCommand(int argc, char** argv) {
    try {
        const std::string sub = argc > 2 ? argv[2] : "trim";
        const AircraftFile file = AircraftFile::load(activeAircraftPath());
        const AeroTables tables = AeroTables::load("aircraft/aero_tables.json");
        QuadPlaneSim s(file, tables);
        const double vbus = s.battery().nominalBusV();
        if (sub == "trim") {
            std::printf("Рівний політ (гондоли на 90°, тягнуть %s), m = %.2f кг, ISA 0 м, U_шини %.1f В\n", s.cruiseRear() ? "усі 4 гвинти" : "передні гвинти", s.mass(), vbus);
            std::printf("  V м/с | α° (до хорди) | кермо висоти° | оберти об/хв | тяга Н | duty | вал Вт | від батареї Вт | L/D\n");
            for (double V : {9.0, 10.0, 12.0, 13.0, 14.0, 14.6, 15.0, 16.1, 18.0, 20.0, 22.0}) {
                const TrimResult t = trimCruise(s, V, vbus);
                std::printf("  %5.1f | %6.2f%s | %8.2f | %9.0f | %6.2f | %4.2f | %6.0f | %8.0f | %5.1f%s\n", V, t.alpha * 57.29578, "", t.elevator * 57.29578,
                            t.cruise_rps * 60, t.cruise_thrust, t.duty, t.shaft_w, t.elec_w, t.ld, t.ok ? "" : "   (немає тримування)");
            }
            std::printf("Перехід (спільний нахил гондол, крило на α = 4° до хорди; кермо висоти 0):\n");
            std::printf("  V м/с | нахил гондол° | оберти перед/зад, об/хв | тяга гвинтів Н | від батареї Вт\n");
            const TrimResult th = hoverPower(s);
            std::printf("  %5.1f | %10.1f | %6.0f / %6.0f | %9.2f | %8.0f   (висіння, крило не працює)\n", 0.0, 0.0, th.vtol_rps_front * 60, th.vtol_rps_rear * 60, th.vtol_thrust, th.elec_w);
            for (double V : {3.0, 5.0, 7.0, 8.0, 9.0}) {
                QuadPlaneSim tr(file, tables);
                const TrimResult t = trimTransition(tr, V, 4.0);
                std::printf("  %5.1f | %10.1f | %6.0f / %6.0f | %9.2f | %8.0f%s\n", V, t.tilt * 57.29578, t.vtol_rps_front * 60, t.vtol_rps_rear * 60,
                            t.vtol_thrust, t.elec_w, t.ok ? "" : "   (вага вже на крилі — див. крейсер)");
            }            return 0;
        }
        if (sub == "modes") {
            const double V = argc > 3 ? std::atof(argv[3]) : 17.0;
            const TrimResult t = trimCruise(s, V);
            if (!t.ok) { std::printf("Немає тримування на %.1f м/с\n", V); return 2; }
            const LinearModes lm = linearizeCruise(s, t);
            std::printf("Моди рівного польоту %.1f м/с (α %.2f°, кермо %.2f°):\n", V, t.alpha * 57.29578, t.elevator * 57.29578);
            printEig("поздовжні (u, w, q, θ): короткий період і фугоїда", lm.long_eig);
            printEig("бокові (v, p, r, φ): голландський крок, крен, спіраль", lm.lat_eig);
            return 0;
        }
        if (sub == "mission") {
            const MissionEstimate m = estimateMission(file, tables);
            std::printf("Місія: зліт → 5 км → висіння 60 с → 5 км назад → посадка\n");
            for (auto& [name, wh] : m.legs_wh) std::printf("  %-62s %6.2f Вт·год\n", name.c_str(), wh);
            std::printf("  Разом %.1f Вт·год за %.1f хв (корисна ємність %.0f Вт·год → запас %.0f%%)\n", m.total_wh, m.total_min, m.usable_wh, 100 * (1 - m.total_wh / m.usable_wh));
            std::printf("  Потужність: висіння %.0f Вт, крейсер 15 м/с %.0f Вт, перехід 8 м/с (гондоли %.0f°) %.0f Вт\n", m.hover_w, m.cruise_w, m.transition_tilt_deg, m.transition_w);
            std::printf("  Тільки крейсер: %.0f хв, %.0f км\n", m.endurance_min, m.range_km);
            return m.ok ? 0 : 2;
        }
        std::printf("drone_sim qp trim | modes [V] | mission\n");
        return 1;
    } catch (const std::exception& e) {
        std::printf("Помилка: %s\n", e.what());
        return 1;
    }
}

static Vec3 parseVec(const char* s) {
    Vec3 v;
    std::sscanf(s, "%f,%f,%f", &v.x, &v.y, &v.z);
    return v;
}

int main(int argc, char** argv) {
    enableUtf8Console();
    if (argc >= 2 && std::strcmp(argv[1], "aircraft") == 0) {   // звіт по aircraft/tiltrotor.ini (або іншому файлу)
        try {
            const AircraftFile a = AircraftFile::load(argc > 2 ? argv[2] : activeAircraftPath());
            const AircraftDerived d = deriveAircraft(a);
            printAircraftReport(a, d);
            return d.problems.empty() ? 0 : 3;
        } catch (const std::exception& e) {
            std::printf("Помилка файлу апарата: %s\n", e.what());
            return 1;
        }
    }
    if (argc >= 2 && std::strcmp(argv[1], "qp") == 0) return qpCommand(argc, argv);
    auto lib = builtinScenarios();
    if (argc < 2) {
        std::printf("Всі сценарії:\n");
        for (auto& [name, cfg] : lib) printMetrics(name.c_str(), runScenario(cfg));
        std::printf("\nПараметри: drone_sim <сценарій|custom> [--wind x,y,z] [--gust sigma] [--com x,y,z] [--mass kg] [--payload kg]\n"
                    "  [--motor idx:eff] [--gyro-bias x,y,z] [--time s] [--alt m] [--analog] [--seed n] [--csv file]\n"
                    "  Планер: [--gps] [--fairing cd] [--wing площа,розмах,кут] [--wing-mass кг/м2] [--fin площа] [--pusher тяга_Н]\n"
                    "          [--pusher-hold] [--no-weathervane] [--fly м/с — летіти вперед з 10-ї секунди]\n");
        return 0;
    }
    ScenarioConfig c;
    std::string name = argv[1];
    if (name != "custom") {
        if (!lib.count(name)) { std::printf("Невідомий сценарій: %s\n", name.c_str()); return 1; }
        c = lib[name];
    }
    const char* csv = nullptr;
    float payload = 0, fly = 0;
    fc::AirframeConfig air;
    bool use_air = false;
    for (int i = 2; i < argc; ++i) {
        auto arg = [&](const char* k) { return std::strcmp(argv[i], k) == 0 && i + 1 < argc; };
        if (arg("--wind")) c.wind.mean = parseVec(argv[++i]);
        else if (arg("--gust")) c.wind.gust_sigma = std::atof(argv[++i]);
        else if (arg("--com")) c.quad.com_offset = parseVec(argv[++i]);
        else if (arg("--mass")) {  // власна вага дрона: змінюємо і реальну, і ту, що в налаштуваннях контролера
            float m = std::atof(argv[++i]);
            c.quad.inertia = c.quad.inertia * (m / c.quad.mass);
            c.quad.mass = m;
            c.fc.vehicle.mass = m;
            c.fc.vehicle.inertia = c.quad.inertia;
        }
        else if (arg("--payload")) payload = std::atof(argv[++i]);  // вантаж — контролер про нього не знає
        else if (arg("--gyro-bias")) c.imu.gyro_bias_dps = parseVec(argv[++i]);
        else if (arg("--time")) c.duration = std::atof(argv[++i]);
        else if (arg("--alt")) c.target_alt = std::atof(argv[++i]);
        else if (arg("--seed")) c.seed = std::atoi(argv[++i]);
        else if (arg("--csv")) csv = argv[++i];
        else if (arg("--motor")) { int idx; float e; std::sscanf(argv[++i], "%d:%f", &idx, &e); if (idx >= 0 && idx < 4) c.quad.motor_eff[idx] = e; }
        else if (std::strcmp(argv[i], "--analog") == 0) c.analog_imu = true;
        else if (std::strcmp(argv[i], "--gps") == 0) c.mode = fc::Mode::PosHold;
        else if (arg("--fairing")) { air.body_cd = std::atof(argv[++i]); use_air = true; }
        else if (arg("--wing")) {
            Vec3 v = parseVec(argv[++i]);
            air.wing.enabled = true; air.wing.area = v.x; air.wing.span = v.y; air.wing.incidence_deg = v.z; use_air = true;
        }
        else if (arg("--wing-mass")) air.wing.mass_per_m2 = std::atof(argv[++i]);
        else if (arg("--fin")) { air.fin.area = std::atof(argv[++i]); use_air = true; }
        else if (arg("--pusher")) { air.pusher.enabled = true; air.pusher.max_thrust = std::atof(argv[++i]); use_air = true; }
        else if (std::strcmp(argv[i], "--pusher-hold") == 0) c.fc.pusher_in_hold = true;
        else if (std::strcmp(argv[i], "--no-weathervane") == 0) c.fc.weathervane = false;
        else if (arg("--fly")) { fly = std::atof(argv[++i]); c.mode = fc::Mode::PosHold; }
        else { std::printf("Невідомий параметр: %s\n", argv[i]); return 1; }
    }
    if (use_air) applyAirframe(c.quad, air, &c.fc.vehicle);
    if (payload > 0) attachPayload(c.quad, payload, {0, 0.06f, -0.15f});
    if (fly > 0) {
        float alt = c.target_alt;
        c.setpoint = [fly, alt](float t, fc::Setpoint& sp) { sp.altitude = alt; sp.speed_limit = 18; sp.vel_fwd = t > 10 ? fly : 0; };
        c.metrics_from = 1e9f;
    }
    // середня потужність після 15-ї секунди — щоб порівнювати планери між собою
    double wsum = 0, lsum = 0;
    long wn = 0;
    auto prev = c.on_step;
    c.on_step = [&](float t, World& w) { if (prev) prev(t, w); if (t > 15) { wsum += w.quad.watts; lsum += w.quad.aero.lift; ++wn; } };
    std::vector<LogRow> log;
    Metrics m = runScenario(c, csv ? &log : nullptr);
    printMetrics(name.c_str(), m);
    if (wn) std::printf("Середня потужність: %.0f Вт, підйомна сила крила: %.1f Н, злітна вага %.2f кг\n", wsum / wn, lsum / wn, c.quad.mass);
    if (csv) std::printf(writeCsv(csv, log) ? "Лог: %s\n" : "Не вдалось записати %s\n", csv);
    return (m.crashed || m.crash_detected) ? 2 : 0;
}
