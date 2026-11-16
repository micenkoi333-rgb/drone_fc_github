// tools/ai_datagen.cpp — генератор навчальних даних для моделі сил (ai/force_model.h): багато польотів симулятора з розкидом
// вітру, поривів, місій і шуму датчиків; запис 50 Гц у двійковий файл float32.
// Рядок: 17 ознак | 3 мітки (зовнішня сила в тілі, частки ваги) | вага коптера w | фаза | номер польоту
//        | 3 виміряна сила (бортова: m·акселерометр − тяга за обертами ESC; частки ваги) | оцінка маси / маса без вантажу = 27 чисел.
// Мітка (3 після ознак) — теж бортовий вимір, а не правда симулятора: мережа вчиться саме того, що апарат бачитиме в польоті.
// Сили й виміряна сила — у частках ваги БЕЗ вантажу. Вантаж (0…1.2 кг, автопілот про нього не знає) — у 60% польотів.
// Збудження гондол (частка польотів, 4-й аргумент, типово 0.5): до команди кожної гондоли додається випадкове повільне відхилення ±5°,
// щоб мережа бачила вплив гондол окремо від вітру (інакше нахил гондол завжди «слідує» за вітром і мережа плутає причину й наслідок).
// Збирання: g++ -std=c++17 -O2 -o ai_datagen tools/ai_datagen.cpp
// Запуск:   ai_datagen <польотів> <перший seed> <файл.bin> [частка польотів зі збудженням гондол]   (кілька процесів з різними seed — паралельно на ядрах)
#include <cstdio>
#include <cstdlib>
#include <random>
#include "../sim/console_utf8.h"
#include "../sim/qp_flight.h"
#include "../sim/portable_rng.h"

using namespace sim;

static AircraftFile loadAircraft() {
    try { return AircraftFile::load(activeAircraftPath()); } catch (...) { return AircraftFile::load("../aircraft/tiltrotor.ini"); }
}
static AeroTables loadTables() {
    try { return AeroTables::load("aircraft/aero_tables.json"); } catch (...) { return AeroTables::load("../aircraft/aero_tables.json"); }
}

int main(int argc, char** argv) {
    if (argc < 4) { std::printf("ai_datagen <польотів> <перший seed> <файл.bin>\n"); return 1; }
    const int flights = std::atoi(argv[1]);
    const unsigned seed0 = unsigned(std::atoi(argv[2]));
    const double excite_frac = argc > 4 ? std::atof(argv[4]) : 0.5;
    FILE* out = std::fopen(argv[3], "wb");
    if (!out) { std::printf("не вдалось відкрити %s\n", argv[3]); return 1; }
    const AircraftFile a = loadAircraft();
    const AeroTables tb = loadTables();
    long rows = 0;
    for (int k = 0; k < flights; ++k) {
        const unsigned seed = seed0 + unsigned(k);
        std::mt19937 rng(seed * 2654435761u);
        prng::uniform<double> U(0, 1);
        QpFlightRig rig(a, tb, seed);
        // повний реальний ланцюжок датчиків (EKF на сирих IMU/GPS/баро, Піто, ESC, енкодери) + розкид, якого автопілот не знає:
        // вібрації, тяга кожного мотора ±3% (знос, балансування гвинта), зсув нуля Піто
        rig.noise = SensorNoise::realistic();
        rig.sensors.vib_scale = 0.5 + 1.5 * U(rng);
        for (int i = 0; i < 4; ++i) rig.sim.faults.vtol_scale[i] = 0.97 + 0.06 * U(rng);
        rig.noise.pitot_offset_pa = 1.2 * U(rng);
        const double wind = 10.0 * U(rng) * U(rng) + 2.0 * U(rng), dir = 6.2832 * U(rng);   // частіше слабкий, інколи до 12 м/с
        WindConfig w; w.mean20ft = Vec3d{wind * std::cos(dir), wind * std::sin(dir), 0};
        rig.setWind(w, seed + 1000);
        using S = fc::QpStep;
        std::vector<S> m;
        S tk; tk.type = S::Takeoff; tk.p = {0, 0, float(20 + 30 * U(rng))}; m.push_back(tk);
        const bool hover_only = U(rng) < 0.4;   // 40% — висіння й переміщення коптером у вітрі, 60% — повна місія з переходами
        if (hover_only) {
            for (int j = 0; j < 4; ++j) { S h; h.type = S::HoverAt; h.p = {float(60 * (U(rng) - 0.5)), float(60 * (U(rng) - 0.5)), float(15 + 35 * U(rng))}; h.hold_s = float(5 + 15 * U(rng)); m.push_back(h); }
        } else {
            const float leg = float(300 + 600 * U(rng)), ang = float(6.2832 * U(rng));
            S h; h.type = S::HoverAt; h.p = {leg * std::cos(ang), leg * std::sin(ang), float(30 + 30 * U(rng))}; h.hold_s = float(5 + 15 * U(rng)); m.push_back(h);
        }
        S ld; ld.type = S::Land; ld.p = {0, 0, 0}; m.push_back(ld);
        rig.setMission(m);
        // вантаж, про який автопілот не знає (оцінює масу сам у висінні): 40% польотів — без вантажу, решта — 0.1…1.2 кг
        const double payload = U(rng) < 0.4 ? 0.0 : 0.1 + 1.1 * U(rng);
        rig.sim.setPayload(payload);
        const float fid = float(seed);
        const bool excite = U(rng) < excite_frac;
        const double ex_sd = 0.087 * (0.5 + U(rng));
        double ex[4] = {0, 0, 0, 0};
        prng::normal<double> N(0, 1);
        if (excite)
            rig.override_act = [&](const fc::QpEstimate& e, fc::QpActuators& act) {
                act = rig.ctl.update(0.002f, e);
                const bool on = rig.ctl.vtolWeight() > 0.05f && rig.sim.state().pos.z - rig.sim.derived().cg_z > 4.0;
                for (int i = 0; i < 4; ++i) {
                    ex[i] += -ex[i] * 0.002 / 0.8 + ex_sd * std::sqrt(2 * 0.002 / 0.8) * N(rng);
                    if (on) act.tilt[i] = std::clamp(act.tilt[i] + float(ex[i]), rig.ctl.cfg.tilt_min, rig.ctl.cfg.tilt_max);
                }
                return true;
            };
        rig.ai_record = [&](const float* f, const float* y, const QpFlightRig& r) {
            float row[27];
            for (int i = 0; i < ai::kForceFeatures; ++i) row[i] = f[i];
            for (int i = 0; i < 3; ++i) row[17 + i] = y[i];
            row[20] = r.ctl.vtolWeight(); row[21] = float(int(r.ctl.phase())); row[22] = fid;
            const double mg = r.ctl.massNominal() * QuadPlaneSim::kG;
            for (int i = 0; i < 3; ++i) row[23 + i] = float(r.ai_meas[i] / mg);
            row[26] = r.ctl.massEstimate() / std::fmax(r.ctl.massNominal(), 0.1f);
            std::fwrite(row, sizeof(float), 27, out);
            ++rows;
        };
        rig.run(700, [](const QpFlightRig& r) { return r.ctl.phase() == fc::QpPhase::Landed; });
        std::printf("політ %u: вітер %.1f м/с, вантаж %.2f кг (оцінка %+.2f), %s%s, %s, записів усього %ld\n", seed, wind, payload, rig.ctl.massEstimate() - rig.ctl.massNominal(), hover_only ? "висіння" : "місія", excite ? " зі збудженням гондол" : "",
                    rig.sim.crashed() ? "АВАРІЯ" : (rig.ctl.phase() == fc::QpPhase::Landed ? "сів" : "не сів"), rows);
        std::fflush(stdout);
    }
    std::fclose(out);
    return 0;
}
