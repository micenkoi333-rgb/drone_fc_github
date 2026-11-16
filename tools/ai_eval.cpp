// tools/ai_eval.cpp — порівняння «без ШІ / з ШІ» (модель сил ai/weights_force.h) на польотах, яких мережа не бачила (seed 900+).
// Сценарії: висіння в поривчастому вітрі; повна місія з переходами у вітрі. Однакові seed — однакові вітер і шум датчиків.
// Збирання: g++ -std=c++17 -O2 -o ai_eval tools/ai_eval.cpp ;  запуск: ai_eval [кількість seed]
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "../sim/console_utf8.h"
#include "../sim/qp_flight.h"

using namespace sim;

struct HoverRes { double pos_rms = 0, pos_max = 0, alt_rms = 0, alt_max = 0, ai_err = 0, ai_truth = 0; bool crashed = false; };
struct MissionRes { bool ok = false; double land_err = 0, alt_dev_hover = 0, wh = 0, t = 0, ai_err = 0, ai_truth = 0; };

static HoverRes hover(const AircraftFile& a, const AeroTables& tb, unsigned seed, int mode, double wind) {
    QpFlightRig rig(a, tb, seed);
    rig.noise = SensorNoise::realistic();   // повний бортовий ланцюжок датчиків
    WindConfig w; w.mean20ft = Vec3d{wind, wind * 0.4, 0}; rig.setWind(w, seed + 77);
    using S = fc::QpStep;
    S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; rig.setMission({tk});
    rig.ai_mode = mode;
    if (const char* fs = std::getenv("AI_FADE")) rig.ai_fade_speed = std::atof(fs);
    if (const char* pl = std::getenv("AI_PAYLOAD")) rig.sim.setPayload(std::atof(pl));
    if (std::getenv("AI_NOMASS")) rig.ctl.mass_adapt = false;
    if (std::getenv("AI_NOTRUST")) rig.ai_self_check = false;
    HoverRes r; double n = 0;
    rig.run(150, [&](const QpFlightRig& g) {
        if (g.sim.time() > 40) {   // устаталось висіння
            const Vec3d p = g.sim.state().pos;
            const double dh = std::hypot(p.x, p.y), dz = p.z - g.sim.derived().cg_z - 30;
            r.pos_rms += dh * dh; r.alt_rms += dz * dz; n += 1;
            r.pos_max = std::fmax(r.pos_max, dh); r.alt_max = std::fmax(r.alt_max, std::fabs(dz));
        }
        return false;
    });
    r.pos_rms = std::sqrt(r.pos_rms / std::fmax(n, 1)); r.alt_rms = std::sqrt(r.alt_rms / std::fmax(n, 1));
    r.crashed = rig.sim.crashed();
    r.ai_err = rig.aiRmsError(); r.ai_truth = rig.aiRmsTruth();
    return r;
}

static MissionRes mission(const AircraftFile& a, const AeroTables& tb, unsigned seed, int mode, double wind) {
    QpFlightRig rig(a, tb, seed);
    rig.noise = SensorNoise::realistic();   // повний бортовий ланцюжок датчиків
    WindConfig w; w.mean20ft = Vec3d{wind * 0.6, wind * 0.8, 0}; rig.setWind(w, seed + 55);
    using S = fc::QpStep;
    S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30};
    S hv; hv.type = S::HoverAt; hv.p = {600, 0, 40}; hv.hold_s = 15;
    S ld; ld.type = S::Land; ld.p = {0, 0, 0};
    rig.setMission({tk, hv, ld});
    rig.ai_mode = mode;
    if (const char* fs = std::getenv("AI_FADE")) rig.ai_fade_speed = std::atof(fs);
    if (const char* pl = std::getenv("AI_PAYLOAD")) rig.sim.setPayload(std::atof(pl));
    if (std::getenv("AI_NOMASS")) rig.ctl.mass_adapt = false;
    if (std::getenv("AI_NOTRUST")) rig.ai_self_check = false;
    MissionRes r;
    rig.run(700, [&](const QpFlightRig& g) {
        const fc::QpPhase ph = g.ctl.phase();
        const double z = g.sim.state().pos.z - g.sim.derived().cg_z;
        // висіння над точкою: ціль — 40 м
        if (ph == fc::QpPhase::VtolHold && g.ctl.phaseTime() > 3) r.alt_dev_hover = std::fmax(r.alt_dev_hover, std::fabs(z - 40));
        return ph == fc::QpPhase::Landed;
    });
    r.ok = rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed();
    r.land_err = std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y);
    r.wh = rig.sim.battery().usedWh(); r.t = rig.sim.time();
    r.ai_err = rig.aiRmsError(); r.ai_truth = rig.aiRmsTruth();
    return r;
}

int main(int argc, char** argv) {
    const int seeds = argc > 1 ? std::atoi(argv[1]) : 6;
    const std::string what = argc > 2 ? argv[2] : "all";   // all | hover | hover6 | hover9 | mission — частини можна гнати паралельно
    const AircraftFile a = AircraftFile::load(activeAircraftPath());
    const AeroTables tb = AeroTables::load("aircraft/aero_tables.json");
    {
        QpFlightRig probe(a, tb, 1);
        std::printf("модель сил: %s\n", probe.advisor.net.loaded() ? "завантажено ai/weights_force.h" : "НЕМАЄ ваг — порівнювати нічого");
        if (!probe.advisor.net.loaded()) return 1;
    }
    if (what != "mission") for (double wind : {6.0, 9.0}) {
        if ((what == "hover9" && wind < 8) || (what == "hover6" && wind > 8)) continue;
        double s0[4] = {}, s2[4] = {}, err = 0, tru = 0; int cr0 = 0, cr2 = 0;
        for (int k = 0; k < seeds; ++k) {
            const unsigned seed = 900u + unsigned(k);
            const HoverRes h0 = hover(a, tb, seed, 1, wind), h2 = hover(a, tb, seed, 2, wind);   // 1 — тінь: як без ШІ, але рахує точність
            s0[0] += h0.pos_rms; s0[1] += h0.pos_max; s0[2] += h0.alt_rms; s0[3] += h0.alt_max;
            s2[0] += h2.pos_rms; s2[1] += h2.pos_max; s2[2] += h2.alt_rms; s2[3] += h2.alt_max;
            err += h0.ai_err; tru += h0.ai_truth; cr0 += h0.crashed; cr2 += h2.crashed;
            std::printf("  висіння, вітер %.0f, seed %u: без ШІ %.2f / %.2f м, з ШІ %.2f / %.2f м (гориз. / верт. RMS)\n", wind, seed, h0.pos_rms, h0.alt_rms, h2.pos_rms, h2.alt_rms);
            std::fflush(stdout);
        }
        const double n = seeds;
        std::printf("ВИСІННЯ 110 с, вітер %.0f м/с з поривами, %d польотів:\n", wind, seeds);
        std::printf("  гориз. відхилення RMS / макс.: без ШІ %.2f / %.2f м, з ШІ %.2f / %.2f м (%+.0f%% RMS)\n", s0[0] / n, s0[1] / n, s2[0] / n, s2[1] / n, 100 * (s2[0] / s0[0] - 1));
        std::printf("  висота RMS / макс.:          без ШІ %.2f / %.2f м, з ШІ %.2f / %.2f м (%+.0f%% RMS)\n", s0[2] / n, s0[3] / n, s2[2] / n, s2[3] / n, 100 * (s2[2] / s0[2] - 1));
        std::printf("  точність мережі (int8, на борту): похибка RMS %.2f Н при силі RMS %.2f Н; аварій: без ШІ %d, з ШІ %d\n", err / n, tru / n, cr0, cr2);
        std::fflush(stdout);
    }
    if (what == "all" || what == "mission") for (double wind : {4.0, 8.0}) {
        double v0 = 0, v2 = 0, l0 = 0, l2 = 0, w0 = 0, w2 = 0, err = 0, tru = 0; int ok0 = 0, ok2 = 0;
        for (int k = 0; k < seeds; ++k) {
            const unsigned seed = 950u + unsigned(k);
            const MissionRes m0 = mission(a, tb, seed, 1, wind), m2 = mission(a, tb, seed, 2, wind);
            ok0 += m0.ok; ok2 += m2.ok;
            v0 += m0.alt_dev_hover; v2 += m2.alt_dev_hover;
            l0 += m0.land_err; l2 += m2.land_err; w0 += m0.wh; w2 += m2.wh; err += m0.ai_err; tru += m0.ai_truth;
            std::printf("  місія, вітер %.0f, seed %u: висота у висінні над точкою ±%.1f / ±%.1f м, посадка за %.2f / %.2f м (без / з ШІ)\n", wind, seed, m0.alt_dev_hover, m2.alt_dev_hover, m0.land_err, m2.land_err);
            std::fflush(stdout);
        }
        const double n = seeds;
        std::printf("МІСІЯ 600 м з переходами, вітер %.0f м/с, %d польотів: успішно без ШІ %d, з ШІ %d\n", wind, seeds, ok0, ok2);
        std::printf("  макс. відхилення висоти у висінні над точкою: без ШІ %.2f м, з ШІ %.2f м\n", v0 / n, v2 / n);
        std::printf("  похибка посадки %.2f → %.2f м; енергія %.1f → %.1f Вт·год; точність мережі: %.2f Н при силі %.2f Н\n", l0 / n, l2 / n, w0 / n, w2 / n, err / n, tru / n);
        std::fflush(stdout);
    }
    return 0;
}
