// qp_flight.h — зв'язка контролера (fc/qp_mission.h) і моделі (qp_sim.h): ідентифікація таблиць тяги/тримування, датчики з шумом,
// цикл 1 кГц фізики / 500 Гц контролера, журнал, звіт про політ.
#pragma once
#include <cstdio>
#include <functional>
#include <deque>
#include <random>
#include <string>
#include <vector>
#include "../fc/qp_mission.h"
#include "../ai/force_model.h"
#include "../ai/force_advisor.h"
#include "../ai/flight_policy.h"
#include "../ai/health_monitor.h"
#include "../ai/energy_manager.h"
#include "../ai/anomaly_monitor.h"
#include "../ai/link.h"
#include "qp_analysis.h"
#include "qp_sensors.h"
#include "portable_rng.h"

namespace sim {

struct SensorNoise {          // 0 — ідеальні датчики
    double gyro_rad_s = 0, airspeed_mps = 0, pos_m = 0, vel_mps = 0, att_rad = 0;
    double airspeed_bias = 0;
    double delay_att_s = 0;     // затримка гіроскопа/кутів/барометра, с (типово 0.02)
    double delay_gps_s = 0;     // затримка GPS-позиції/швидкості, с (типово 0.15)
    double gps_hz = 0;          // частота оновлення GPS, Гц (0 — на кожен цикл; типово 10)
    double rpm_frac = 0;        // відносний шум телеметрії обертів ESC (типово 0.01)
    bool use_ekf = false;       // true — повний ланцюжок: сирі датчики (qp_sensors.h) → EKF (fc/qp_ekf.h) → контролер; поля вище (крім Піто й ESC) не діють
    // трубка Піто (диференційний датчик MS4525DO / SDP33): міряє напір уздовж осі X тіла
    bool pitot_model = false;
    double pitot_noise_pa = 1.2;      // шум тиску, Па (на 2 м/с напір лише 2.5 Па — нижче ≈3 м/с показ ненадійний)
    double pitot_offset_pa = 0.6;     // залишок нуля після калібрування на землі, Па (знак випадковий)
    double pitot_drift_pa_s = 0.015;  // дрейф нуля (температура), Па/√с
    double pitot_tau_s = 0.03;        // трубка + шланг
    double pitot_hz = 100;
    double pitot_wash_k = 0.04;       // частка напору від підсмоктування передніх гвинтів
    // телеметрія ESC (двонапрямний DShot): затримка, дискретність, пропуски кадрів
    bool esc_model = false;
    double esc_delay_s = 0.002, esc_quant_rps = 0.5, esc_drop = 0.002;
    // датчик кута гондоли (магнітний енкодер AS5600, 12 біт)
    double tilt_noise_rad = 0.005, tilt_quant_rad = 0, tilt_tau_s = 0;
    // затримка від обчислення команди до виходу (цикл FC + DShot/ШІМ), с
    double delay_act_s = 0;

    static SensorNoise legacy() { return SensorNoise{}; }   // «правда + шум» без EKF (так навчено ШІ першої версії)
    static SensorNoise ideal() { SensorNoise n; n.tilt_noise_rad = 0; return n; }
    // реалістичний набір: сирі датчики → EKF, реальна трубка Піто, телеметрія ESC, енкодери гондол, затримка виходу
    static SensorNoise realistic() {
        SensorNoise n;
        n.use_ekf = true;
        n.gyro_rad_s = 0.01; n.pos_m = 0.5; n.vel_mps = 0.1; n.att_rad = 0.004;   // діють, поки EKF сходиться
        n.delay_att_s = 0.02; n.delay_gps_s = 0.15; n.gps_hz = 10;
        n.rpm_frac = 0.005;
        n.pitot_model = true; n.esc_model = true;
        n.tilt_noise_rad = 0.0015; n.tilt_quant_rad = 2 * 3.14159265358979323846 / 4096; n.tilt_tau_s = 0.003;
        n.delay_act_s = 0.003;
        return n;
    }
};

struct SensorFaults { bool gps_lost = false, pitot_dead = false, rc_lost = false; };   // відмови датчиків для випробувань

struct FlightRow {
    float t, x, y, z, vx, vy, vz, roll, pitch_down, yaw, airspeed, volts, amps, wv;
    int phase, step;
    // докладний журнал (після перших 16 полів — старі CSV і повтор польоту читаються як і раніше)
    float wx = 0, wy = 0, wz = 0;                    // вітер біля апарата, м/с (світ ENU)
    float power = 0, soc = 0, batt_temp = 0;         // Вт, 0…1, °C
    float rps[4] = {0, 0, 0, 0};                     // оберти моторів, об/с
    float duty[4] = {0, 0, 0, 0};                    // газ (ШІМ) 0…1
    float tilt[4] = {0, 0, 0, 0};                    // нахил гондол, ° (0 — вгору, 90 — уперед)
    float aileron = 0, elevator = 0, rudder = 0;     // керма, −1…1
    float cur_lim = 1;                               // обмежувач струму: 1 — без обмеження
    float omx = 0, omy = 0, omz = 0;                 // кутові швидкості тіла, рад/с
};

// подія у звіті: стан, що вмикався (можливо, кілька разів поспіль) — один запис з першою й останньою появою
struct FlightEvent {
    std::string what;
    double t0 = 0, t1 = 0, active_s = 0;
    int count = 1;
};

namespace detail {
// усталені оберти мотора з навантаженням гвинта: розв'язок M_мотора(ω) = M_гвинта(ω) бісекцією
inline double steadyRps(const MotorEsc& m, const Propeller& p, double duty, double v_bus, double v_axial, double i_max, double rho) {
    double lo = 0.5, hi = std::fmax(duty * v_bus / m.ke(), 1.0);
    auto f = [&](double w) {
        const double i_ph = std::clamp((duty * v_bus - m.ke() * w) / m.rm(), 0.0, i_max);
        const double qm = m.ke() * (i_ph - m.i0());
        const double n = w / (2 * 3.14159265358979323846);
        return qm - p.eval(n, v_axial, 0.0, rho).torque;
    };
    for (int i = 0; i < 60; ++i) { const double mid = 0.5 * (lo + hi); (f(mid) > 0 ? lo : hi) = mid; }
    return 0.5 * (lo + hi) / (2 * 3.14159265358979323846);
}
}  // namespace detail

// Підсумок польоту: накопичується під час польоту (100 Гц), показується й зберігається після посадки/аварії
struct FlightReport {
    double t = 0, dist_ground = 0, dist_air = 0, max_alt = 0, max_airspeed = 0, max_groundspeed = 0, max_climb = 0, max_sink = 0;
    double min_volt = 1e9, max_amps = 0, max_power = 0, max_tilt = 0, max_temp = 0, max_wind = 0, wind_sum = 0;
    double max_home_dist = 0, touch_vz = 0, last_vz = 0;
    double phase_t[10] = {};             // час у кожній фазі (індекс — fc::QpPhase)
    int trans_fwd = 0, trans_back = 0;
    long samples = 0;
    std::vector<std::string> events;     // аварійні стани, що з'являлися (з часом), — текстом (з ev)
    std::vector<FlightEvent> ev;         // ті самі події: повторення в межах 5 с зливаються в одну
    int prev_phase = -1;
    std::string prev_alert;
};

class QpFlightRig {
public:
    QuadPlaneSim sim;
    fc::QpFlight ctl;
    std::vector<FlightRow> log;
    SensorNoise noise = SensorNoise::realistic();   // типово — реальний ланцюжок датчиків; SensorNoise::ideal() — ідеальні
    SensorFaults sfaults;
    SensorSuite sensors;        // сирі датчики (для use_ekf): вібрації, відмови IMU — sensors.vib_scale, sensors.imu_fault[]
    fc::QpNavigator nav;        // EKF на два IMU
    FlightReport report;
    // ШІ-співпроцесор (ai/): модель сил радить контролеру поправку прискорення через сторож. 0 — вимкнено, 1 — тінь (рахує, не впливає),
    // 2 — радить. ai_record — запис ознак і правди для навчання (50 Гц, лише в повітрі).
    // Перехоплення керування (пілотажні трюки): повертає true — act заповнено, автопілот цього кроку не працює
    std::function<bool(const fc::QpEstimate&, fc::QpActuators&)> override_act;
    int ai_mode = 0;
    bool ai_ff_horizontal = false;
    ai::ForceAdvisor advisor;   // бортова частина: вимір сили, мережа, самоперевірка, сторож
    ai::FlightPolicy pilot;     // ШІ-пілот, навчений з нуля (tools/ai_rl.cpp): радить прискорення у висінні
    bool pilot_on = false;
    std::vector<float> trim_v, trim_w;   // модель: потужність від батареї в рівному польоті за швидкістю (для прогнозу до першого польоту)
    float hover_w = 0;                   // модель: потужність висіння від батареї
    ai::FlightPolicy2 pilot2;   // v2: ще й повітряна швидкість, оберти моторів і кут носа (ai/policy_hover2.txt)
    bool pilot2_on = false;
    ai::HealthMonitor health;   // бортова діагностика: гвинти, мотори, приводи, вітер, центр мас, енергія
    ai::AnomalyMonitor anomaly; // передбачення поломок: вібрації, опір батареї, приводи гондол — проти «здорового» профілю
    ai::EnergyManager energy;   // ШІ-енергоменеджер: швидкість і висота крейсеру за вітром і вивченою кривою потужності
    std::function<void(const float* features, const float* label, const QpFlightRig&)> ai_record;
    struct AiStats { double n = 0, err2 = 0, truth2 = 0; } ai_stats;
    // «Виміряна» аеродинамічна сила, як її отримає бортовий ШІ (Н, тіло): маса × акселерометр − тяга гвинтів за таблицею тяги.
    // Похибки: таблиця тяги (повільний дрейф до ±4% тяги кожного гвинта), шум акселерометра й вібрацій, затримка 20 мс.
    double ai_meas[3] = {0, 0, 0};
    bool ai_trace_on = false;              // GUI «Мозок ШІ»: зберігати повний знімок мережі
    ai::ForceModel::Trace ai_trace;
    bool ai_trace_valid = false;
    float ai_truth[3] = {0, 0, 0}, ai_used_acc[3] = {0, 0, 0}, ai_fade = 0;
    // самоперевірка ШІ: останнє передбачення, довіра 0…1, середній промах (Н), пара «передбачила / виміряно» для показу
    float ai_pred[3] = {0, 0, 0}, ai_trust = 0.5f, ai_check_rms = 0, ai_check_pred[3] = {0, 0, 0}, ai_check_meas[3] = {0, 0, 0};
    bool ai_self_check = true;
    double ai_fade_speed = 11.0;
private:
    bool ai_has_pred_ = false;
public:   // точність передбачення (у повітрі, гвинти несуть вагу)

    QpFlightRig(const AircraftFile& file, const AeroTables& tables, unsigned seed = 1) : sim(file, tables, seed), sensors(seed), file_(file), tables_(tables), rng_(seed * 7919u + 3u) {
        buildConfig();
        if (file.has("battery.usable_wh_target")) health.batt_wh = float(file.num("battery.usable_wh_target"));
        energy.v_max = ctl.cfg.v_max; energy.v_min = std::fmax(ctl.cfg.assist_speed + 2.0f, 1.3f * ctl.cfg.af.v_stall);
        double gear_z = 0;   // найнижча опора (ini-координати)
        if (file.has("ground.gear_points")) { const std::vector<double> gp = file.nums("ground.gear_points"); gear_z = 1e9; for (size_t i = 2; i < gp.size(); i += 3) gear_z = std::fmin(gear_z, gp[i]); }
        else gear_z = file.num("vtol_motor.z_m") - file.num("geometry.gear_height_m");
        sim.place(Vec3d{0, 0, sim.derived().cg_z - gear_z - 0.003}, Vec3d{}, 0, 0, 0);
    }

    void setMission(std::vector<fc::QpStep> m) { ctl.setMission(std::move(m)); }
    void setWind(const WindConfig& w, unsigned seed = 1) { sim.setWind(w, seed); }

    // Крок фізики 1 мс; контролер — кожні 2 мс; журнал — 10 Гц
    void step() {
        if (step_count_ % 2 == 0) {
            if (noise.use_ekf) {   // цикл оцінювача 500 Гц: усереднені відліки IMU за 2 мс, свіжі GPS/баро/компас
                nav.prm.mag_decl = float(sensors.magDeclination());   // схилення за положенням (як COMPASS_AUTODEC)
                double rps = 0; for (int i = 0; i < 4; ++i) rps += sim.vtolRps(i) / 4;
                const fc::EkfInputs in = sensors.take(!sfaults.gps_lost);
                nav.update(in, 0.002f, float(rps));
                imu_acc_ = in.imu[nav.primaryIndex()].accel - nav.primary().accelBias();
            }
            const fc::QpEstimate e = estimate();
            last_e_ = e;
            advisor.meas.update(0.002f, e, ctl.cfg, ctl.massNominal());   // маса без вантажу: вимір не має залежати від оцінки маси (інакше петля «оцінка ↔ сила»)
            ctl.setMeasuredForce(advisor.meas.force(), advisor.meas.valid());
            if ((ai_mode > 0 || ai_record) && step_count_ % 10 == 0) aiTick(e);   // ШІ: 100 Гц
            if (pilot_on && step_count_ % 10 == 0) {   // ШІ-пілот: 100 Гц, лише в утриманні точки на гвинтах
                const fc::QpPhase ph = ctl.phase();
                const bool hover = ph == fc::QpPhase::VtolHold && ctl.vtolWeight() > 0.99f;
                if (hover) ctl.setAiAccel(pilot.tick(e, ctl.holdPoint(), advisor.meas.force(), ctl.massNominal(), 0.02f));
                else { pilot.reset(); ctl.setAiAccel(fc::Vec3{0, 0, 0}); }
            }
            if (pilot2_on && step_count_ % 10 == 0) {
                const bool hover = ctl.phase() == fc::QpPhase::VtolHold && ctl.vtolWeight() > 0.99f;
                if (hover) { ctl.setAiAccel(pilot2.tick(e, ctl.holdPoint(), advisor.meas.force(), ctl.massNominal(), 0.02f)); ctl.setAiPitchUp(pilot2.pitch_up); }
                else { pilot2.reset(); ctl.setAiAccel(fc::Vec3{0, 0, 0}); ctl.setAiPitchUp(0); }
            }
            if (!(override_act && override_act(e, act_))) act_ = ctl.update(0.002f, e);   // трюк (fc/qp_tricks.h) керує сам, поки активний
            for (int i = 0; i < 4; ++i) { u_.duty_vtol[i] = act_.duty_vtol[i]; u_.tilt[i] = act_.tilt[i]; }
            u_.duty_cruise = act_.duty_cruise; u_.aileron = act_.aileron; u_.elevator = act_.elevator; u_.rudder = act_.rudder;
            health.update(0.002f, e, act_, ctl, advisor.meas.force());
            energy.update(0.002f, e, ctl);
            anomaly.update(0.002f, e, act_, ctl);
            ctl.energy_v_dem = energy.on ? energy.v_best : 0.0f; ctl.energy_alt = energy.on ? energy.alt_best : 0.0f;
        }
        if (noise.delay_act_s > 0) {   // команда доходить до моторів і машинок із затримкою
            act_q_.push_back(u_);
            const size_t back = size_t(noise.delay_act_s * 1000 + 0.5);
            while (act_q_.size() > back + 1) act_q_.pop_front();
            sim.step(0.001, act_q_.front());
        } else sim.step(0.001, u_);
        if (noise.use_ekf) sensors.sample(sim, 0.001);
        sampleAux(0.001);
        hist_.push_back(sim.state());
        if (hist_.size() > 600) hist_.pop_front();
        if (step_count_ % 100 == 0) {
            const QpState& s = sim.state();
            const Vec3d v = sim.velWorld(), eu = sim.eulerRad();
            log.push_back({float(sim.time()), float(s.pos.x), float(s.pos.y), float(s.pos.z), float(v.x), float(v.y), float(v.z), float(eu.x), float(eu.y), float(eu.z),
                           float(sim.airspeed()), float(sim.busVoltage()), float(sim.busCurrent()), ctl.vtolWeight(), int(ctl.phase()), int(ctl.stepIndex())});
            FlightRow& r = log.back();
            const Vec3d w = sim.windWorld();
            r.wx = float(w.x); r.wy = float(w.y); r.wz = float(w.z);
            r.power = float(sim.busPower()); r.soc = float(sim.battery().soc()); r.batt_temp = float(sim.battery().temperature());
            for (int i = 0; i < 4; ++i) { r.rps[i] = float(sim.vtolRps(i)); r.duty[i] = act_.duty_vtol[i]; }
            for (int i = 0; i < 4; ++i) r.tilt[i] = float(sim.tilt(i) * 57.29578);
            r.aileron = act_.aileron; r.elevator = act_.elevator; r.rudder = act_.rudder;
            r.cur_lim = ctl.currentLimit();
            r.omx = float(s.omega.x); r.omy = float(s.omega.y); r.omz = float(s.omega.z);
        }
        if (step_count_ % 10 == 0) accumulateReport(0.01);
        ++step_count_;
    }
    void run(double seconds, const std::function<bool(const QpFlightRig&)>& stop = nullptr) {
        const long n = long(seconds * 1000);
        for (long i = 0; i < n; ++i) { step(); if (stop && i % 100 == 0 && stop(*this)) break; if (sim.crashed() || sim.diverged()) break; }
    }
    const fc::QpActuators& lastActuators() const { return act_; }
    const fc::QpEstimate& lastEstimate() const { return last_e_; }   // що бачив контролер на останньому циклі
    double aiRmsError() const { return ai_stats.n > 0 ? std::sqrt(ai_stats.err2 / ai_stats.n) : 0.0; }     // Н
    double aiRmsTruth() const { return ai_stats.n > 0 ? std::sqrt(ai_stats.truth2 / ai_stats.n) : 0.0; }   // Н

    // повний журнал (10 Гц): перші 16 стовпців — як раніше (повтор польоту), далі — вітер, енергія, мотори, керма, обмежувач, кутові швидкості
    static const char* csvHeader() {
        return "t,x,y,z,vx,vy,vz,roll,pitch_down,yaw,airspeed,volts,amps,vtol_weight,phase,step,"
               "wind_x,wind_y,wind_z,power_w,soc,batt_temp_c,rps1,rps2,rps3,rps4,duty1,duty2,duty3,duty4,tilt1_deg,tilt2_deg,tilt3_deg,tilt4_deg,"
               "aileron,elevator,rudder,current_limit,p_rad_s,q_rad_s,r_rad_s";
    }
    static void csvRow(std::string& o, const FlightRow& r) {
        char b[640];
        std::snprintf(b, sizeof b, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.1f,%.2f,%d,%d,"
                      "%.2f,%.2f,%.2f,%.0f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
                      r.t, r.x, r.y, r.z, r.vx, r.vy, r.vz, r.roll, r.pitch_down, r.yaw, r.airspeed, r.volts, r.amps, r.wv, r.phase, r.step,
                      r.wx, r.wy, r.wz, r.power, r.soc, r.batt_temp, r.rps[0], r.rps[1], r.rps[2], r.rps[3],
                      r.duty[0], r.duty[1], r.duty[2], r.duty[3], r.tilt[0], r.tilt[1], r.tilt[2], r.tilt[3], r.aileron, r.elevator, r.rudder, r.cur_lim, r.omx, r.omy, r.omz);
        o += b;
    }
    std::string csvText() const {
        std::string o = csvHeader(); o += "\n";
        o.reserve(log.size() * 260);
        for (const FlightRow& r : log) csvRow(o, r);
        return o;
    }
    bool writeCsv(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        const std::string o = csvText();
        std::fwrite(o.data(), 1, o.size(), f);
        std::fclose(f);
        return true;
    }

    // текстовий звіт про політ (українською) — для вікна й файлу
    std::string reportText() const {
        const FlightReport& r = report;
        char b[512]; std::string o;
        auto L = [&](const char* fmt, auto... a) { if constexpr (sizeof...(a) == 0) o += fmt; else { std::snprintf(b, sizeof b, fmt, a...); o += b; } o += "\n"; };
        const double used = sim.battery().usedWh();
        L("ЗВІТ ПРО ПОЛІТ");
        L("Результат: %s", sim.crashed() ? "АВАРІЯ" : (ctl.phase() == fc::QpPhase::Landed ? "посадка" : "політ триває"));
        L("Тривалість: %.0f с (%.1f хв)", r.t, r.t / 60);
        L("Пройдено: %.2f км над землею, %.2f км у повітрі (вітер)", r.dist_ground / 1000, r.dist_air / 1000);
        L("Найдальше від дому: %.0f м;  посадка за %.1f м від дому, вертикальна швидкість у момент торкання %.1f м/с", r.max_home_dist,
          std::hypot(sim.state().pos.x, sim.state().pos.y), r.touch_vz);
        L("Висота макс.: %.0f м;  підйом до %.1f м/с, зниження до %.1f м/с", r.max_alt, r.max_climb, r.max_sink);
        L("Швидкість макс.: повітряна %.1f м/с, земна %.1f м/с;  нахил макс. %.0f°", r.max_airspeed, r.max_groundspeed, r.max_tilt);
        L("Вітер біля апарата: середній %.1f м/с, макс. %.1f м/с", r.samples ? r.wind_sum / double(r.samples) : 0.0, r.max_wind);
        L("");
        L("ЕНЕРГІЯ");
        L("Витрачено: %.1f Вт·год (%.0f%% ємності), лишилось %.0f%%;  середня потужність %.0f Вт", used, 100 * used / std::fmax(sim.battery().capacityAh() * 21.6, 1.0),
          100 * sim.battery().soc(), r.t > 0 ? used * 3600 / r.t : 0.0);
        L("Пікова потужність %.0f Вт, піковий струм %.0f А, мін. напруга %.1f В, макс. температура батареї %.0f °C", r.max_power, r.max_amps, r.min_volt < 1e8 ? r.min_volt : 0.0, r.max_temp);
        if (r.dist_ground > 100) L("Витрата: %.1f Вт·год/км", used / (r.dist_ground / 1000));
        L("");
        L("РЕЖИМИ");
        static const char* pn[] = {"на землі", "зліт", "висіння", "перехід у літак", "літак", "перехід у коптер", "посадка", "сів", "посадка літаком", "ручне"};
        for (int i = 0; i < 10; ++i) if (r.phase_t[i] > 0.5) {
            std::string name = pn[i];
            size_t cps = 0; for (unsigned char ch : name) cps += (ch & 0xC0) != 0x80;   // ширина в символах (UTF-8)
            name += std::string(cps < 20 ? 20 - cps : 1, ' ');
            std::snprintf(b, sizeof b, "  %s%6.0f с  (%2.0f%%)", name.c_str(), r.phase_t[i], 100 * r.phase_t[i] / std::fmax(r.t, 1.0)); o += b; o += "\n";
        }
        L("Переходів у літак: %d, у коптер: %d", r.trans_fwd, r.trans_back);
        L("");
        L("СХЕМА: %s; у польоті літаком тягнуть %s", sim.rearPusher() ? "задні гвинти штовхальні (під/за гондолою)" : "задні гвинти тягнучі (над/перед гондолою)",
          ctl.cfg.cruise_rear == 0 ? "лише передні" : (ctl.cfg.cruise_rear == 1 ? "усі 4" : "передні, задні — на допомогу"));
        L("");
        L("ПОДІЇ");
        if (r.events.empty()) L("  без аварійних станів");
        for (const std::string& e : r.events) o += "  " + e + "\n";
        return o;
    }

private:
    // ШІ: ознаки з того, що бачить контролер; правда — аеродинамічна сила моделі (лише для навчання й оцінки точності)
    // правда для порівняння (лише симулятор): аеродинамічна сила планера, тіло, Н
    Vec3d aiTruth() const { return sim.aeroNow().force; }
    void aiTick(const fc::QpEstimate& e) {
        const Vec3d F = aiTruth();
        const fc::Vec3 mf = advisor.meas.force();
        ai_meas[0] = mf.x; ai_meas[1] = mf.y; ai_meas[2] = mf.z;
        const ai::ForceInputs in = ai::ForceAdvisor::inputs(e, act_, ctl.massEstimate() / std::fmax(ctl.massNominal(), 0.1f));
        const double mg = ctl.massNominal() * QuadPlaneSim::kG;
        const bool airborne = sim.state().pos.z > sim.derived().cg_z + 1.0 && !sim.crashed();
        const bool frame = step_count_ % 20 == 0;   // кадр пам'яті й запису: 50 Гц
        if (ai_record && airborne && frame) {   // мітка — те, що виміряв би сам апарат (не правда симулятора)
            float feat[ai::kForceFeatures], label[ai::kForceOutputs] = {float(mf.x / mg), float(mf.y / mg), float(mf.z / mg)};
            ai::forceFeatures(in, feat);
            ai_record(feat, label, *this);
        }
        advisor.mode = ai_mode; advisor.self_check = ai_self_check; advisor.horizontal = ai_ff_horizontal;
        advisor.fade_speed = float(ai_fade_speed); advisor.trace_on = ai_trace_on;
        const fc::Vec3 u = advisor.tick(float(sim.time()), frame, in, ctl.massControl(), ctl.massNominal(), e.q);
        ctl.setAiAccel(u);
        if (ai_mode == 2 && advisor.has_pred && advisor.trust > 0.3f) ctl.ai_mass_meas = ctl.massNominal() - e.q.rotate(mf).z / 9.80665f;
        else ctl.ai_mass_meas = 0;
        for (int k = 0; k < 3; ++k) { ai_pred[k] = advisor.pred[k]; ai_check_pred[k] = advisor.check_pred[k]; ai_check_meas[k] = advisor.check_meas[k]; ai_used_acc[k] = advisor.used_acc[k]; }
        ai_trust = advisor.trust; ai_check_rms = advisor.check_rms; ai_fade = advisor.fade; ai_has_pred_ = advisor.has_pred;
        if (ai_trace_on) { ai_trace_valid = advisor.trace_valid; if (ai_trace_valid) ai_trace = advisor.trace; ai_truth[0] = float(F.x); ai_truth[1] = float(F.y); ai_truth[2] = float(F.z); }
        if (ai_mode > 0 && airborne && ctl.vtolWeight() > 0.99f && ai_has_pred_) {
            const double ex = ai_pred[0] - F.x, ey = ai_pred[1] - F.y, ez = ai_pred[2] - F.z;
            ai_stats.n += 1; ai_stats.err2 += ex * ex + ey * ey + ez * ez; ai_stats.truth2 += F.x * F.x + F.y * F.y + F.z * F.z;
        }
    }
    void accumulateReport(double dt) {
        FlightReport& r = report;
        const int ph = int(ctl.phase());
        const bool flying = ph != int(fc::QpPhase::OnGround) && !(ph == int(fc::QpPhase::Landed) && !ctl.manual.on);
        if (ph != r.prev_phase) {
            if (ph == int(fc::QpPhase::TransFwd)) ++r.trans_fwd;
            if (ph == int(fc::QpPhase::TransBack)) ++r.trans_back;
            r.prev_phase = ph;
        }
        if (!flying || sim.crashed()) return;
        const QpState& s = sim.state();
        const Vec3d v = sim.velWorld(), w = sim.windWorld(), e = sim.eulerRad();
        r.t += dt; ++r.samples;
        if (ph >= 0 && ph < 10) r.phase_t[ph] += dt;
        r.dist_ground += std::hypot(v.x, v.y) * dt; r.dist_air += sim.airspeed() * dt;
        r.max_alt = std::fmax(r.max_alt, s.pos.z); r.max_airspeed = std::fmax(r.max_airspeed, sim.airspeed()); r.max_groundspeed = std::fmax(r.max_groundspeed, std::hypot(v.x, v.y));
        r.max_climb = std::fmax(r.max_climb, v.z); r.max_sink = std::fmax(r.max_sink, -v.z);
        r.min_volt = std::fmin(r.min_volt, sim.busVoltage()); r.max_amps = std::fmax(r.max_amps, sim.busCurrent()); r.max_power = std::fmax(r.max_power, sim.busPower());
        r.max_tilt = std::fmax(r.max_tilt, std::acos(std::clamp(s.q.rotate(Vec3d{0, 0, 1}).z, -1.0, 1.0)) * 57.2958);
        r.max_temp = std::fmax(r.max_temp, sim.battery().temperature());
        r.max_wind = std::fmax(r.max_wind, std::hypot(w.x, w.y)); r.wind_sum += std::hypot(w.x, w.y);
        r.max_home_dist = std::fmax(r.max_home_dist, std::hypot(s.pos.x, s.pos.y));
        if (sim.groundContactForce() < 1.0) r.last_vz = -v.z; else if (r.touch_vz == 0 && r.t > 5) r.touch_vz = r.last_vz;
        (void)e;
        // події: кожен стан окремо; повторне ввімкнення в межах 5 с — той самий запис (обмежувач струму перемикається десятки разів за секунду)
        const std::string al = ctl.alertText();
        const double now = sim.time();
        size_t a = 0;
        bool changed = false;
        while (a < al.size()) {
            size_t b = al.find("; ", a);
            if (b == std::string::npos) b = al.size();
            const std::string what = al.substr(a, b - a);
            a = b + 2;
            if (what.empty()) continue;
            const bool was = r.prev_alert.find(what + "; ") != std::string::npos;
            FlightEvent* last = nullptr;
            for (auto it = r.ev.rbegin(); it != r.ev.rend(); ++it) if (it->what == what) { last = &*it; break; }
            if (last && now - last->t1 < 5.0) { if (!was) ++last->count; last->t1 = now; last->active_s += dt; }
            else { r.ev.push_back({what, now, now, dt, 1}); changed = true; }
            if (!was) changed = true;
        }
        r.prev_alert = al;
        if (changed || !al.empty()) {   // текст оновлюється, поки стан активний (кінець епізоду)
            r.events.clear();
            char b[128];
            for (const FlightEvent& e : r.ev) {
                if (e.count > 1 || e.t1 - e.t0 >= 1.0) std::snprintf(b, sizeof b, "%.0f–%.0f с: ", e.t0, e.t1);
                else std::snprintf(b, sizeof b, "%.0f с: ", e.t0);
                std::string line = b + e.what;
                if (e.count > 1) { std::snprintf(b, sizeof b, " (%d разів)", e.count); line += b; }
                r.events.push_back(line);
            }
        }
    }

    const AircraftFile& file_;
    const AeroTables& tables_;
    std::mt19937 rng_;
    prng::normal<double> nd_{0.0, 1.0};
    fc::QpActuators act_;
    QpControls u_;
    long step_count_ = 0;
    std::deque<QpState> hist_;
    QpState gps_;

    double n(double sigma) { return sigma > 0 ? sigma * nd_(rng_) : 0.0; }
    std::deque<QpControls> act_q_;
    fc::Vec3 imu_acc_{0, 0, 9.80665f};
    Vec3d f_true_{0, 0, QuadPlaneSim::kG}, prev_vw_{};
    fc::QpEstimate last_e_{};
    // Піто, ESC, енкодери гондол
    double pitot_dp_ = 0, pitot_off_ = 0, pitot_v_ = 0;
    bool aux_init_ = false;
    std::deque<std::array<double, 5>> rps_hist_;
    std::array<double, 5> esc_rps_{};
    std::array<double, 4> tilt_f_{};
    void sampleAux(double dt) {
        {
            const Vec3d vw = sim.velWorld();
            if (step_count_ > 0) f_true_ = sim.state().q.rotateInv((vw - prev_vw_) / dt + Vec3d{0, 0, QuadPlaneSim::kG});
            prev_vw_ = vw;
        }
        if (!aux_init_) {
            aux_init_ = true;
            if (noise.pitot_model) pitot_off_ = noise.pitot_offset_pa * (nd_(rng_) > 0 ? 1 : -1);
            for (int i = 0; i < 4; ++i) tilt_f_[size_t(i)] = sim.tilt(i);
        }
        if (noise.pitot_model) {
            const double rho = sim.density();
            const double vx = sim.airspeedVec(Vec3d{}).x;
            double dp = 0.5 * rho * vx * std::fabs(vx);
            if (sim.tiltEnabled()) {   // передні гвинти підсмоктують повітря повз приймач
                const double r = sim.vtolProp().diameter() / 2, A = 3.14159265358979323846 * r * r;
                for (int i : {0, 2}) {
                    const double T = std::fmax(sim.vtolThrust(i), 0.0), vi2 = T / (2 * rho * A);
                    dp += noise.pitot_wash_k * 0.5 * rho * vi2 * std::fmax(std::sin(sim.tilt(i)), 0.0);
                }
            }
            pitot_off_ += noise.pitot_drift_pa_s * std::sqrt(dt) * nd_(rng_);
            pitot_dp_ += (dp - pitot_dp_) * std::fmin(1.0, dt / std::fmax(noise.pitot_tau_s, 1e-4));
            if (noise.pitot_hz <= 0 || step_count_ % std::max(1, int(1000.0 / noise.pitot_hz)) == 0) {
                const double m = pitot_dp_ + pitot_off_ + n(noise.pitot_noise_pa);
                pitot_v_ = std::sqrt(2 * std::fmax(m, 0.0) / rho);
            }
        }
        if (noise.esc_model) {
            rps_hist_.push_back({sim.vtolRps(0), sim.vtolRps(1), sim.vtolRps(2), sim.vtolRps(3), sim.hasCruiseMotor() ? sim.cruiseRps() : 0.0});
            const size_t back = size_t(noise.esc_delay_s * 1000 + 0.5);
            while (rps_hist_.size() > back + 1) rps_hist_.pop_front();
            for (size_t i = 0; i < 5; ++i) {
                if (noise.esc_drop > 0 && prng::uniform<double>(0, 1)(rng_) < noise.esc_drop) continue;   // кадр загубився — старе значення
                double v = rps_hist_.front()[i] * (1.0 + n(noise.rpm_frac));
                if (noise.esc_quant_rps > 0) v = std::round(v / noise.esc_quant_rps) * noise.esc_quant_rps;
                esc_rps_[i] = v;
            }
        }
        if (noise.tilt_tau_s > 0 && sim.tiltEnabled())
            for (int i = 0; i < 4; ++i) tilt_f_[size_t(i)] += (sim.tilt(i) - tilt_f_[size_t(i)]) * std::fmin(1.0, dt / noise.tilt_tau_s);
    }

    const QpState& histAt(double delay_s) const {   // стан із затримкою (кадр = 1 мс)
        if (hist_.empty()) return sim.state();
        const size_t back = std::min<size_t>(size_t(delay_s * 1000 + 0.5), hist_.size() - 1);
        return hist_[hist_.size() - 1 - back];
    }

    fc::QpEstimate estimate() {
        const QpState& s = histAt(noise.delay_att_s);   // кути, гіро, висота (барометр)
        if (!sfaults.gps_lost && (noise.gps_hz <= 0 || step_count_ % std::max(1, int(1000.0 / noise.gps_hz)) == 0)) gps_ = histAt(noise.delay_gps_s);   // при відмові GPS дані застигають   // GPS: затримка й дискретність
        fc::QpEstimate e;
        const Vec3d v = gps_.q.rotate(gps_.vel_b);
        e.pos = fc::Vec3{float(gps_.pos.x + n(noise.pos_m)), float(gps_.pos.y + n(noise.pos_m)), float(s.pos.z + n(noise.pos_m * 0.5))};
        e.vel = fc::Vec3{float(v.x + n(noise.vel_mps)), float(v.y + n(noise.vel_mps)), float(s.q.rotate(s.vel_b).z + n(noise.vel_mps))};   // вертикальна швидкість — баро/IMU (без затримки GPS)
        fc::Quat q(float(s.q.w), float(s.q.x), float(s.q.y), float(s.q.z));
        if (noise.att_rad > 0) q = q * fc::Quat::fromEuler(float(n(noise.att_rad)), float(n(noise.att_rad)), float(n(noise.att_rad)));
        e.q = q;
        e.omega = fc::Vec3{float(s.omega.x + n(noise.gyro_rad_s)), float(s.omega.y + n(noise.gyro_rad_s)), float(s.omega.z + n(noise.gyro_rad_s))};
        e.airspeed = noise.pitot_model ? float(std::fmax(0.0, pitot_v_ + noise.airspeed_bias))
                                       : float(std::fmax(0.0, sim.airspeed() + noise.airspeed_bias + n(noise.airspeed_mps)));
        e.v_bus = float(sim.busVoltage());
        e.gps_ok = !sfaults.gps_lost;
        e.est_ok = !noise.use_ekf || nav.ready();
        e.accel = noise.use_ekf ? imu_acc_ : fc::Vec3{float(f_true_.x + n(0.3)), float(f_true_.y + n(0.3)), float(f_true_.z + n(0.3))};
        e.rc_ok = !sfaults.rc_lost;
        e.bus_current = float(sim.busCurrent());
        if (noise.use_ekf && nav.ready()) {   // оцінка з EKF замість «правда + шум»
            const fc::Eskf& L = nav.primary();
            e.pos = L.pos(); e.vel = L.vel(); e.q = L.att(); e.omega = nav.omega();
        }
        if (noise.esc_model) {
            for (int i = 0; i < 4; ++i) e.vtol_rps[i] = float(esc_rps_[size_t(i)]);
            e.cruise_rps = sim.hasCruiseMotor() ? float(esc_rps_[4]) : -1.0f;
        } else {
            for (int i = 0; i < 4; ++i) e.vtol_rps[i] = float(sim.vtolRps(i) * (1.0 + n(noise.rpm_frac)));   // телеметрія ESC (BLHeli32/AM32)
            e.cruise_rps = sim.hasCruiseMotor() ? float(sim.cruiseRps() * (1.0 + n(noise.rpm_frac))) : -1.0f;
        }
        if (sim.tiltEnabled()) for (int i = 0; i < 4; ++i) {   // датчик положення гондоли
            double t = (noise.tilt_tau_s > 0 ? tilt_f_[size_t(i)] : sim.tilt(i)) + n(noise.tilt_noise_rad);
            if (noise.tilt_quant_rad > 0) t = std::round(t / noise.tilt_quant_rad) * noise.tilt_quant_rad;
            e.tilt_meas[i] = float(t);
        }
        if (sfaults.pitot_dead) e.airspeed = 0;
        return e;
    }

    void buildConfig() {
        fc::QpFlightConfig c;
        const double rho = 1.225, vref = file_.num("battery.cells_series") * 4.0;   // 6S → 24 В
        const Propeller& vp = sim.vtolProp();
        const MotorEsc& vm = sim.vtolMotor();
        const bool cruise = sim.hasCruiseMotor();
        const double vi_max = file_.num("vtol_motor.max_cont_a") * 1.35, ci_max = cruise ? file_.num("cruise_motor.max_cont_a") * 1.35 : vi_max;
        auto build = [&](const Propeller& p, const MotorEsc& m, double v_axial, double i_max) {
            std::vector<float> duty, thr;
            for (int i = 0; i <= 40; ++i) {
                const double d = i / 40.0;
                const double nr = detail::steadyRps(m, p, d, vref, v_axial, i_max, rho);
                const double t = std::fmax(p.eval(nr, v_axial, 0.0, rho).thrust, i == 0 ? 0.0 : double(thr.back()) + 1e-3);
                duty.push_back(float(d)); thr.push_back(float(t));
            }
            fc::ThrustMap tm; tm.set(duty, thr, float(vref));
            return tm;
        };
        c.vtol_map = build(vp, vm, 0.0, vi_max);
        const bool rd = sim.rearDiffers();
        const Propeller& vpr = sim.vtolProp(1);
        const MotorEsc& vmr = sim.vtolMotor(1);
        const double vi_max_r = rd && file_.has("vtol_motor_rear.max_cont_a") ? file_.num("vtol_motor_rear.max_cont_a") * 1.35 : vi_max;
        if (rd) { c.rear_differs = true; c.vtol_map_r = build(vpr, vmr, 0.0, vi_max_r); }
        {   // очікувані оберти за duty (статика) — для контролю відмов за телеметрією ESC
            std::vector<float> duty, rps;
            for (int i = 0; i <= 40; ++i) {
                const double d = i / 40.0;
                duty.push_back(float(d));
                rps.push_back(float(std::fmax(detail::steadyRps(vm, vp, d, vref, 0.0, vi_max, rho), i == 0 ? 0.0 : double(rps.back()) + 1e-3)));
            }
            c.vtol_rps_map.set(duty, rps, float(vref));
            if (rd) {
                std::vector<float> rr;
                for (int i = 0; i <= 40; ++i) rr.push_back(float(std::fmax(detail::steadyRps(vmr, vpr, i / 40.0, vref, 0.0, vi_max_r, rho), i == 0 ? 0.0 : double(rr.back()) + 1e-3)));
                c.vtol_rps_map_r.set(duty, rr, float(vref));
            }
            if (cruise)
                for (double va : {0.0, 6.0, 10.0, 14.0, 18.0, 22.0}) {   // за повітряною швидкістю
                    std::vector<float> cr;
                    for (int i = 0; i <= 40; ++i)
                        cr.push_back(float(std::fmax(detail::steadyRps(sim.cruiseMotor(), sim.cruiseProp(), i / 40.0, vref, va, ci_max, rho), i == 0 ? 0.0 : double(cr.back()) + 1e-3)));
                    fc::ThrustMap m; m.set(duty, cr, float(vref)); c.cruise_rps_map.add(float(va), m);
                }
        }
        if (cruise) for (double v : {0.0, 6.0, 10.0, 14.0, 18.0, 22.0}) c.cruise_map.add(float(v), build(sim.cruiseProp(), sim.cruiseMotor(), v, ci_max));
        // мікшер: k_yaw = Q/T (м) на висінні, плечі моторів
        double n_h = 0;
        const double w4 = sim.mass() * QuadPlaneSim::kG / 4;
        for (double x = 20; x < 300; x += 0.05) if (vp.eval(x, 0, 0, rho).thrust < w4) n_h = x;
        const Propeller::Out ph = vp.eval(n_h, 0, 0, rho);
        c.af.mass = float(sim.mass());
        c.af.inertia = fc::Vec3{float(sim.inertia().xx), float(sim.inertia().yy), float(sim.inertia().zz)};
        // плечі — від ЦМ до осей гондол (мотор 2 — передній лівий, 1 — задній лівий)
        c.af.mixer.arm_x = float(sim.pivot(2).x); c.af.mixer.arm_x_rear = float(-sim.pivot(1).x);   // від ЦМ до осей передніх і задніх гондол
        c.af.mixer.arm_y = float(sim.pivot(2).y); c.af.mixer.arm_y_rear = float(sim.pivot(1).y);
        c.af.mixer.k_yaw = float(ph.torque / ph.thrust); c.af.mixer.max_thrust = c.vtol_map.maxThrust(); c.af.mixer.idle = 0.04f;
        if (rd) c.af.mixer.max_thrust_rear = c.vtol_map_r.maxThrust();
        // швидкості — з файлу апарата (єдине джерело)
        c.af.v_stall = float(file_.num("targets.v_stall_mps"));
        c.af.v_cruise = float(0.5 * (file_.num("targets.v_cruise_min_mps") + file_.num("targets.v_cruise_max_mps")));
        c.v_cruise = c.af.v_cruise;
        // пороги переходу масштабуються від швидкості звалювання (QuadPlane: Vs 10 → v_trans 12, v_back 10.5, assist 11)
        const float vs = c.af.v_stall;
        c.v_trans = 1.2f * vs; c.v_back = 1.05f * vs; c.assist_speed = 1.1f * vs;
        c.fw_land_speed = 1.3f * vs;
        c.v_max = std::fmax(c.v_cruise + 5.0f, 20.0f);
        if (cruise) c.cruise_thrust_max = std::fmin(float(file_.num("cruise_motor.thrust_limit_n")), c.cruise_map.maxThrust(0.0f));
        if (sim.tiltEnabled()) {   // конвертоплан: тяга вперед — нахилом гондол
            c.tilt = true;
            const double d2r = 3.14159265358979323846 / 180;
            c.tilt_min = float(file_.num("tilt.min_deg") * d2r); c.tilt_max = float(file_.num("tilt.max_deg") * d2r);
            c.tilt_rate = float(file_.num("tilt.rate_deg_s") * d2r); c.tilt_tau = float(file_.num("tilt.tau_s"));
            c.tilt_yaw_vector = file_.num("tilt.yaw_vector") > 0.5;
            c.cruise_rear = int(file_.num("tilt.cruise_rear") + 0.5);
            for (int i = 0; i < 4; ++i) c.motor_y[i] = float(sim.pivot(i).y);
            c.cruise_thrust_max = float(file_.num("tilt.fwd_thrust_limit_n"));
            c.tilt_brake_n = float(0.30 * sim.mass() * QuadPlaneSim::kG);   // ≈ нахил гондол назад на 17° (упор −25°) — гальмування без задирання носа
            c.back_decel = 2.5f;
            for (double v : {0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 25.0}) c.vtol_map2.add(float(v), build(vp, vm, v, vi_max));
            if (rd) for (double v : {0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 25.0}) c.vtol_map2_r.add(float(v), build(vpr, vmr, v, vi_max_r));
            c.yaw_diff_thrust = file_.has("tilt.yaw_diff_thrust") && file_.num("tilt.yaw_diff_thrust") > 0.5;
        }
        c.prop_d = float(vp.diameter());
        for (int i = 0; i < 4; ++i) c.prop_k[i] = float(sim.thrustFactor(i));
        for (int k = 0; k <= 24; ++k) { const double j = -0.3 + k * 0.05; c.prop_j.push_back(float(j)); c.prop_ct.push_back(float(vp.ct(j))); if (rd) c.prop_ct_r.push_back(float(vpr.ct(j))); }
        c.prop_d_r = float(vpr.diameter());
        if (file_.has("battery.i_max_a")) c.i_max = float(file_.num("battery.i_max_a"));
        if (file_.has("battery.v_low_v")) c.v_low = float(file_.num("battery.v_low_v"));
        if (file_.has("battery.r_est_ohm")) c.batt_r_est = float(file_.num("battery.r_est_ohm"));
        ctl.configure(c);
        advisor.net.select(file_.has("ai.force_weights") ? file_.str("ai.force_weights") : std::string());   // ваги моделі сил — під цей апарат
        // TECS: тримування за швидкістю (ідентифікація моделі, як TECS_THR_TRIM в ArduPilot)
        std::vector<float> vv, th, al;
        trim_v.clear(); trim_w.clear();
        struct TrimSaved { std::vector<float> vv, th, al, tv, tw; float hover_w = 0; };
        static std::mutex trim_mx;
        static std::map<std::string, TrimSaved> trim_cache;
        std::string key = aircraftKey(file_);
        {
            std::vector<double> kv{double(vref)};
            for (const char* name : {"aero_model.wing_table", "aero_model.tail_table"}) {
                const AirfoilTable& tbl = tables_.get(file_.str(name));
                for (double deg = -30; deg <= 40; deg += 2.5) { const auto pt = tbl.eval(deg * 0.017453292519943295); kv.push_back(pt.cl); kv.push_back(pt.cd); }
            }
            key.append(reinterpret_cast<const char*>(kv.data()), kv.size() * sizeof(double));
        }
        bool hit = false;
        {
            std::lock_guard<std::mutex> l(trim_mx);
            const auto it = trim_cache.find(key);
            if (it != trim_cache.end()) { vv = it->second.vv; th = it->second.th; al = it->second.al; trim_v = it->second.tv; trim_w = it->second.tw; hover_w = it->second.hover_w; hit = true; }
        }
        if (!hit) {
            { QuadPlaneSim tmp(file_, tables_); hover_w = float(hoverPower(tmp, vref).elec_w); }
            for (double v : {9.0, 10.0, 12.0, 14.0, 16.0, 17.0, 18.0, 20.0, 22.0}) {
                QuadPlaneSim tmp(file_, tables_);
                const TrimResult t = trimCruise(tmp, v, vref);
                if (!t.ok) continue;   // нижче звалювання тримування немає — точку не беремо
                vv.push_back(float(v)); th.push_back(float(t.cruise_thrust)); al.push_back(float(t.alpha));
                trim_v.push_back(float(v)); trim_w.push_back(float(t.elec_w));
            }
            std::lock_guard<std::mutex> l(trim_mx);
            if (trim_cache.size() > 64) trim_cache.clear();
            trim_cache[key] = TrimSaved{vv, th, al, trim_v, trim_w, hover_w};
        }
        ctl.tecs.setTrim(vv, th, al);
        ctl.tecs.setTrimMass(float(sim.mass()));
    }
};

}  // namespace sim
