// scenario.h — запуск сценарію: фізика 1 кГц, контролер 500 Гц, збір метрик і лог
#pragma once
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "../fc/flight_controller.h"
#include "sensors_sim.h"

namespace sim {

struct World;

struct ScenarioConfig {
    std::string name = "custom";
    float duration = 15;
    unsigned seed = 1;
    QuadParams quad;
    WindParams wind;
    ImuErrors imu;
    bool analog_imu = false;          // false = цифровий IMU, true = аналоговий через АЦП
    fc::FcConfig fc;
    fc::Mode mode = fc::Mode::AltHold;
    float arm_time = 1.5f;            // після калібрування
    float target_alt = 2.0f;
    float metrics_from = 6.0f;        // з якого моменту рахуємо метрики (після зльоту)
    FaultWindow imu_fault;
    FaultWindow gps_fault;            // втрата GPS
    FaultWindow flow_fault;           // втрата оптичного потоку
    bool use_flow = true;             // чи стоїть на дроні оптичний потік + далекомір
    BatteryParams battery;            // Li-ion 6S4P 18 А·год
    float initial_yaw = 0;            // рад — куди дивиться ніс на старті
    std::function<void(float t, World&)> on_step;           // події: вантаж, вітер, відмова мотора...
    std::function<void(float t, fc::Setpoint&)> setpoint;   // зміна уставок у часі
};

struct Metrics {
    bool crashed = false, crash_detected = false, failsafe_seen = false, armed_at_end = false;
    float max_tilt_err_deg = 0, rms_tilt_err_deg = 0, final_tilt_err_deg = 0;  // істинний нахил vs уставка
    float max_alt_err = 0, rms_alt_err = 0, final_alt = 0;
    float max_est_err_deg = 0;   // похибка оцінювача орієнтації vs істина
    float max_yaw_drift_deg = 0;
    float horiz_drift = 0;       // кінцеве відхилення від точки старту, м
    float max_horiz = 0;         // максимальне відхилення (після metrics_from), м
    float sat_fraction = 0;      // частка часу, коли мотори в насиченні
};

struct LogRow {
    float t, roll, pitch, yaw, est_roll, est_pitch, est_yaw, alt, est_alt, alt_sp, m[4], wind_x, wind_y;
};

struct World {
    QuadSim quad;
    SimRawImuBus raw_bus;
    SimAdc adc;
    SimBaro baro;
    SimGps gps;
    SimMag mag;
    SimFlow flow;
    SimRangefinder range;
    SimBattery battery;
    SimPower power;
    SimAirspeed airspeed;
    SimMotors motors;
    fc::DigitalImu dimu;
    fc::AnalogImu aimu;
    fc::FlightController fc;
    fc::Setpoint sp;

    explicit World(const ScenarioConfig& c)
        : quad(c.seed), raw_bus(quad, c.imu, c.seed + 1), adc(quad, c.imu, c.seed + 2), baro(quad, c.seed + 3),
          gps(quad, c.seed + 4), mag(quad, c.seed + 5), flow(quad, c.seed + 6), range(quad, c.seed + 7), battery(quad, c.seed + 8),
          power(quad, flow, range), airspeed(quad, c.seed + 9), motors(quad),
          dimu(raw_bus), aimu(adc),
          fc(c.analog_imu ? static_cast<fc::IImu&>(aimu) : static_cast<fc::IImu&>(dimu), &baro, motors, c.fc) {
        quad.p = c.quad;
        quad.wind = c.wind;
        quad.bat = c.battery;
        quad.bat.avionics_w = fc::avionicsWatts(power.state, power.spec);
        quad.soc = c.battery.soc0;
        quad.volts = liionOcv(quad.soc) * c.battery.cells;
        quad.soc_init = true;
        raw_bus.fault = c.imu_fault;
        adc.fault = c.imu_fault;
        gps.fault = c.gps_fault;
        quad.s.q = Quat::fromEuler(0, 0, c.initial_yaw);
        fc.attachGps(&gps);
        fc.attachMag(&mag);
        fc.attachBattery(&battery);
        fc.attachPower(&power);
        if (c.quad.air.pitot && c.quad.air.wing.enabled) fc.attachAirspeed(&airspeed);
        flow.fault = c.flow_fault;
        if (c.use_flow) { fc.attachFlow(&flow); fc.attachRangefinder(&range); }
    }
};

inline float tiltErrDeg(const Quat& q_true, float roll_sp, float pitch_sp) {
    Vec3 e = q_true.euler();
    Quat qsp = Quat::fromEuler(roll_sp, pitch_sp, e.z);
    float c = q_true.rotate({0, 0, 1}).dot(qsp.rotate({0, 0, 1}));
    return std::acos(fc::clampf(c, -1, 1)) * fc::kRad2Deg;
}

inline Metrics runScenario(const ScenarioConfig& c, std::vector<LogRow>* log = nullptr) {
    auto w = std::make_unique<World>(c);
    const float dt_phys = 0.001f;
    const int fc_div = 2;  // 500 Гц
    const int steps = int(c.duration / dt_phys);
    Metrics m;
    double sum_tilt2 = 0, sum_alt2 = 0;
    int n = 0, n_sat = 0, n_all = 0;
    bool armed_once = false;
    float yaw0 = 0;
    w->fc.setMode(c.mode);

    for (int k = 0; k < steps; ++k) {
        float t = w->quad.t;
        if (c.on_step) c.on_step(t, *w);
        if (k % fc_div == 0) {
            if (!armed_once && t >= c.arm_time && w->fc.telemetry().calibrated) {
                armed_once = w->fc.arm();
                yaw0 = w->quad.s.q.euler().z;
            }
            w->sp.altitude = c.target_alt;
            if (c.setpoint) c.setpoint(t, w->sp);
            w->fc.setSetpoint(w->sp);
            w->fc.update(dt_phys * fc_div);
        }
        w->quad.step(w->motors.cmd, dt_phys);
        const auto& tel = w->fc.telemetry();
        if (tel.failsafe) m.failsafe_seen = true;
        if (w->quad.crashed) m.crashed = true;

        if (t >= c.metrics_from) {
            float te = tiltErrDeg(w->quad.s.q, w->sp.roll, w->sp.pitch);
            float ae = w->quad.s.pos.z - c.target_alt;
            Vec3 et = w->quad.s.q.euler(), ee = tel.euler;
            float est = std::max(std::fabs(et.x - ee.x), std::fabs(et.y - ee.y)) * fc::kRad2Deg;
            float yd = std::fabs(std::remainder(et.z - yaw0, 2 * fc::kPi)) * fc::kRad2Deg;
            m.max_tilt_err_deg = std::max(m.max_tilt_err_deg, te);
            m.max_alt_err = std::max(m.max_alt_err, std::fabs(ae));
            m.max_est_err_deg = std::max(m.max_est_err_deg, est);
            m.max_horiz = std::max(m.max_horiz, std::sqrt(w->quad.s.pos.x * w->quad.s.pos.x + w->quad.s.pos.y * w->quad.s.pos.y));
            if (c.setpoint == nullptr) m.max_yaw_drift_deg = std::max(m.max_yaw_drift_deg, yd);
            sum_tilt2 += te * te;
            sum_alt2 += ae * ae;
            ++n;
            m.final_tilt_err_deg = te;
        }
        if (tel.armed) { ++n_all; if (tel.sat_attitude) ++n_sat; }
        if (log && k % 10 == 0) {
            Vec3 e = w->quad.s.q.euler() * fc::kRad2Deg, ee = tel.euler * fc::kRad2Deg;
            LogRow r{t, e.x, e.y, e.z, ee.x, ee.y, ee.z, w->quad.s.pos.z, tel.alt, c.target_alt,
                     {w->motors.cmd[0], w->motors.cmd[1], w->motors.cmd[2], w->motors.cmd[3]},
                     w->quad.wind_now.x, w->quad.wind_now.y};
            log->push_back(r);
        }
        if (m.crashed) break;
    }
    m.crash_detected = w->fc.telemetry().crash_detected;
    m.armed_at_end = w->fc.telemetry().armed;
    m.rms_tilt_err_deg = n ? float(std::sqrt(sum_tilt2 / n)) : 0;
    m.rms_alt_err = n ? float(std::sqrt(sum_alt2 / n)) : 0;
    m.final_alt = w->quad.s.pos.z;
    m.horiz_drift = std::sqrt(w->quad.s.pos.x * w->quad.s.pos.x + w->quad.s.pos.y * w->quad.s.pos.y);
    m.sat_fraction = n_all ? float(n_sat) / n_all : 0;
    return m;
}

inline bool writeCsv(const char* path, const std::vector<LogRow>& log) {
    FILE* f = std::fopen(path, "w");
    if (!f) return false;
    std::fprintf(f, "t,roll,pitch,yaw,est_roll,est_pitch,est_yaw,alt,est_alt,alt_sp,m0,m1,m2,m3,wind_x,wind_y\n");
    for (auto& r : log)
        std::fprintf(f, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n", r.t, r.roll,
                     r.pitch, r.yaw, r.est_roll, r.est_pitch, r.est_yaw, r.alt, r.est_alt, r.alt_sp, r.m[0], r.m[1],
                     r.m[2], r.m[3], r.wind_x, r.wind_y);
    std::fclose(f);
    return true;
}

inline void printMetrics(const char* name, const Metrics& m) {
    std::printf("%-26s crash=%d tilt_err max=%5.2f° rms=%5.2f° final=%5.2f° | alt_err max=%5.2fm rms=%5.3fm | est_err=%4.2f° yaw_drift=%5.2f° sat=%4.1f%% fs=%d | зноси max=%5.2fm final=%5.2fm\n",
                name, m.crashed || m.crash_detected, m.max_tilt_err_deg, m.rms_tilt_err_deg, m.final_tilt_err_deg,
                m.max_alt_err, m.rms_alt_err, m.max_est_err_deg, m.max_yaw_drift_deg, m.sat_fraction * 100,
                m.failsafe_seen, m.max_horiz, m.horiz_drift);
}

}  // namespace sim
