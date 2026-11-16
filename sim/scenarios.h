// scenarios.h — бібліотека готових сценаріїв перевірки
#pragma once
#include <map>
#include "scenario.h"

namespace sim {

// Підвісити вантаж масою m (кг) у точці r (м, відносно центру дрона): змінює масу, центр мас та інерцію
inline void attachPayload(QuadParams& p, float m, Vec3 r) {
    float M = p.mass + m;
    p.com_offset = (p.com_offset * p.mass + r * m) / M;
    p.inertia += Vec3{m * (r.y * r.y + r.z * r.z), m * (r.x * r.x + r.z * r.z), m * (r.x * r.x + r.y * r.y)};
    p.mass = M;
}

// Контролер знає власний вантаж (і коли його скинули): маса й інерція — разом з ним
inline void tellFcMass(const QuadParams& p, fc::VehicleModel& vm) {
    vm.mass = p.mass;
    vm.inertia = p.inertia;
}

// Поставити на дрон планер (корпус / крило / кіль / тяговий гвинт): маса, інерція, центр мас.
// fc_vehicle (якщо задано) — те саме знає і контролер (свій планер він знає, на відміну від вантажу).
inline void applyAirframe(QuadParams& p, const fc::AirframeConfig& a, fc::VehicleModel* fc_vehicle = nullptr) {
    p.air = a;
    p.drag_xy = a.body_cd;
    const fc::WingConfig& w = a.wing;
    if (w.enabled) {
        const float m = w.mass();
        attachPayload(p, m, {w.x, 0, w.z});
        p.inertia += Vec3{m * w.span * w.span / 12, m * w.chord() * w.chord() / 12, m * (w.span * w.span + w.chord() * w.chord()) / 12};
    }
    if (a.fin.area > 0) attachPayload(p, a.fin.mass(), {-a.fin.arm, 0, a.fin.height * 0.5f});
    if (a.htail.area > 0) attachPayload(p, a.htail.mass(), {-a.htail.arm, 0, a.htail.z});
    if (a.pusher.enabled) attachPayload(p, a.pusher.mass, {a.pusher.x, 0, 0});
    if (a.surfaces.enabled) attachPayload(p, a.surfaces.mass, {-0.3f, 0, 0.03f});   // машинки на крилі й хвості
    if (fc_vehicle) {
        fc_vehicle->air = a;
        fc_vehicle->mass = p.mass;
        fc_vehicle->inertia = p.inertia;
        fc_vehicle->drag_quad = a.body_cd;
    }
}

inline std::map<std::string, ScenarioConfig> builtinScenarios() {
    std::map<std::string, ScenarioConfig> s;
    auto add = [&](const char* name, ScenarioConfig c) { c.name = name; s[name] = c; };
    const float D = fc::kDeg2Rad;

    { ScenarioConfig c; add("hover", c); }

    { ScenarioConfig c; c.duration = 18;  // постійний вітер 8 м/с, з'являється раптово на 8-й секунді
      c.on_step = [](float t, World& w) { w.quad.wind.mean = t >= 8 ? Vec3{8, 0, 0} : Vec3{}; };
      add("wind_step", c); }

    { ScenarioConfig c; c.duration = 25; c.wind.mean = {5, 3, 0}; c.wind.gust_sigma = 3.0f;
      add("turbulence", c); }

    { ScenarioConfig c; c.duration = 20; c.quad.com_offset = {0.03f, -0.02f, 0.0f};  // ЦМ зміщено на 3.6 см
      add("com_offset", c); }

    { ScenarioConfig c; c.duration = 20;  // на 10 с підвісили 1.5 кг вантажу (+33% маси) знизу і трохи збоку
      c.on_step = [](float t, World& w) {
          if (t >= 10 && w.quad.p.mass < 5.5f) attachPayload(w.quad.p, 1.5f, {0.0f, 0.06f, -0.15f});
      };
      add("payload_drop_on", c); }

    { ScenarioConfig c; c.duration = 20;  // мотор 2 втрачає 30% тяги на 10 с (пошкоджений пропелер)
      c.on_step = [](float t, World& w) { w.quad.p.motor_eff[2] = t >= 10 ? 0.7f : 1.0f; };
      add("motor_degraded", c); }

    { ScenarioConfig c; c.duration = 14;  // відпрацювання команд: крен 20° -> тангаж -15° -> рівно
      c.setpoint = [D](float t, fc::Setpoint& sp) {
          sp.roll = (t >= 7 && t < 9) ? 20 * D : 0;
          sp.pitch = (t >= 9 && t < 11) ? -15 * D : 0;
          sp.yaw_rate = (t >= 11 && t < 12) ? 90 * D : 0;
      };
      add("setpoint_tracking", c); }

    { ScenarioConfig c; c.imu.gyro_bias_dps = {3, -2.5f, 2}; c.imu.gyro_noise_dps = 0.6f;
      c.imu.vibration_g = 0.4f; c.imu.vibration_gyro_dps = 3.0f; c.imu.accel_bias_g = {0.05f, -0.04f, 0.03f};
      // дрейф гіроскопа після калібрування (прогрів): ще +2 °/с з 5-ї секунди
      c.on_step = [](float t, World& w) { if (t > 5) w.raw_bus.errors().gyro_bias_dps.x = 5.0f; };
      add("noisy_sensors", c); }

    { ScenarioConfig c; c.analog_imu = true; c.wind.gust_sigma = 1.5f; add("analog_imu", c); }

    { ScenarioConfig c; c.imu_fault = {8.0f, 8.04f}; c.wind.gust_sigma = 1.0f;  // 40 мс збій IMU
      add("imu_glitch", c); }

    { ScenarioConfig c; c.duration = 15;  // різкий поштовх (удар об гілку): 0.05 с, момент 1.5 Н·м
      c.on_step = [](float t, World& w) {
          w.quad.extra_torque_body = (t >= 8 && t < 8.05f) ? Vec3{13.0f, 5.0f, 0} : Vec3{};
      };
      add("impact_kick", c); }

    { ScenarioConfig c; c.quad.thrust_expo = 0.5f; c.quad.mass = 5.2f; c.quad.inertia = {0.15f, 0.14f, 0.27f};
      c.quad.arm_length = 0.33f; c.wind.gust_sigma = 1.5f;  // модель у контролері неточна
      add("model_mismatch", c); }

    // ---------- GPS: утримання позиції ----------
    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 20;
      c.on_step = [](float t, World& w) { w.quad.wind.mean = t >= 8 ? Vec3{8, 0, 0} : Vec3{}; };
      add("gps_wind_step", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 30; c.wind.mean = {5, 3, 0}; c.wind.gust_sigma = 3.0f;
      add("gps_turbulence", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 25; c.initial_yaw = 2.2f;  // ніс не на схід — перевірка компаса
      c.wind.mean = {-4, 6, 0}; c.wind.gust_sigma = 1.5f;
      c.on_step = [](float t, World& w) { if (t >= 10 && w.quad.p.mass < 5.5f) attachPayload(w.quad.p, 1.5f, {0, 0.06f, -0.15f}); };
      add("gps_heading_payload", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 30; c.wind.mean = {6, -3, 0}; c.wind.gust_sigma = 1.5f;
      c.gps_fault = {10, 16};  // 6 с без GPS
      add("gps_loss", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 22;  // політ стіками і зупинка
      c.setpoint = [](float t, fc::Setpoint& sp) { sp.vel_fwd = (t >= 7 && t < 11) ? 4.0f : 0; sp.vel_left = (t >= 7 && t < 9) ? 2.0f : 0; };
      add("gps_fly_and_stop", c); }

    // ---------- Без GPS: оптичний потік + далекомір ----------
    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 60; c.target_alt = 10;
      c.gps_fault = {0, 1e9f};  // GPS немає взагалі (глушіння / приміщення)
      c.wind.mean = {6, 0, 0}; c.wind.gust_sigma = 2.0f; c.wind.shear_exp = 0.14f;
      c.metrics_from = 10;
      add("noGPS_flow_10m", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 90; c.target_alt = 100;
      c.gps_fault = {0, 1e9f};
      c.wind.mean = {6, 0, 0}; c.wind.gust_sigma = 2.0f; c.wind.shear_exp = 0.14f;  // на 100 м ~8.5 м/с + пориви
      c.metrics_from = 40;
      add("noGPS_flow_100m", c); }

    // ---------- Батарея ----------
    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 45 * 60; c.target_alt = 30;  // висіння до розряду
      c.metrics_from = 1e9f;
      add("battery_hover_full", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 30 * 60; c.target_alt = 30;  // активний політ з вантажем
      attachPayload(c.quad, 1.5f, {0, 0.06f, -0.15f});
      c.wind.mean = {5, 0, 0}; c.wind.gust_sigma = 1.5f; c.wind.shear_exp = 0.14f;
      c.setpoint = [](float t, fc::Setpoint& sp) {
          int ph = int(t / 15) % 4;
          sp.altitude = 30 + 15 * std::sin(t * 0.05f);
          if (t > 8) { sp.vel_fwd = (ph == 0 || ph == 2) ? 8.0f : 0.0f; sp.yaw_rate = (ph == 1 || ph == 3) ? 0.3f : 0.0f; }
      };
      c.metrics_from = 1e9f;
      add("battery_active_payload", c); }

    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 200; c.target_alt = 25;  // відлетів на 60 м, батарея сіла -> додому
      c.battery.soc0 = 0.225f;
      c.wind.mean = {4, 2, 0}; c.wind.gust_sigma = 1.0f;
      c.setpoint = [](float t, fc::Setpoint& sp) { sp.altitude = 25; sp.vel_fwd = (t > 10 && t < 18) ? 8.0f : 0.0f; };
      c.metrics_from = 1e9f;
      add("battery_rtl", c); }

    // ---------- Висота ----------
    { ScenarioConfig c; c.mode = fc::Mode::PosHold; c.duration = 115;  // підйом на 100 м з вантажем, вітер посилюється з висотою
      attachPayload(c.quad, 1.5f, {0, 0.06f, -0.15f});
      c.wind.mean = {6, 2, 0}; c.wind.gust_sigma = 2.0f; c.wind.shear_exp = 0.14f;  // 6 м/с на 10 м -> ~8.5 м/с на 100 м
      c.setpoint = [](float t, fc::Setpoint& sp) { sp.altitude = (t >= 8 && t < 60) ? 100.0f : 2.0f; };
      c.metrics_from = 1e9f;  // висота змінюється — загальні метрики не рахуємо, перевірки окремо в тестах
      add("high_climb_100m", c); }

    return s;
}

}  // namespace sim
