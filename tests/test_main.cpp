// test_main.cpp — автотести: юніт-тести блоків + сценарії + Монте-Карло.
// Код повернення 0 = все пройдено.
#include <cstdio>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <cstdlib>
#include <algorithm>
#include <initializer_list>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include "../sim/console_utf8.h"
#include "../sim/scenarios.h"
#include "../sim/weather.h"
#include "../sim/aircraft_params.h"
#include "../sim/aero_tables.h"
#include "../sim/qp_aero.h"
#include "../sim/qp_battery.h"
#include "../sim/qp_sim.h"
#include "../sim/qp_analysis.h"
#include "../sim/qp_flight.h"

using namespace sim;
using fc::Vec3;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (cond) { ++g_pass; }                                            \
        else { ++g_fail; std::printf("  FAIL %s:%d  %s  -> ", __FILE__, __LINE__, #cond); \
               std::printf(__VA_ARGS__); std::printf("\n"); }              \
    } while (0)
#define SECTION(name) std::printf("\n== %s ==\n", name)
#include "test_physics.h"
#include "test_ai.h"   // ШІ-співпроцесор (ai/): потребує CHECK і SECTION
#include "test_realism.h"   // реалістичні приводи/датчики, безпека польоту
#include "../sim/portable_rng.h"

// ---------------- Юніт-тести ----------------
static void testMath() {
    SECTION("Математика (кватерніони)");
    fc::Quat q = fc::Quat::fromEuler(0.3f, -0.2f, 1.1f);
    Vec3 e = q.euler();
    CHECK(std::fabs(e.x - 0.3f) < 1e-4f && std::fabs(e.y + 0.2f) < 1e-4f && std::fabs(e.z - 1.1f) < 1e-4f,
          "euler=(%f %f %f)", e.x, e.y, e.z);
    Vec3 v{1, 2, 3}, r = q.rotateInv(q.rotate(v));
    CHECK((r - v).norm() < 1e-5f, "rotate/rotateInv не обернені");
    fc::Quat qi;
    for (int i = 0; i < 1000; ++i) qi.integrate({0, 0, fc::kPi / 2}, 0.001f);  // 90°/с * 1 с
    CHECK(std::fabs(qi.euler().z - fc::kPi / 2) < 1e-3f, "інтегрування yaw=%f", qi.euler().z);
}

static void testMixer() {
    SECTION("Мікшер");
    fc::MixerConfig c;
    auto r = fc::mixQuadX(4 * 3.0f, Vec3{}, c);
    bool eq = true;
    for (int i = 0; i < 4; ++i) eq &= std::fabs(r.thrust[i] - 3.0f) < 1e-5f;
    CHECK(eq && !r.sat_attitude, "чиста тяга має ділитись порівну");

    // Відновлюємо моменти з тяг моторів тією ж фізикою, що в симуляторі
    Vec3 tq{0.3f, -0.2f, 0.05f};
    r = fc::mixQuadX(12.0f, tq, c);
    const float d = c.arm_length * 0.70710678f;
    Vec3 back;
    float T = 0;
    for (int i = 0; i < 4; ++i) {
        back.x += fc::kQuadX[i].sy * d * r.thrust[i];
        back.y += -fc::kQuadX[i].sx * d * r.thrust[i];
        back.z += fc::kQuadX[i].yaw_sign * c.k_yaw * r.thrust[i];
        T += r.thrust[i];
    }
    CHECK((back - tq).norm() < 1e-4f && std::fabs(T - 12) < 1e-4f, "момент назад=(%f %f %f) T=%f", back.x, back.y, back.z, T);

    // Насичення: величезний yaw + roll -> roll має зберегтись, yaw урізатись
    r = fc::mixQuadX(14.0f, Vec3{0.8f, 0, 2.0f}, c);
    back = Vec3{};
    for (int i = 0; i < 4; ++i) {
        back.x += fc::kQuadX[i].sy * d * r.thrust[i];
        back.z += fc::kQuadX[i].yaw_sign * c.k_yaw * r.thrust[i];
    }
    CHECK(r.sat_attitude && std::fabs(back.x - 0.8f) < 1e-3f && back.z < 2.0f, "roll=%f yaw=%f", back.x, back.z);
    bool in_range = true;
    for (float m : r.cmd) in_range &= (m >= c.idle - 1e-6f && m <= 1 + 1e-6f);
    CHECK(in_range, "команди поза діапазоном");
}

static void testPid() {
    SECTION("PID");
    fc::Pid p({1.0f, 10.0f, 0.0f, 0.5f, 1e9f, 0});
    for (int i = 0; i < 1000; ++i) p.update(1.0f, 0.0f, 0.01f);
    CHECK(std::fabs(p.integral() - 0.5f) < 1e-5f, "анти-віндап: I=%f", p.integral());
    fc::Pid d({0, 0, 1.0f, 1, 1e9f, 0});
    d.update(0, 0, 0.01f);
    float out = d.update(10.0f, 0.0f, 0.01f);  // різка зміна уставки
    CHECK(std::fabs(out) < 1e-6f, "D по вимірюванню не повинен «бити» від зміни уставки: %f", out);
    fc::Pid f({0, 1.0f, 0, 10, 1e9f, 0});
    for (int i = 0; i < 100; ++i) f.update(1.0f, 0.0f, 0.01f, false);
    CHECK(f.integral() == 0, "заморожений інтеграл змінився");
}

struct FakeRaw : fc::IRawImuBus {
    int16_t g[3], a[3];
    FakeRaw(std::initializer_list<int16_t> gg, std::initializer_list<int16_t> aa) {
        std::copy(gg.begin(), gg.end(), g); std::copy(aa.begin(), aa.end(), a);
    }
    bool readRaw(int16_t gg[3], int16_t aa[3]) override { for (int i = 0; i < 3; ++i) { gg[i] = g[i]; aa[i] = a[i]; } return true; }
};
struct FakeAdc : fc::IAdc {
    uint16_t c[6];
    FakeAdc(std::initializer_list<uint16_t> v) { std::copy(v.begin(), v.end(), c); }
    bool sample(uint16_t* o, int n) override { for (int i = 0; i < n; ++i) o[i] = c[i]; return true; }
};

static void testDrivers() {
    SECTION("Драйвери сенсорів (цифровий / аналоговий)");
    FakeRaw raw{{164, -328, 0}, {0, 0, 4096}};
    fc::DigitalImu dimu(raw);
    fc::ImuSample s;
    CHECK(dimu.read(s), "read");
    CHECK(std::fabs(s.gyro.x * fc::kRad2Deg - 10) < 1e-3f && std::fabs(s.gyro.y * fc::kRad2Deg + 20) < 1e-3f,
          "gyro=(%f %f)", s.gyro.x * fc::kRad2Deg, s.gyro.y * fc::kRad2Deg);
    CHECK(std::fabs(s.accel.z - fc::kGravity) < 1e-4f, "accel z=%f", s.accel.z);

    fc::DigitalImu::Config cfg;  // чип повернуто на 90°: X чипа = -Y дрона
    cfg.axis_map[0] = 1; cfg.axis_sign[0] = -1;
    cfg.axis_map[1] = 0;
    fc::DigitalImu rot(raw, cfg);
    rot.read(s);
    CHECK(std::fabs(s.gyro.x * fc::kRad2Deg - 20) < 1e-3f && std::fabs(s.gyro.y * fc::kRad2Deg - 10) < 1e-3f, "axis map");

    // Аналог: 1.65 В = 0; 1.65+0.3 В = 1 g; 1.65+0.025 В = 10 °/с
    const float k = 4095.0f / 3.3f;
    FakeAdc adc{{uint16_t(std::lround(1.65f * k)), uint16_t(std::lround(1.675f * k)), uint16_t(std::lround(1.65f * k)),
                 uint16_t(std::lround(1.65f * k)), uint16_t(std::lround(1.65f * k)), uint16_t(std::lround(1.95f * k))}};
    fc::AnalogImu aimu(adc);
    aimu.read(s);
    CHECK(std::fabs(s.gyro.y * fc::kRad2Deg - 10) < 0.5f, "аналоговий гіроскоп %f °/с", s.gyro.y * fc::kRad2Deg);
    CHECK(std::fabs(s.accel.z - fc::kGravity) < 0.05f, "аналоговий акселерометр %f", s.accel.z);
}

static void testEstimators() {
    SECTION("Оцінювачі");
    fc::AttitudeEstimator est;
    const float tilt = 10 * fc::kDeg2Rad;
    Vec3 acc{0, fc::kGravity * std::sin(tilt), fc::kGravity * std::cos(tilt)};  // roll = +10°
    Vec3 bias{1.5f * fc::kDeg2Rad, -1.0f * fc::kDeg2Rad, 0};
    est.reset(Vec3{0, 0, fc::kGravity});  // стартуємо з неправильного «рівно»
    std::mt19937 rng(3);
    prng::normal<float> n(0, 0.3f);
    for (int i = 0; i < 500 * 120; ++i) est.update(bias + Vec3{n(rng), n(rng), n(rng)} * 0.02f, acc + Vec3{n(rng), n(rng), n(rng)}, 0.002f);
    Vec3 e = est.euler() * fc::kRad2Deg;
    CHECK(std::fabs(e.x - 10) < 0.5f && std::fabs(e.y) < 0.5f, "нахил зійшовся до (%f %f), чекали (10 0)", e.x, e.y);
    Vec3 b = est.gyroBias() * fc::kRad2Deg;
    CHECK(std::fabs(b.x - 1.5f) < 0.3f && std::fabs(b.y + 1.0f) < 0.3f, "дрейф гіроскопа оцінено як (%f %f)", b.x, b.y);

    fc::AltitudeEstimator alt(0.8f);
    alt.reset(0);
    for (int i = 0; i < 500 * 20; ++i) {
        if (i % 10 == 0) alt.baro(5.0f + n(rng) * 0.3f);
        alt.update(0.3f, 0.002f);  // акселерометр зі зміщенням 0.3 м/с²
    }
    CHECK(std::fabs(alt.z() - 5.0f) < 0.15f && std::fabs(alt.vz()) < 0.1f && std::fabs(alt.accelBias() + 0.3f) < 0.05f,
          "висота %f, швидкість %f, bias %f", alt.z(), alt.vz(), alt.accelBias());
}

static void testArming() {
    SECTION("Логіка армування");
    ScenarioConfig c;
    World w(c);
    CHECK(!w.fc.arm(), "армування до калібрування заборонене");
    for (int i = 0; i < 600; ++i) { w.fc.update(0.002f); w.quad.step(w.motors.cmd, 0.002f); }
    CHECK(w.fc.telemetry().calibrated, "калібрування не завершилось");
    CHECK(w.motors.cmd[0] == 0, "мотори мають бути вимкнені до армування");
    CHECK(w.fc.arm(), "армування після калібрування");
}

// ---------------- Сценарії ----------------
struct Limits { float tilt_max, tilt_final, alt_max, est_max; };

static void checkScenario(const char* name, const Limits& L) {
    auto lib = builtinScenarios();
    Metrics m = runScenario(lib[name]);
    printMetrics(name, m);
    CHECK(!m.crashed && !m.crash_detected && m.armed_at_end, "%s: аварія", name);
    CHECK(m.max_tilt_err_deg < L.tilt_max, "%s: max нахил %.2f >= %.2f", name, m.max_tilt_err_deg, L.tilt_max);
    CHECK(m.final_tilt_err_deg < L.tilt_final, "%s: кінцевий нахил %.2f >= %.2f", name, m.final_tilt_err_deg, L.tilt_final);
    CHECK(m.max_alt_err < L.alt_max, "%s: висота %.2f >= %.2f", name, m.max_alt_err, L.alt_max);
    CHECK(m.max_est_err_deg < L.est_max, "%s: похибка оцінювача %.2f >= %.2f", name, m.max_est_err_deg, L.est_max);
}

static void testScenarios() {
    SECTION("Сценарії (метрики по ІСТИННОМУ стану дрона)");
    //                          tilt_max tilt_final alt_max est_max
    checkScenario("hover",            {1.0f,  0.5f, 0.15f, 1.0f});
    checkScenario("wind_step",        {10.f,  4.0f, 0.30f, 10.f});
    checkScenario("turbulence",       {10.f,  8.0f, 0.30f, 10.f});
    checkScenario("com_offset",       {4.0f,  1.5f, 0.15f, 3.0f});
    checkScenario("payload_drop_on",  {15.f,  2.0f, 0.80f, 4.0f});
    checkScenario("motor_degraded",   {10.f,  2.0f, 0.30f, 3.0f});
    checkScenario("noisy_sensors",    {12.f,  8.0f, 0.25f, 12.f});
    checkScenario("analog_imu",       {5.0f,  2.0f, 0.15f, 4.0f});
    checkScenario("imu_glitch",       {5.0f,  2.0f, 0.20f, 4.0f});
    checkScenario("impact_kick",      {25.f,  1.0f, 0.20f, 3.0f});
    checkScenario("model_mismatch",   {6.0f,  2.0f, 0.20f, 4.0f});
}

static void testTracking() {
    SECTION("Відпрацювання команди: крен 20°");
    auto c = builtinScenarios()["setpoint_tracking"];
    std::vector<LogRow> log;
    Metrics m = runScenario(c, &log);
    float t90 = -1, peak = 0, roll_end = 0;
    for (auto& r : log) {
        if (r.t >= 7 && r.t < 9) {
            if (t90 < 0 && r.roll >= 18) t90 = r.t - 7;
            peak = std::max(peak, r.roll);
            roll_end = r.roll;
        }
    }
    std::printf("  час до 90%%: %.3f с, пік: %.1f°, наприкінці: %.1f°\n", t90, peak, roll_end);
    CHECK(!m.crashed && !m.crash_detected, "аварія");
    CHECK(t90 > 0 && t90 < 0.4f, "повільна реакція: %.3f с", t90);
    CHECK(peak < 23.5f, "перерегулювання: пік %.1f° (команда 20°)", peak);
    CHECK(std::fabs(roll_end - 20) < 2.5f, "не тримає кут: %.1f°", roll_end);
    CHECK(m.max_alt_err < 0.4f, "висота під час маневрів %.2f", m.max_alt_err);
}

static void testFailsafe() {
    SECTION("Збої");
    {  // довгий збій IMU: має спрацювати failsafe і після відновлення дрон знову стабілізується
        ScenarioConfig c;
        c.imu_fault = {8.0f, 8.15f};
        c.duration = 16;
        Metrics m = runScenario(c);
        printMetrics("imu_fault_150ms", m);
        CHECK(m.failsafe_seen, "failsafe не спрацював");
        CHECK(!m.crashed && m.final_tilt_err_deg < 2, "не відновився після збою");
    }
    {  // мотор повністю відмовив -> квадрокоптер не втримати, але контролер має це розпізнати і вимкнути мотори
        ScenarioConfig c;
        c.duration = 14;
        c.on_step = [](float t, World& w) { if (t > 8) w.quad.p.motor_eff[0] = 0; };
        Metrics m = runScenario(c);
        printMetrics("motor_dead", m);
        CHECK(m.crash_detected || m.crashed, "відмова мотора мала призвести до падіння/детекції");
        CHECK(!m.armed_at_end, "мотори мали бути вимкнені після аварії");
    }
}

static void testGps() {
    SECTION("GPS: утримання позиції");
    auto lib = builtinScenarios();
    struct G { const char* name; float max_h, final_h; };
    // пороги з урахуванням похибки самого GPS (~1–1.5 м, як у звичайного приймача)
    for (G g : {G{"gps_wind_step", 3.0f, 2.0f}, G{"gps_turbulence", 4.0f, 2.5f}, G{"gps_heading_payload", 3.5f, 2.0f},
                G{"gps_loss", 12.0f, 2.5f}}) {
        Metrics m = runScenario(lib[g.name]);
        printMetrics(g.name, m);
        CHECK(!m.crashed && !m.crash_detected, "%s: аварія", g.name);
        CHECK(m.max_horiz < g.max_h, "%s: знесло %.2f м >= %.2f", g.name, m.max_horiz, g.max_h);
        CHECK(m.horiz_drift < g.final_h, "%s: кінцеве відхилення %.2f м >= %.2f", g.name, m.horiz_drift, g.final_h);
    }
    {  // Якість саме керування: ідеальний GPS (без шуму), сильний вітер — тримає точку в межах 1–2 м при раптовому пориві 8 м/с, потім точно повертається
        auto c = lib["gps_wind_step"];
        auto old = c.on_step;
        c.on_step = [old](float t, World& w) { w.gps.noise_m = 0; w.gps.walk_m = 0; w.gps.vel_noise = 0; old(t, w); };
        Metrics m = runScenario(c);
        printMetrics("gps_ideal_wind8", m);
        CHECK(m.max_horiz < 2.0f && m.horiz_drift < 0.3f, "ідеальний GPS: max %.2f final %.2f", m.max_horiz, m.horiz_drift);
    }
    {  // Без GPS той самий вітер зносить дрон — для порівняння
        auto c = lib["gps_wind_step"];
        c.mode = fc::Mode::AltHold;
        Metrics m = runScenario(c);
        std::printf("  для порівняння без GPS: знесло на %.1f м\n", m.horiz_drift);
        CHECK(m.horiz_drift > 20, "без GPS мало б знести");
    }
    {  // Політ стіками і зупинка: після відпускання гальмує і стоїть
        auto c = lib["gps_fly_and_stop"];
        std::vector<LogRow> log;
        float stop_pos[2] = {0, 0}, end_pos[2] = {0, 0};
        float travelled = 0;
        auto old = c.on_step;
        c.on_step = [&](float t, World& w) {
            if (std::fabs(t - 14.0f) < 0.0005f) { stop_pos[0] = w.quad.s.pos.x; stop_pos[1] = w.quad.s.pos.y; }
            end_pos[0] = w.quad.s.pos.x; end_pos[1] = w.quad.s.pos.y;
            travelled = std::sqrt(end_pos[0] * end_pos[0] + end_pos[1] * end_pos[1]);
        };
        Metrics m = runScenario(c);
        float creep = std::hypot(end_pos[0] - stop_pos[0], end_pos[1] - stop_pos[1]);
        std::printf("  пролетів %.1f м, після зупинки зсунувся на %.2f м\n", travelled, creep);
        CHECK(!m.crashed && travelled > 12, "мав пролетіти >12 м, пролетів %.1f", travelled);
        CHECK(creep < 1.5f, "після зупинки повзе: %.2f м", creep);
    }
}

static void testAltitude() {
    SECTION("Висота: підйом на 100 м з вантажем 1.5 кг, вітер посилюється з висотою");
    auto c = builtinScenarios()["high_climb_100m"];
    float reach = -1, peak = 0, hold_err = 0, wind_top = 0, horiz = 0, end_z = 0;
    c.on_step = [&](float t, World& w) {
        float z = w.quad.s.pos.z;
        if (t > 8 && t < 60) {
            peak = std::max(peak, z);
            if (reach < 0 && z > 99) reach = t - 8;
            if (reach > 0 && t > 8 + reach + 5) {
                hold_err = std::max(hold_err, std::fabs(z - 100));
                horiz = std::max(horiz, std::hypot(w.quad.s.pos.x, w.quad.s.pos.y));
                wind_top = std::max(wind_top, w.quad.wind_now.norm());
            }
        }
        end_z = z;
    };
    Metrics m = runScenario(c);
    std::printf("  набір 100 м за %.1f с, перельот %.2f м, тримає висоту ±%.2f м, вітер нагорі до %.1f м/с, зноси %.1f м, після спуску %.2f м\n",
                reach, peak - 100, hold_err, wind_top, horiz, end_z);
    CHECK(!m.crashed && !m.crash_detected, "аварія");
    CHECK(reach > 0 && reach < 35, "довго набирає висоту: %.1f с", reach);
    CHECK(peak - 100 < 1.0f, "перельот висоти %.2f м", peak - 100);
    CHECK(hold_err < 0.6f, "погано тримає висоту: %.2f м", hold_err);
    CHECK(horiz < 4.0f, "на висоті зносить: %.1f м", horiz);
    CHECK(std::fabs(end_z - 2) < 0.6f, "після спуску висота %.2f замість 2 м", end_z);
}

static void testNoGps() {
    SECTION("Без GPS: утримання точки по оптичному потоку (камера вниз) + далекомір");
    auto lib = builtinScenarios();
    struct G { const char* name; float max_h; };
    for (G g : {G{"noGPS_flow_10m", 1.5f}, G{"noGPS_flow_100m", 8.0f}}) {
        auto c = lib[g.name];
        Vec3 p0;
        bool got = false;
        float maxd = 0, fin = 0;
        c.on_step = [&](float t, World& w) {
            if (t < c.metrics_from) return;
            if (!got) { got = true; p0 = w.quad.s.pos; }
            float d = std::hypot(w.quad.s.pos.x - p0.x, w.quad.s.pos.y - p0.y);
            maxd = std::max(maxd, d);
            fin = d;
        };
        Metrics m = runScenario(c);
        std::printf("  %-18s відхилення max %.2f м, наприкінці %.2f м (за %.0f с, вітер з поривами)\n", g.name, maxd, fin,
                    c.duration - c.metrics_from);
        CHECK(!m.crashed && !m.crash_detected, "%s: аварія", g.name);
        CHECK(maxd < g.max_h, "%s: знесло %.2f м", g.name, maxd);
    }
    {  // для порівняння: ні GPS, ні потоку — тримається лише нахил «наосліп»
        auto c = lib["noGPS_flow_10m"];
        c.use_flow = false;
        Metrics m = runScenario(c);
        std::printf("  без GPS і без потоку: знесло на %.0f м\n", m.horiz_drift);
        CHECK(m.horiz_drift > 20, "без жодних датчиків позиції мало б знести");
    }
}

static void testBattery() {
    SECTION("Батарея Li-ion 6S 18 А·год: розряд пропорційно навантаженню, повернення додому, посадка");
    auto lib = builtinScenarios();
    for (const char* name : {"battery_hover_full", "battery_active_payload"}) {
        auto c = lib[name];
        float t_warn = -1, t_rtl = -1, t_land = -1, soc_land = 0, a_sum = 0, v_min = 99;
        long n = 0;
        c.on_step = [&](float t, World& w) {
            const auto& tel = w.fc.telemetry();
            if (tel.batt_state >= 1 && t_warn < 0) t_warn = t;
            if ((tel.rtl || tel.auto_landing) && t_rtl < 0) t_rtl = t;
            if (tel.landed && t_land < 0) { t_land = t; soc_land = w.quad.soc; }
            if (tel.armed) { a_sum += w.quad.amps; v_min = std::min(v_min, w.quad.volts); ++n; }
        };
        Metrics m = runScenario(c);
        std::printf("  %-24s 30%% на %.1f хв, додому/посадка на %.1f хв, сів на %.1f хв із %.0f%%, сер. струм %.1f А, мін. %.1f В\n",
                    name, t_warn / 60, t_rtl / 60, t_land / 60, soc_land * 100, n ? a_sum / n : 0, v_min);
        CHECK(!m.crashed && !m.crash_detected, "%s: аварія", name);
        CHECK(t_land > 0 && soc_land > 0.12f, "%s: не сів вчасно (заряд при посадці %.0f%%)", name, soc_land * 100);
    }
    // час висіння має збігатися з розрахунком (~30+ хв до 20%)
    {
        auto c = lib["battery_hover_full"];
        float t_rtl = -1;
        c.on_step = [&](float t, World& w) { if ((w.fc.telemetry().rtl || w.fc.telemetry().auto_landing) && t_rtl < 0) t_rtl = t; };
        runScenario(c);
        CHECK(t_rtl > 28 * 60, "замало висить: %.1f хв до 20%%", t_rtl / 60);
    }
    {  // повернення додому
        auto c = lib["battery_rtl"];
        float max_away = 0, t_rtl = -1;
        Vec3 land_pos;
        bool landed = false;
        c.on_step = [&](float t, World& w) {
            max_away = std::max(max_away, std::hypot(w.quad.s.pos.x, w.quad.s.pos.y));
            if (w.fc.telemetry().rtl && t_rtl < 0) t_rtl = t;
            if (w.fc.telemetry().landed && !landed) { landed = true; land_pos = w.quad.s.pos; }
        };
        Metrics m = runScenario(c);
        float dist = std::hypot(land_pos.x, land_pos.y);
        std::printf("  повернення додому: відлетів на %.0f м, на 20%% (%.0f с) повернувся і сів за %.1f м від старту\n", max_away, t_rtl, dist);
        CHECK(!m.crashed && !m.crash_detected, "аварія при поверненні");
        CHECK(t_rtl > 0 && landed, "не повернувся/не сів");
        CHECK(dist < 3.5f, "сів далеко від дому: %.1f м", dist);
    }
}

static void testEco() {
    SECTION("ЕКО-патрулювання: у штиль коло економить енергію, у вітер дрон сам лишається висіти");
    for (float wind : {0.0f, 6.0f}) {
        float p_hover = 0, p_eco = 0;
        for (int eco = 0; eco < 2; ++eco) {
            ScenarioConfig c;
            c.mode = fc::Mode::PosHold;
            c.duration = 240;
            c.target_alt = 30;
            c.metrics_from = 1e9f;
            c.wind.mean = {wind, 0, 0};
            c.wind.gust_sigma = wind > 0 ? 1.5f : 0;
            c.setpoint = [eco](float t, fc::Setpoint& sp) { sp.altitude = 30; sp.eco_orbit = eco && t > 15; };
            double w = 0;
            long n = 0;
            c.on_step = [&](float t, World& wd) { if (t > 60) { w += wd.quad.watts; ++n; } };
            Metrics m = runScenario(c);
            CHECK(!m.crashed && !m.crash_detected, "аварія");
            (eco ? p_eco : p_hover) = float(w / n);
        }
        std::printf("  вітер %.0f м/с: висіння %.0f Вт, ЕКО %.0f Вт (%+.1f%%)\n", wind, p_hover, p_eco, (p_eco / p_hover - 1) * 100);
        // (з профільною потужністю лопатей, яка в русі не зникає, виграш кола ~5%, а не 7%, як у простішій моделі)
        if (wind == 0) CHECK(p_eco < 0.97f * p_hover, "у штиль ЕКО мало б економити: %.0f vs %.0f", p_eco, p_hover);
        else CHECK(p_eco < 1.02f * p_hover, "у вітер ЕКО не повинен витрачати більше: %.0f vs %.0f", p_eco, p_hover);
    }
}

static void testSport() {
    SECTION("Спорт-режим: максимальна швидкість і гальмування");
    for (float payload : {0.0f, 1.5f}) {
        ScenarioConfig c;
        c.mode = fc::Mode::PosHold;
        c.duration = 40;
        c.wind.gust_sigma = 1.0f;
        if (payload > 0) attachPayload(c.quad, payload, {0, 0.06f, -0.15f});
        c.setpoint = [](float t, fc::Setpoint& sp) { sp.altitude = 10; sp.sport = true; sp.vel_fwd = (t > 10 && t < 26) ? 18.0f : 0; };
        float vmax = 0, alt_min = 1e9f, stop = 0;
        Vec3 rel_p;
        bool rel = false;
        c.on_step = [&](float t, World& w) {
            Vec3 v = w.quad.s.vel;
            if (t > 10 && t < 26) { vmax = std::max(vmax, std::hypot(v.x, v.y)); alt_min = std::min(alt_min, w.quad.s.pos.z); }
            if (t >= 26 && !rel) { rel = true; rel_p = w.quad.s.pos; }
            if (rel) stop = std::max(stop, std::hypot(w.quad.s.pos.x - rel_p.x, w.quad.s.pos.y - rel_p.y));
        };
        c.metrics_from = 1e9f;
        Metrics m = runScenario(c);
        std::printf("  вантаж %.1f кг: макс. швидкість %.1f м/с (%.0f км/год), мін. висота %.1f м, гальмівний шлях %.1f м\n",
                    payload, vmax, vmax * 3.6f, alt_min, stop);
        CHECK(!m.crashed && !m.crash_detected, "аварія на швидкості");
        CHECK(vmax > 12.0f, "замала швидкість %.1f", vmax);
        CHECK(alt_min > 7.0f, "просідає на швидкості до %.1f м", alt_min);
        CHECK(stop < 25.0f, "довгий гальмівний шлях %.1f м", stop);
    }
}

// Утримання точки у вітрі заданий час; повертає середню потужність і найбільше відхилення
struct HoldResult { float watts, max_h, rotor_w; bool crashed; fc::Telemetry tel; };
static HoldResult holdInWind(float wind, float gust, bool soft, float alt = 10, float T = 90, unsigned seed = 3,
                             std::function<void(float, World&)> extra = nullptr) {
    ScenarioConfig c;
    c.mode = fc::Mode::PosHold; c.duration = T; c.target_alt = alt; c.metrics_from = 1e9f; c.seed = seed;
    c.wind.mean = {wind * 0.8f, wind * 0.6f, 0}; c.wind.gust_sigma = gust; c.wind.shear_exp = 0.14f;
    c.setpoint = [alt, soft](float, fc::Setpoint& sp) { sp.altitude = alt; sp.soft_hold = soft; };
    HoldResult r{0, 0, 0, false, {}};
    double wsum = 0, rsum = 0; long n = 0;
    c.on_step = [&](float t, World& w) {
        if (extra) extra(t, w);
        if (t < 25) return;
        wsum += w.quad.watts; rsum += w.quad.rotor_w; ++n;
        r.max_h = std::max(r.max_h, float(std::hypot(w.quad.s.pos.x, w.quad.s.pos.y)));
        r.tel = w.fc.telemetry();
    };
    Metrics m = runScenario(c);
    r.watts = float(wsum / n); r.rotor_w = float(rsum / n);
    r.crashed = m.crashed || m.crash_detected;
    return r;
}

static void testEnergy() {
    SECTION("Енергія: м'яке утримання у пориви, оцінка вітру, економна швидкість повернення, живлення електроніки");
    // 1) у сильні пориви м'яке утримання економить енергію, але точку все одно тримає
    {
        // три різні пориви (seed), медіана відходу й середня потужність: один порив вирішує надто багато
        auto med3 = [](bool sf) {
            HoldResult r[3] = {holdInWind(10, 3, sf, 10, 90, 3), holdInWind(10, 3, sf, 10, 90, 4), holdInWind(10, 3, sf, 10, 90, 5)};
            float h[3] = {r[0].max_h, r[1].max_h, r[2].max_h};
            std::sort(h, h + 3);
            HoldResult o = r[0];
            o.max_h = h[1]; o.watts = (r[0].watts + r[1].watts + r[2].watts) / 3; o.rotor_w = (r[0].rotor_w + r[1].rotor_w + r[2].rotor_w) / 3;
            o.crashed = r[0].crashed || r[1].crashed || r[2].crashed;
            return o;
        };
        HoldResult hard = med3(false), soft = med3(true);
        std::printf("  вітер 10 м/с, пориви 3: точно %.0f Вт (смикання %.0f), м'яко %.0f Вт (смикання %.0f) = %+.1f%%, відхилення %.1f -> %.1f м\n",
                    hard.watts, hard.rotor_w, soft.watts, soft.rotor_w, (soft.watts / hard.watts - 1) * 100, hard.max_h, soft.max_h);
        CHECK(!hard.crashed && !soft.crashed, "аварія у вітрі");
        // висновок замірів: при обмеженні відходу ±2 м м'якість майже нічого не економить, тому за замовчуванням вона вимкнена
        // (після виправлення обліку енергії роторів «смикання» майже не коштує — м'якість не дає нічого, ±3% — шум)
        CHECK(soft.watts < 1.03f * hard.watts, "м'яке утримання не повинно витрачати помітно більше: %.0f vs %.0f", soft.watts, hard.watts);
        CHECK(soft.max_h < 5.0f, "м'яко — але не далі 5 м: %.1f м", soft.max_h);
        CHECK(hard.max_h < 3.0f, "точне утримання у вітер 10 м/с з поривами: %.1f м", hard.max_h);
        HoldResult calm = holdInWind(0, 0, true);
        CHECK(calm.tel.softness < 0.1f, "у штиль утримання має бути точним (м'якість %.2f)", calm.tel.softness);
        CHECK(calm.max_h < 1.5f, "у штиль тримає точку погано: %.1f м", calm.max_h);
    }
    // 2) оцінка вітру з акселерометра
    for (float wnd : {3.0f, 7.0f, 11.0f}) {
        HoldResult r = holdInWind(wnd, 0, true, 10, 50);
        float est = std::hypot(r.tel.wind_est.x, r.tel.wind_est.y);
        std::printf("  вітер %.0f м/с -> контролер оцінив %.1f м/с\n", wnd, est);
        CHECK(std::fabs(est - wnd) < 0.15f * wnd + 0.3f, "оцінка вітру %.1f замість %.0f", est, wnd);
    }
    // 3) повернення додому: за вітром — швидше, проти — повільніше
    for (float wx : {-5.0f, 5.0f}) {
        ScenarioConfig c;
        c.mode = fc::Mode::PosHold; c.duration = 120; c.target_alt = 25; c.metrics_from = 1e9f;
        c.battery.soc0 = 0.225f;   // 20% настає вже далеко від дому (оцінка заряду з напруги має похибку ~1%)
        c.wind.mean = {wx, 0, 0}; c.wind.gust_sigma = 0.8f;
        // дрон відлітає на схід (+X) і там його застає 20%: повертається на захід, тобто wx<0 — вітер у спину
        c.setpoint = [](float t, fc::Setpoint& sp) { sp.altitude = 25; sp.speed_limit = 10; sp.vel_fwd = (t > 6 && t < 16) ? 10.0f : 0.0f; };
        float vmax = 0; bool landed = false; float land_d = 0; double vs = 0; long vn = 0; float t_rtl = -1;
        c.on_step = [&](float t, World& w) {
            if (w.fc.telemetry().rtl && t_rtl < 0) t_rtl = t;
            // крейсерська швидкість повернення: після розгону, поки ще далеко від дому
            if (w.fc.telemetry().rtl && t > t_rtl + 2.5f && std::hypot(w.quad.s.pos.x, w.quad.s.pos.y) > 15) {
                vs += std::hypot(w.quad.s.vel.x, w.quad.s.vel.y); ++vn; vmax = float(vs / vn);
            }
            if (w.fc.telemetry().landed && !landed) { landed = true; land_d = std::hypot(w.quad.s.pos.x, w.quad.s.pos.y); }
        };
        Metrics m = runScenario(c);
        std::printf("  повернення, вітер %s: швидкість %.1f м/с, сів за %.1f м від дому\n", wx < 0 ? "у спину" : "у лоб", vmax, land_d);
        CHECK(!m.crashed && !m.crash_detected && landed, "не повернувся");
        CHECK(land_d < 3.5f, "сів далеко: %.1f м", land_d);
        if (wx < 0) CHECK(vmax > 11.5f, "за вітром мало б летіти швидше: %.1f", vmax);
        else CHECK(vmax < 9.5f && vmax > 5.0f, "проти вітру — помірно: %.1f", vmax);
    }
    // 4) електроніка: режими комп'ютера/камер і автоматичне вимкнення сенсорів на висоті
    {
        fc::PowerState s;
        float full = fc::avionicsWatts(s);
        s.payload = fc::PayloadMode::Eco; s.on[fc::PD_CAM_PTZ] = false;
        float eco = fc::avionicsWatts(s);
        s.payload = fc::PayloadMode::Sleep; s.on[fc::PD_CAM_WIDE] = false;
        float sleep = fc::avionicsWatts(s);
        std::printf("  електроніка: повний %.1f Вт, економ %.1f Вт, сон %.1f Вт\n", full, eco, sleep);
        CHECK(full > 25 && full < 40 && eco < full - 10 && sleep < eco - 5, "розрахунок споживання електроніки");
    }
    {
        bool flow_slept = false, range_slept = false, woke = false;
        float drift_after_jam = 0;
        Vec3 p_jam;
        HoldResult r = holdInWind(4, 1, true, 100, 140, 5, [&](float t, World& w) {
            if (t > 40 && t < 41) w.gps.fault = {0, 0};
            const auto& tel = w.fc.telemetry();
            if (t > 60 && t < 90) { flow_slept |= !tel.power.on[fc::PD_FLOW]; range_slept |= !tel.power.on[fc::PD_RANGE]; }
            if (t >= 95 && w.gps.fault.t1 < 1) { w.gps.fault = {95, 1e9f}; p_jam = w.quad.s.pos; }   // глушимо GPS на висоті
            if (t > 97) { woke |= tel.flow_ok; drift_after_jam = std::max(drift_after_jam, float(std::hypot(w.quad.s.pos.x - p_jam.x, w.quad.s.pos.y - p_jam.y))); }
        });
        std::printf("  на 100 м з GPS: потік %s, далекомір %s; GPS заглушили -> потік %s, знесло %.1f м за 45 с\n",
                    flow_slept ? "спить" : "працює", range_slept ? "спить" : "працює", woke ? "прокинувся" : "НЕ прокинувся", drift_after_jam);
        CHECK(!r.crashed, "аварія");
        CHECK(flow_slept && range_slept, "на висоті з GPS потік і далекомір мали б вимкнутись");
        CHECK(woke, "без GPS потік мав увімкнутись");
        CHECK(drift_after_jam < 8.0f, "без GPS знесло %.1f м", drift_after_jam);
    }
}

static void testWeather() {
    SECTION("Випадкова погода: змінюється сама, дрон витримує 15 хв");
    {
        sim::Weather wx(7);
        sim::WindParams wp;
        int seen[sim::Weather::kKinds] = {0};
        float vmax = 0;
        for (int i = 0; i < 4 * 3600 * 100; ++i) {  // 4 год з кроком 10 мс
            wx.step(0.01f, wp);
            seen[wx.kind()] = 1;
            vmax = std::max(vmax, wp.mean.norm());
        }
        int kinds = 0;
        for (int k : seen) kinds += k;
        std::printf("  за 4 години: %d з 5 видів погоди, найсильніший вітер %.1f м/с (на 10 м)\n", kinds, vmax);
        CHECK(kinds == 5, "мали б трапитись усі види погоди");
        CHECK(vmax < 13.0f, "надто сильний вітер %.1f", vmax);
    }
    for (unsigned seed : {11u, 12u, 13u}) {
        sim::Weather wx(seed);
        float max_h = 0;
        HoldResult r = holdInWind(0, 0, false, 30, 15 * 60, seed, [&](float t, World& w) {
            wx.step(0.001f, w.quad.wind);
            if (t > 25) max_h = std::max(max_h, float(std::hypot(w.quad.s.pos.x, w.quad.s.pos.y)));
        });
        std::printf("  погода #%u: 15 хв на 30 м, найбільше відхилення %.1f м, середня потужність %.0f Вт\n", seed, max_h, r.watts);
        CHECK(!r.crashed, "аварія у випадковій погоді");
        CHECK(max_h < 3.5f, "знесло %.1f м", max_h);
    }
}

// Утримання точки / політ з планером; повертає середню потужність, підйомну силу, похибку курсу від вітру
struct AirResult { float watts, lift, max_h, yaw_err; bool crashed; };
static AirResult flyAirframe(const fc::AirframeConfig& a, float wind, float cruise, float T = 100) {
    ScenarioConfig c;
    c.mode = fc::Mode::PosHold; c.duration = T; c.target_alt = 30; c.metrics_from = 1e9f; c.seed = 4;
    applyAirframe(c.quad, a, &c.fc.vehicle);
    const float d = 120 * fc::kDeg2Rad;   // вітер дме збоку-ззаду відносно початкового курсу
    c.wind.mean = {wind * std::cos(d), wind * std::sin(d), 0};
    c.wind.gust_sigma = wind > 0 ? 0.15f + 0.2f * wind : 0;
    c.wind.shear_exp = 0.14f;
    c.setpoint = [cruise](float t, fc::Setpoint& sp) { sp.altitude = 30; sp.speed_limit = 18; sp.vel_fwd = (cruise > 0 && t > 15) ? cruise : 0; };
    AirResult r{0, 0, 0, 0, false};
    double ws = 0, ls = 0, ye = 0; long n = 0;
    c.on_step = [&](float t, World& w) {
        if (t < 45) return;
        ws += w.quad.watts; ls += w.quad.aero.lift; ++n;
        if (cruise == 0) r.max_h = std::max(r.max_h, float(std::hypot(w.quad.s.pos.x, w.quad.s.pos.y)));
        const float from = std::atan2(-w.quad.wind.mean.y, -w.quad.wind.mean.x);
        ye += std::fabs(std::remainder(w.quad.s.q.euler().z - from, 2 * fc::kPi)) * fc::kRad2Deg;
    };
    Metrics m = runScenario(c);
    r.watts = float(ws / n); r.lift = float(ls / n); r.yaw_err = float(ye / n); r.crashed = m.crashed || m.crash_detected;
    return r;
}

static void testAirframe() {
    SECTION("Планер: обтічний корпус, крило, кіль, тяговий гвинт");
    {   // модель крила: до зриву — лінійна, після — як пластина
        fc::WingConfig w; w.enabled = true;
        float cl, cd;
        fc::wingCoeffs(w, 4 * fc::kDeg2Rad, cl, cd);
        CHECK(std::fabs(cl - w.clAlpha() * 4 * fc::kDeg2Rad) < 0.03f, "лінійна ділянка: cl=%.3f", cl);
        fc::wingCoeffs(w, 90 * fc::kDeg2Rad, cl, cd);
        CHECK(std::fabs(cl) < 0.05f && cd > 1.2f, "крило поперек потоку: cl=%.2f cd=%.2f", cl, cd);
        fc::wingCoeffs(w, 25 * fc::kDeg2Rad, cl, cd);
        CHECK(cl < w.cl_max, "після зриву підйомна сила падає: cl=%.2f", cl);
    }
    fc::AirframeConfig quad, fair, wing, wingp;
    fair.body_cd = 0.05f;
    wing = fair; wing.wing.enabled = true; wing.fin.area = 0.04f;
    wingp = wing; wingp.pusher.enabled = true; wingp.pusher.max_thrust = 45; wingp.pusher.eff = 0.65f;
    wingp.wing.area = 0.6f; wingp.wing.span = 1.8f; wingp.wing.incidence_deg = 6; wingp.wing.mass_per_m2 = 0.8f;
    AirResult q8 = flyAirframe(quad, 8, 0), f8 = flyAirframe(fair, 8, 0), w8 = flyAirframe(wing, 8, 0);
    std::printf("  висіння у вітер 8 м/с: квадро %.0f Вт, обтічний %.0f Вт (%+.0f%%), з крилом %.0f Вт (%+.0f%%, підйомна %.1f Н, ніс від вітру %.0f°)\n",
                q8.watts, f8.watts, (f8.watts / q8.watts - 1) * 100, w8.watts, (w8.watts / q8.watts - 1) * 100, w8.lift, w8.yaw_err);
    CHECK(!q8.crashed && !f8.crashed && !w8.crashed, "аварія");
    CHECK(f8.watts < 0.92f * q8.watts, "обтічний корпус мав би економити у вітер: %.0f vs %.0f", f8.watts, q8.watts);
    CHECK(w8.yaw_err < 25, "з крилом дрон мав би повернутись носом проти вітру (похибка %.0f°)", w8.yaw_err);
    CHECK(w8.max_h < 3.0f, "з крилом точку тримає погано: %.1f м", w8.max_h);
    AirResult fc12 = flyAirframe(fair, 0, 12), wc12 = flyAirframe(wingp, 0, 12);
    std::printf("  політ 12 м/с: обтічний квадро %.0f Вт, крило + тяговий гвинт %.0f Вт (підйомна %.1f Н)\n", fc12.watts, wc12.watts, wc12.lift);
    CHECK(!fc12.crashed && !wc12.crashed, "аварія в польоті");
    CHECK(wc12.lift > 15, "крило в польоті мало б тримати помітну частку ваги: %.1f Н", wc12.lift);
}

static void testMonteCarlo(int runs) {
    SECTION("Монте-Карло: випадкові вітер / дисбаланс / маса / сенсори / мотори / курс (половина — з GPS)");
    std::mt19937 rng(12345);
    prng::uniform<float> U(0, 1);
    auto u = [&](float a, float b) { return a + (b - a) * U(rng); };
    int ok = 0;
    float worst_tilt = 0;
    for (int i = 0; i < runs; ++i) {
        ScenarioConfig c;
        c.seed = 1000 + i;
        c.duration = 16;
        float wa = u(0, 2 * fc::kPi), ws = u(0, 8);
        c.wind.mean = {ws * std::cos(wa), ws * std::sin(wa), 0};
        c.wind.gust_sigma = u(0, 3);
        c.quad.com_offset = {u(-0.03f, 0.03f), u(-0.03f, 0.03f), u(-0.03f, 0.03f)};
        c.quad.mass = u(4.0f, 5.1f);                       // вага ±12% від того, що думає контролер
        c.quad.inertia = Vec3{u(0.10f, 0.15f), u(0.10f, 0.15f), u(0.18f, 0.28f)};
        float payload = U(rng) < 0.5f ? u(0.0f, 1.5f) : 0.0f;  // половина польотів — з вантажем до 1.5 кг
        if (payload > 0) attachPayload(c.quad, payload, {u(-0.05f, 0.05f), u(-0.05f, 0.05f), -0.15f});
        c.quad.thrust_expo = u(0, 0.5f);
        c.quad.motor_tau = u(0.02f, 0.05f);
        for (float& e : c.quad.motor_eff) e = u(0.85f, 1.0f);
        c.imu.gyro_bias_dps = {u(-3, 3), u(-3, 3), u(-3, 3)};
        c.imu.accel_bias_g = {u(-0.05f, 0.05f), u(-0.05f, 0.05f), u(-0.05f, 0.05f)};
        c.imu.vibration_g = u(0.02f, 0.3f);
        c.analog_imu = U(rng) < 0.3f;
        c.target_alt = u(1.5f, 5.0f);
        bool gps_mode = (i % 2 == 1);
        if (gps_mode) c.mode = fc::Mode::PosHold;
        c.initial_yaw = u(-3.1f, 3.1f);
        c.metrics_from = c.arm_time + c.target_alt / 1.5f + 3.0f;  // після набору висоти
        c.duration = c.metrics_from + 10;
        Metrics m = runScenario(c);
        bool pass = !m.crashed && !m.crash_detected && m.max_alt_err < 0.8f &&
                    (gps_mode ? m.max_horiz < 5.0f : m.max_tilt_err_deg < 15);
        ok += pass;
        worst_tilt = std::max(worst_tilt, m.max_tilt_err_deg);
        if (!pass) {
            std::printf("  ✗ run %d: wind=%.1f м/с gust=%.1f mass=%.2f com=(%.3f %.3f %.3f) analog=%d -> ",
                        i, ws, c.wind.gust_sigma, c.quad.mass, c.quad.com_offset.x, c.quad.com_offset.y,
                        c.quad.com_offset.z, c.analog_imu);
            printMetrics("", m);
        }
    }
    float rate = 100.0f * ok / runs;
    std::printf("  Успішних: %d/%d (%.1f%%), найбільший нахил: %.1f°\n", ok, runs, rate, worst_tilt);
    CHECK(rate >= 95.0f, "успішність %.1f%% < 95%%", rate);
}


// ---- Реалізм фізики: ефект землі, вихровий режим, ковзання при посадці, мороз ----
static void testPhysicsRealism() {
    std::printf("\n== Реалізм фізики: ефект землі, вихровий режим, приземлення, мороз ==\n");
    const float hov[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    auto prep = [&](sim::QuadSim& q, float z, Vec3 v) {
        q.on_ground = false; q.s.pos = Vec3{0, 0, z}; q.s.vel = v;
        for (int i = 0; i < 4; ++i) q.thrust[i] = q.p.mass * fc::kGravity / 4;
    };
    auto fz = [&](bool ge, bool vrs, float z, Vec3 v) {
        sim::QuadSim q(1); q.p.ground_effect = ge; q.p.vortex_ring = vrs;
        prep(q, z, v);
        for (int k = 0; k < 20; ++k) { q.step(hov, 0.001f); prep(q, z, v); }
        return q.specific_force_body.z;
    };
    const float lo = fz(true, false, 0.02f, Vec3{}), lo0 = fz(false, false, 0.02f, Vec3{}), hi = fz(true, false, 6.0f, Vec3{});
    std::printf("  ефект землі: тяга біля землі (диски на 0.17 м) +%.1f%%, на 6 м +%.2f%%\n", 100 * (lo / lo0 - 1), 100 * (hi / lo0 - 1));
    CHECK(lo > 1.04f * lo0 && lo < 1.20f * lo0, "біля землі тяга має рости на 4–20%%");
    CHECK(hi < 1.01f * lo0, "на висоті ефекту землі немає");
    const float vrs = fz(false, true, 100, Vec3{0, 0, -5.0f}), vr0 = fz(false, false, 100, Vec3{0, 0, -5.0f});
    const float vrs_f = fz(false, true, 100, Vec3{8.0f, 0, -5.0f}), vrs_f0 = fz(false, false, 100, Vec3{8.0f, 0, -5.0f});
    const float vrs_slow = fz(false, true, 100, Vec3{0, 0, -1.0f}), vrs_slow0 = fz(false, false, 100, Vec3{0, 0, -1.0f});
    std::printf("  вихровий режим: спуск 5 м/с — тяга %.0f%% від звичайної; з бічним 8 м/с — %.0f%%; спуск 1 м/с — %.0f%%\n",
                100 * vrs / vr0, 100 * vrs_f / vrs_f0, 100 * vrs_slow / vrs_slow0);
    CHECK(vrs < 0.85f * vr0, "швидкий спуск у власному обдуві втрачає тягу");
    CHECK(vrs_f > 0.97f * vrs_f0 && vrs_slow > 0.97f * vrs_slow0, "з бічним потоком і на повільному спуску втрат немає");
    {   // приземлення з горизонтальною швидкістю: ковзає й зупиняється; надто швидко — перекидання
        auto land = [&](float hs, float* slide, bool* crashed) {
            sim::QuadSim q(1); const float off[4] = {0, 0, 0, 0};
            q.on_ground = false; q.s.pos = Vec3{0, 0, 0.05f}; q.s.vel = Vec3{hs, 0, -1.0f};
            for (int k = 0; k < 6000; ++k) q.step(off, 0.001f);
            *slide = q.s.pos.x; *crashed = q.crashed;
        };
        float d3 = 0, d9 = 0; bool c3 = false, c9 = false;
        land(3.0f, &d3, &c3); land(9.0f, &d9, &c9);
        std::printf("  приземлення з 3 м/с: ковзає %.2f м (теорія v²/2μg = %.2f); з 9 м/с: %s\n", d3, 9 / (2 * 0.5f * fc::kGravity),
                    c9 ? "перекинувся" : "ціле");
        CHECK(!c3 && d3 > 0.6f * 9 / (2 * 0.5f * fc::kGravity) && d3 < 1.3f * 9 / (2 * 0.5f * fc::kGravity), "ковзання за тертям: %.2f м", d3);
        CHECK(c9, "9 м/с по горизонталі — аварія");
    }
    {   // мороз: просідання напруги під тим самим навантаженням більше
        auto sag = [&](float tc) {
            sim::QuadSim q(1); q.bat.temp_c = tc; prep(q, 20, Vec3{});
            for (int k = 0; k < 2000; ++k) { q.step(hov, 0.001f); prep(q, 20, Vec3{}); }
            return q.volts;
        };
        const float v25 = sag(25), vm10 = sag(-10);
        std::printf("  батарея під навантаженням: 25 °C — %.2f В, −10 °C — %.2f В\n", v25, vm10);
        CHECK(v25 - vm10 > 0.4f, "на морозі напруга просідає помітно сильніше");
    }
}

// ---- Дані апарата: aircraft/tiltrotor.ini ----
static AircraftFile loadQuadPlane() {
    try { return AircraftFile::load("aircraft/tiltrotor.ini"); }
    catch (const std::exception&) { return AircraftFile::load("../aircraft/tiltrotor.ini"); }
}

static bool throwsParse(const std::string& text) {
    std::istringstream in(text);
    try { AircraftFile::parse(in, "тест"); } catch (const std::runtime_error&) { return true; }
    return false;
}

static void testAircraftData() {
    SECTION("Дані апарата: конвертоплан (aircraft/tiltrotor.ini)");
    const AircraftFile a = loadQuadPlane();
    const AircraftDerived d = deriveAircraft(a);
    for (const std::string& p : d.problems) std::printf("  порушено: %s\n", p.c_str());
    CHECK(d.problems.empty(), "перевірка файлу апарата не пройдена: %zu порушень", d.problems.size());
    CHECK(std::fabs(d.mass_total - a.num("targets.takeoff_mass_kg")) < 0.005 && d.mass_total > 2.0 && d.mass_total < 2.6, "маса %.3f кг", d.mass_total);
    CHECK(std::fabs(d.cg_x) < 0.002 && std::fabs(d.cg_y) < 1e-6, "ЦМ x=%.4f y=%.5f", d.cg_x, d.cg_y);
    CHECK(d.inertia.xx > 0 && d.inertia.yy > 0 && d.inertia.zz > 0 && d.inertia.xx + d.inertia.yy >= d.inertia.zz &&
          d.inertia.yy + d.inertia.zz >= d.inertia.xx && d.inertia.xx + d.inertia.zz >= d.inertia.yy, "тензор інерції фізично неможливий");
    CHECK(std::fabs(d.inertia.xy) < 1e-9 && std::fabs(d.inertia.yz) < 1e-9, "апарат симетричний: Ixy=%g Iyz=%g", d.inertia.xy, d.inertia.yz);
    std::printf("  маса %.2f кг, ЦМ x %+.1f мм, I = %.3f / %.3f / %.3f кг·м², V_stall %.2f м/с, запас стійкості %.1f%% САХ, V_H %.2f, V_V %.3f\n",
                d.mass_total, 1000 * d.cg_x, d.inertia.xx, d.inertia.yy, d.inertia.zz, d.v_stall, 100 * d.static_margin, d.v_h, d.v_v);
    CHECK(std::fabs(d.v_stall - 7.9) < 0.4, "звалювання %.2f м/с", d.v_stall);   // два крила: основне на CLmax несе ≈ 87% ваги
    CHECK(d.static_margin > 0.05 && d.static_margin < 0.15, "запас стійкості %.3f", d.static_margin);
    // поздовжню стійкість тепер дає переважно заднє крило (V-хвіст менший): сумарний горизонтальний об'єм заднього крила й V-хвоста
    const double vh_total = (d.rw_area * (d.cg_x - d.rw_ac_x) + d.tail_area * d.tail_arm) / (d.wing_area * a.num("geometry.wing_chord_m"));
    CHECK(vh_total > 0.5 && d.v_v > 0.02, "об'єми: горизонтальний (заднє крило + V-хвіст) %.3f, вертикальний V_V %.4f", vh_total, d.v_v);
    CHECK(d.rw_lift_frac > 0.05 && d.rw_lift_frac < 0.3, "заднє крило несе %.0f%% ваги", 100 * d.rw_lift_frac);
    // жодного перетину: обід кожного гвинта в усьому діапазоні нахилу (−25…92°) ≥ 2 см від конструкції, землі й інших гвинтів
    std::printf("  найменший зазор гвинта %.1f мм: %s\n", 1000 * d.clear_min, d.clear_what.c_str());
    CHECK(d.clear_min >= 0.02, "зазор гвинта %.1f мм (%s)", 1000 * d.clear_min, d.clear_what.c_str());
    {   // перевірка справді ловить перетин: задня гондола зсунута на 10 см назад — диск заходить на заднє крило й ніжку
        AircraftFile c = a;
        c.values["vtol_motor.rear_x_m"] = {"-0.45"};
        const AircraftDerived f = deriveAircraft(c);
        CHECK(f.clear_min < 0.005, "зсунута гондола: зазор %.1f мм (%s)", 1000 * f.clear_min, f.clear_what.c_str());
    }
    CHECK(d.rear_prop_clear >= 0.020 - 1e-9 && d.front_prop_clear > 0.05, "зазори гвинтів: задні між собою %.1f мм, передні до крила %.1f мм", 1000 * d.rear_prop_clear, 1000 * d.front_prop_clear);
    CHECK(d.hover_elec_w > 200 && d.hover_elec_w < 450, "висіння %.0f Вт", d.hover_elec_w);
    // гондоли: передні й задні на однакових плечах від ЦМ — у висінні несуть порівну (рівні оберти, однаковий запас тяги)
    CHECK(std::fabs(a.num("vtol_motor.front_x_m") + a.num("vtol_motor.rear_x_m")) < 0.01, "плечі гондол: перед %.3f, зад %.3f м", a.num("vtol_motor.front_x_m"), a.num("vtol_motor.rear_x_m"));
    // помилки у файлі мають бути помітні, а не мовчки проковтнуті
    CHECK(throwsParse("[geometry]\nwing_span_m = abc\n[mass]\nx = 1 0 0 0 0 0\n"), "неправильний рядок у [mass] має кидати помилку");
    CHECK(throwsParse("[geometry]\nwing_span_m 1.6\n"), "рядок без «=» має кидати помилку");
    CHECK(throwsParse("[mass\nx = 1 0 0 0 0 0 0\n"), "розділ без «]» має кидати помилку");
    CHECK(throwsParse("[mass]\nx = 1 0 0 0 0 0 nan\n"), "NaN у масі має кидати помилку");
    bool missing = false;
    try { a.num("geometry.немає_такого"); } catch (const std::runtime_error&) { missing = true; }
    CHECK(missing, "відсутній параметр має кидати помилку");
    {   // порушення умов ловляться перевіркою: зайва маса й зсунутий ЦМ
        AircraftFile c = a;
        c.items[0].m += 0.2; c.items[0].x += 0.1;
        const AircraftDerived f = deriveAircraft(c);
        CHECK(f.problems.size() >= 2, "зламані маса й ЦМ мають давати порушення, а дало %zu", f.problems.size());
    }
}

// ---- Таблиці профілю: aircraft/aero_tables.json ----
static AeroTables loadAeroTables() {
    try { return AeroTables::load("aircraft/aero_tables.json"); }
    catch (const std::exception&) { return AeroTables::load("../aircraft/aero_tables.json"); }
}

static void testAeroTables() {
    SECTION("Таблиці профілю (aircraft/aero_tables.json, tools/aero_gen.py)");
    const double kPi = 3.14159265358979323846, d2r = kPi / 180;
    const AeroTables T = loadAeroTables();
    const AirfoilTable& cy = T.get("clarky_re3e5");
    const AirfoilTable& sym = T.get("naca0010_re1p5e5");
    // нахил, нуль підйомної сили, максимум
    const double slope = (cy.eval(6 * d2r).cl - cy.eval(-2 * d2r).cl) / (8 * d2r);
    std::printf("  Clark Y, Re 3e5: CLα %.2f 1/рад, α0 %.1f°, CLmax %.2f при %.1f°; CDmax %.2f\n", slope, cy.alpha_zero / d2r, cy.cl_max,
                cy.alpha_stall_pos / d2r, cy.cd_max);
    CHECK(slope > 5.6 && slope < 6.4, "CLα профілю %.2f", slope);
    CHECK(std::fabs(cy.eval(cy.alpha_zero).cl) < 0.03, "cl при α0 = %.3f", cy.eval(cy.alpha_zero).cl);
    CHECK(cy.cl_max > 1.25 && cy.cl_max < 1.5 && cy.alpha_stall_pos > 9 * d2r && cy.alpha_stall_pos < 14 * d2r, "CLmax %.2f при %.1f°", cy.cl_max, cy.alpha_stall_pos / d2r);
    // скінченні, плавні, фізичні значення на всьому колі
    bool finite = true; double max_dcl = 0, max_dcd = 0, min_cd = 1e9;
    for (const AirfoilTable* tb : {&cy, &sym}) {
        for (size_t i = 0; i < tb->size(); ++i) {
            const SectionCoeffs c = tb->at(i);
            finite = finite && std::isfinite(c.cl) && std::isfinite(c.cd) && std::isfinite(c.cm);
            min_cd = std::min(min_cd, c.cd);
            if (i) {
                const SectionCoeffs p = tb->at(i - 1);
                max_dcl = std::max(max_dcl, std::fabs(c.cl - p.cl)); max_dcd = std::max(max_dcd, std::fabs(c.cd - p.cd));
            }
        }
    }
    std::printf("  найбільший стрибок між сусідніми точками: Δcl %.3f, Δcd %.3f\n", max_dcl, max_dcd);
    CHECK(finite && min_cd >= 0, "є NaN/Inf або від'ємний опір (мін. cd %.4f)", min_cd);
    CHECK(max_dcl < 0.15 && max_dcd < 0.1, "таблиці недостатньо плавні: Δcl %.3f Δcd %.3f", max_dcl, max_dcd);
    // пластина поперек потоку: cl = 0, cd = CDmax; по колу — періодичність
    CHECK(std::fabs(cy.eval(kPi / 2).cl) < 0.01 && std::fabs(cy.eval(kPi / 2).cd - cy.cd_max) < 0.02, "α=90°: cl %.3f cd %.3f", cy.eval(kPi / 2).cl, cy.eval(kPi / 2).cd);
    CHECK(std::fabs(cy.eval(0.3).cl - cy.eval(0.3 + 2 * kPi).cl) < 1e-9 && std::fabs(cy.eval(kPi - 1e-4).cl - cy.eval(-kPi + 1e-4).cl) < 0.01, "α по колу має бути періодичним");
    CHECK(cy.eval(NAN).cl == 0 && cy.eval(1e9).cd >= 0, "некоректний кут не повинен давати сміття");
    // симетричний профіль: cl(−α) = −cl(α), cm(−α) = −cm(α); cd парний
    double asym = 0;
    for (double a = 1; a <= 179; a += 1) {
        const SectionCoeffs p = sym.eval(a * d2r), m = sym.eval(-a * d2r);
        asym = std::max({asym, std::fabs(p.cl + m.cl), std::fabs(p.cd - m.cd) * 0.5});
    }
    CHECK(asym < 0.05, "NACA 0010 несиметричний: %.3f", asym);
    // зрив: після кута зриву cl падає, cd росте
    CHECK(cy.eval(cy.alpha_stall_pos + 8 * d2r).cl < cy.cl_max - 0.1 && cy.eval(25 * d2r).cd > 5 * cy.eval(5 * d2r).cd, "після зриву підйомна сила має падати, а опір рости");
    // зворотний потік (рев'ю): центр тиску від задньої кромки → при α ≈ 175° момент має бути вагомий і від'ємний (ніс вниз)
    CHECK(cy.eval(175 * d2r).cm < -0.01 && cy.eval(-175 * d2r).cm > 0.01, "cm у зворотному потоці: %.3f / %.3f", cy.eval(175 * d2r).cm, cy.eval(-175 * d2r).cm);
    // помилки файлу не ковтаються
    bool bad = false;
    try { Json::parse("{\"airfoils\": {\"x\": [1, 2,}"); } catch (const std::runtime_error&) { bad = true; }
    CHECK(bad, "зламаний JSON має кидати помилку");
    bool missing = false;
    try { T.get("немає"); } catch (const std::runtime_error&) { missing = true; }
    CHECK(missing, "невідомий профіль має кидати помилку");
}

// ---- Аеродинаміка QuadPlane: смуги, оперення, скіс потоку ----
struct AeroFixture {
    AircraftFile file = loadQuadPlane();
    AeroTables tables = loadAeroTables();
    AircraftDerived d = deriveAircraft(file);
    QuadPlaneAero aero{file, tables, d};
    static constexpr double kRho = 1.225, kS = 0.448, kC = 0.28;
    // сили й моменти (тіло) при набіганні V під кутом атаки alpha_deg (рад — всередині), без обертання
    AeroResult at(double V, double alpha_deg, double beta_deg = 0, double da = 0, double de = 0, double dr = 0, Vec3d omega = {}) const {
        const double a = alpha_deg * 3.14159265358979323846 / 180, b = beta_deg * 3.14159265358979323846 / 180;
        AeroInputs in;
        in.v_body = Vec3d{V * std::cos(a) * std::cos(b), V * std::sin(b), -V * std::sin(a) * std::cos(b)};
        in.omega = omega; in.rho = kRho; in.aileron = da; in.elevator = de; in.rudder = dr;
        return aero.eval(in);
    }
    // CL, CD у вісях вітру; моменти в тілі
    static double lift(const AeroResult& r, double alpha_deg) {
        const double a = alpha_deg * 3.14159265358979323846 / 180;
        return r.force.x * std::sin(a) + r.force.z * std::cos(a);
    }
    static double drag(const AeroResult& r, double alpha_deg) {
        const double a = alpha_deg * 3.14159265358979323846 / 180;
        return -r.force.x * std::cos(a) + r.force.z * std::sin(a);
    }
};

static void testQuadPlaneAero() {
    SECTION("Аеродинаміка конвертоплана (смуги + підйомна лінія + V-хвіст)");
    const AeroFixture f;
    const double V = 15, q = 0.5 * f.kRho * V * V, W = f.d.mass_total * 9.80665, kDeg = 3.14159265358979323846 / 180;
    auto CL = [&](double a) { return AeroFixture::lift(f.at(V, a), a) / (q * f.kS); };
    const double cla = (CL(6) - CL(0)) / (6 * kDeg);
    double a_zero = -10; for (double a = -10; a < 0; a += 0.1) if (CL(a) < 0) a_zero = a;
    double cl_max = 0, a_max = 0; for (double a = 0; a <= 30; a += 0.25) if (CL(a) > cl_max) { cl_max = CL(a); a_max = a; }
    std::printf("  CLα %.2f 1/рад, α(CL=0) %.1f°, CLmax %.2f при %.1f° (звалювання при m=%.2f кг: %.1f м/с)\n", cla, a_zero, cl_max, a_max, f.d.mass_total,
                std::sqrt(2 * W / (f.kRho * f.kS * cl_max)));
    // крило 3D ≈ 4.3 (Гельмбольд/VLM) + заднє крило (S 0.16 м² у скосі потоку) ≈ 0.8 + V-хвіст ≈ 0.2 — усе віднесено до площі основного крила
    CHECK(cla > 4.8 && cla < 6.2, "CLα крила з оперенням %.2f", cla);
    CHECK(a_zero > -5.5 && a_zero < -2.5, "нульова підйомна сила при %.1f°", a_zero);
    // CLmax усього апарата на площу основного: + заднє крило (≈ 0.36 площі основного, зривається пізніше — у скосі потоку)
    // максимум може бути вже після зриву основного (≈ 17°): заднє крило в скосі потоку ще тягне — зрив «ніс униз», як у тандема
    CHECK(cl_max > 1.2 && cl_max < 1.8 && a_max > 11 && a_max < 24, "CLmax %.2f при %.1f°", cl_max, a_max);
    {   // основне крило зривається раніше за заднє: при α ≈ 18° момент тангажу пікіруючий (самовихід зі звалювання)
        const AeroResult rs = f.at(V, 18);
        std::printf("  α 18°: момент тангажу %+.2f Н·м (+ — ніс униз)\n", rs.moment.y);
        CHECK(rs.moment.y > 0, "на зриві момент має бути пікіруючим, а %.2f Н·м", rs.moment.y);
    }
    const double vs_model = std::sqrt(2 * W / (f.kRho * f.kS * cl_max));
    CHECK(vs_model > 6.6 && vs_model < 8.6, "звалювання за моделлю %.2f м/с (оцінка ≈ %.1f)", vs_model, f.d.v_stall);
    // тримування на 15 м/с: підйомна сила = вага
    double a_trim = 0; for (double a = -2; a < 14; a += 0.01) if (AeroFixture::lift(f.at(V, a), a) < W) a_trim = a;
    const AeroResult tr = f.at(V, a_trim);
    const double ld = AeroFixture::lift(tr, a_trim) / AeroFixture::drag(tr, a_trim), p_aero = AeroFixture::drag(tr, a_trim) * V;
    std::printf("  15 м/с, L = вага: α %.1f°, L/D %.1f, опір %.1f Н, аеродинамічна потужність %.0f Вт, момент тангажу %.2f Н·м\n", a_trim, ld,
                AeroFixture::drag(tr, a_trim), p_aero, tr.moment.y);
    std::printf("  частки підйомної сили: основне крило %.0f%%, заднє %.0f%%, V-хвіст %.0f%%\n", 100 * tr.wing_force.z / W, 100 * tr.rear_force.z / W, 100 * tr.stab_force.z / W);
    // легший апарат (2.27 кг замість 3.5) — менший CL на крейсері, тож і α менший, ніж у QuadPlane
    CHECK(a_trim > -1.0 && a_trim < 5, "α тримування %.1f° (до хорди крила)", a_trim);
    CHECK(ld > 6 && ld < 14, "аеродинамічна якість %.1f", ld);
    // статична стійкість: dM_y/dα > 0 (ніс угору → момент ніс вниз); запас = −Cmα/CLα
    const double dM = (f.at(V, a_trim + 1).moment.y - f.at(V, a_trim - 1).moment.y) / (2 * kDeg);
    const double dL = (AeroFixture::lift(f.at(V, a_trim + 1), a_trim + 1) - AeroFixture::lift(f.at(V, a_trim - 1), a_trim - 1)) / (2 * kDeg);
    const double sm = dM / (dL * f.kC);
    std::printf("  статичний запас з аеромоделі: %.1f%% САХ (аналітика без корпусу: %.1f%%)\n", 100 * sm, 100 * f.d.static_margin);
    // без внеску фюзеляжу й гвинта (Munk, носовий гвинт: −2…−4% САХ) — тому верхня межа вища за 15%
    CHECK(dM > 0 && sm > 0.04 && sm < 0.19, "запас стійкості %.3f", sm);
    // скіс потоку на V-хвості
    const double deps = (f.at(V, 6).tail_downwash_rad - f.at(V, 0).tail_downwash_rad) / (6 * kDeg);
    std::printf("  dε/dα на V-хвості: %.2f\n", deps);
    CHECK(deps > 0.2 && deps < 0.65, "dε/dα = %.2f", deps);
    // керування: знаки та величина
    const AeroResult pe = f.at(V, a_trim, 0, 0, 0.1), pa = f.at(V, a_trim, 0, 0.1), pr = f.at(V, a_trim, 0, 0, 0, 0.1);
    CHECK(pe.moment.y - tr.moment.y > 0.3, "elevator>0 має давати ніс вниз (+My): %.2f", pe.moment.y - tr.moment.y);
    CHECK(pa.moment.x > 0.3 && std::fabs(pa.moment.z) < 0.6 * pa.moment.x, "aileron>0 має давати правий крен (+Mx): %.2f / Mz %.2f", pa.moment.x, pa.moment.z);
    CHECK(pr.moment.z > 0.1, "rudder>0 має давати ніс ліворуч (+Mz): %.2f", pr.moment.z);
    std::printf("  керма на 0.1 рад (5.7°): Δ момент тангажу %+.2f, крену %+.2f, рискання %+.2f Н·м\n", pe.moment.y - tr.moment.y, pa.moment.x, pr.moment.z);
    // демпфування за швидкостями обертання
    CHECK(f.at(V, a_trim, 0, 0, 0, 0, {0, 0.5, 0}).moment.y - tr.moment.y < -0.1, "демпфування тангажу");
    CHECK(f.at(V, a_trim, 0, 0, 0, 0, {0.5, 0, 0}).moment.x < -0.05, "демпфування крену");
    CHECK(f.at(V, a_trim, 0, 0, 0, 0, {0, 0, 0.5}).moment.z < -0.02, "демпфування рискання");
    // ковзання: флюгерна стійкість (+Mz при v>0) і бічна сила проти руху
    const AeroResult sl = f.at(V, a_trim, 5);
    CHECK(sl.moment.z > 0.1 && sl.force.y < 0, "ковзання вліво: Mz %.2f Fy %.2f", sl.moment.z, sl.force.y);
    // дзеркальна симетрія
    const AeroResult sr = f.at(V, a_trim, -5);
    CHECK(std::fabs(sl.force.y + sr.force.y) < 1e-6 && std::fabs(sl.moment.z + sr.moment.z) < 1e-6 && std::fabs(sl.moment.x + sr.moment.x) < 1e-6 &&
          std::fabs(sl.force.z - sr.force.z) < 1e-6, "аеродинаміка несиметрична");
    // вся область кутів і швидкостей: без NaN і розумні межі
    bool finite = true; double fmax = 0;
    for (double v : {0.0, 0.5, 3.0, 8.0, 17.0, 30.0})
        for (double a = -180; a <= 180; a += 15)
            for (double b = -180; b <= 180; b += 15) {
                const AeroResult r = f.at(v, a, b, 0.2, -0.2, 0.2, {0.3, -0.4, 0.5});
                finite = finite && r.force.finite() && r.moment.finite();
                fmax = std::fmax(fmax, r.force.norm());
            }
    CHECK(finite && fmax < 700, "NaN або неправдоподібні сили: макс %.0f Н", fmax);
    CHECK(f.aero.eval(AeroInputs{Vec3d{NAN, 0, 0}, {}, 1.225}).force.norm() == 0, "NaN на вході має давати нульові сили");
    // підйомна лінія має збігатися до пост-зривного діапазону; за зривом нев'язка може бути більшою
    const double ll_ok = f.aero.liftingLineResidual(-10 * kDeg, 12 * kDeg), ll_all = f.aero.liftingLineResidual(-25 * kDeg, 30 * kDeg);
    std::printf("  нев'язка підйомної лінії: до зриву %.1e, на всьому діапазоні %.1e\n", ll_ok, ll_all);
    CHECK(ll_ok < 1e-5, "підйомна лінія не збіглася до зриву: %.1e", ll_ok);
}

// ---- Гвинти, мотори, батарея QuadPlane ----
// Уточнення моделі (крок 3): динамічний зрив, ефект землі, вихрове кільце, слід підйомних гвинтів, затінення кіля, термічні потоки
static void testQuadPlaneAeroEffects() {
    SECTION("Уточнення аеромоделі: динамічний зрив, ефект землі, вихрове кільце, слід гвинтів, кіль, термік");
    const double kDeg = 3.14159265358979323846 / 180;
    {   // динамічний зрив: коливання α 12° ± 8° (≈ 2 Гц, k = ωc/2V ≈ 0.15) — петля гістерезису, динамічний максимум вище статичного
        AeroFixture f;
        f.aero.setDynamicStall(true);
        const double V = 12, w = 2 * 3.14159265358979323846 * 2.0, dt = 0.001;
        double cl_up = 0, cl_dn = 0, cl_max_dyn = 0;
        for (int k = 0; k < 3000; ++k) {
            const double t = k * dt, a = (12 + 8 * std::sin(w * t)) * kDeg, ad = 8 * kDeg * w * std::cos(w * t);
            AeroInputs in;
            in.v_body = Vec3d{V * std::cos(a), 0, -V * std::sin(a)}; in.rho = f.kRho;
            f.aero.advance(dt, in);
            const double cl = AeroFixture::lift(f.aero.eval(in), a / kDeg) / (0.5 * f.kRho * V * V * f.kS);
            if (t > 1.0) {
                cl_max_dyn = std::fmax(cl_max_dyn, cl);
                if (std::fabs(a / kDeg - 16) < 0.3) (ad > 0 ? cl_up : cl_dn) = cl;
            }
        }
        const AeroFixture g;   // статична модель (без накопиченого стану відриву)
        double cl_max_st = 0;
        for (double a = 0; a <= 30; a += 0.25) cl_max_st = std::fmax(cl_max_st, AeroFixture::lift(g.at(V, a), a) / (0.5 * g.kRho * V * V * g.kS));
        std::printf("  динамічний зрив: CL(16°) вгору %.2f / вниз %.2f; CLmax динамічний %.2f, статичний %.2f\n", cl_up, cl_dn, cl_max_dyn, cl_max_st);
        CHECK(cl_up > cl_dn + 0.05, "гістерезис зриву: CL на підйомі α (%.2f) має бути більшим, ніж на спаді (%.2f)", cl_up, cl_dn);
        CHECK(cl_max_dyn > cl_max_st + 0.03 && cl_max_dyn < cl_max_st * 1.6, "динамічний CLmax %.2f проти статичного %.2f", cl_max_dyn, cl_max_st);
    }
    {   // ефект землі крила: на 0.3 м індукований опір менший, підйомна сила при тому ж α — більша
        AeroFixture f;
        auto at_h = [&](double h) { AeroInputs in; const double a = 6 * kDeg; in.v_body = Vec3d{12 * std::cos(a), 0, -12 * std::sin(a)}; in.rho = f.kRho; in.h_agl = h; return f.aero.eval(in); };
        const AeroResult lo = at_h(0.3), hi = at_h(100);
        const double l_lo = AeroFixture::lift(lo, 6), l_hi = AeroFixture::lift(hi, 6), d_lo = AeroFixture::drag(lo, 6), d_hi = AeroFixture::drag(hi, 6);
        std::printf("  ефект землі крила (h 0.3 м, α 6°): підйомна сила %+.0f%%, опір %+.0f%%\n", 100 * (l_lo / l_hi - 1), 100 * (d_lo / d_hi - 1));
        // індукований опір падає ≈ 8% (φ = 0.92 на h/b ≈ 0.2), але це лише частина повного опору (профіль, корпус, оперення)
        CHECK(l_lo > l_hi * 1.015 && d_lo < d_hi * 0.995, "ефект землі: L %.2f/%.2f, D %.3f/%.3f", l_lo, l_hi, d_lo, d_hi);
    }
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    auto hoverRig = [&](double z, Vec3d vel) {
        auto sim = std::make_unique<QuadPlaneSim>(a, tb, 5);
        sim->place(Vec3d{0, 0, z}, vel, 0, 0, 0);
        sim->setRotorRps(118, 0);
        return sim;
    };
    {   // ефект землі гвинтів: біля землі тяга більша (≈ +3% на висоті шасі), на 2 м — практично немає
        auto lo = hoverRig(0.2, {}), hi = hoverRig(2.0, {});
        QpControls u; for (double& d : u.duty_vtol) d = 0.33;
        for (int k = 0; k < 20; ++k) { lo->step(0.001, u); hi->step(0.001, u); }
        std::printf("  ефект землі гвинтів: множник тяги %.3f (диск ≈ %.2f м над землею), %.3f (2 м)\n", lo->vtolFactor(0), 0.2 + 0.03, hi->vtolFactor(0));
        // передній диск конвертоплана ≈ 0.23 м над землею (у QuadPlane — нижче): ефект слабший, але є
        CHECK(lo->vtolFactor(0) > 1.008 && lo->vtolFactor(0) < 1.12 && std::fabs(hi->vtolFactor(0) - 1) < 0.005, "ефект землі гвинтів %.3f / %.3f", lo->vtolFactor(0), hi->vtolFactor(0));
    }
    {   // вихрове кільце: вертикальне зниження ≈ v_h (≈ 8 м/с) — тяга падає; з горизонтальною швидкістю 12 м/с — ефекту немає
        QpControls u; for (double& d : u.duty_vtol) d = 0.33;
        auto avg = [&](Vec3d vel) {
            auto s = hoverRig(200, vel);
            double m = 0; int n = 0;
            for (int k = 0; k < 400; ++k) { s->setState([&] { QpState st = s->state(); st.vel_b = st.q.rotateInv(vel); st.omega = {}; return st; }()); s->step(0.001, u); if (k > 100) { m += s->vtolFactor(0); ++n; } }
            return m / n;
        };
        const double f_vrs = avg({0, 0, -7.5}), f_fwd = avg({12, 0, -7.5}), f_slow = avg({0, 0, -1.5});
        std::printf("  вихрове кільце: множник тяги %.2f (зниження 7.5 м/с), %.2f (з 12 м/с вперед), %.2f (зниження 1.5 м/с)\n", f_vrs, f_fwd, f_slow);
        CHECK(f_vrs < 0.9 && f_fwd > 0.97 && f_slow > 0.98, "вихрове кільце: %.2f / %.2f / %.2f", f_vrs, f_fwd, f_slow);
    }
    {   // слід гвинтів: у переході (12 м/с, гондоли ще вертикальні) задні гвинти на кінцях V-хвоста дмуть на панелі; передні — поза крилом у плані
        auto run = [&](Vec3d vel, bool wash) {
            auto s = hoverRig(100, vel);
            s->setRotorEffects(false, false, wash);
            QpControls u; for (double& d : u.duty_vtol) d = 0.33;
            for (int k = 0; k < 50; ++k) { s->setState([&] { QpState st = s->state(); st.vel_b = vel; st.omega = {}; st.q = Quatd::fromEuler(0, 0, 0); return st; }()); s->step(0.001, u); }
            return s->aeroNow();
        };
        const AeroResult tr_on = run({12, 0, 0}, true), tr_off = run({12, 0, 0}, false), h_on = run({0, 0, 0}, true), h_off = run({0, 0, 0}, false);
        std::printf("  слід гвинтів на 12 м/с: сила стабілізатора %+.2f Н (без сліду %+.2f), підйомна сила крила %.1f Н (без %.1f); висіння: крило %.2f / %.2f Н\n",
                    tr_on.stab_force.z, tr_off.stab_force.z, tr_on.wing_force.z, tr_off.wing_force.z, h_on.wing_force.z, h_off.wing_force.z);
        CHECK(std::fabs(tr_on.stab_force.z - tr_off.stab_force.z) > 0.1, "слід задніх гвинтів має діяти на V-хвіст у переході");
        CHECK(std::fabs(h_on.wing_force.z - h_off.wing_force.z) < 0.3, "у висінні гвинти поза крилом у плані — слід не має давати помітної сили на крило");
    }
    {   // V-хвіст: «висота» (обидві панелі однаково) майже не дає бічної сили, «напрям» (різницево) — майже не дає підйомної
        AeroFixture f;
        AeroInputs in; in.v_body = Vec3d{14, 0, -0.5}; in.rho = f.kRho;
        const AeroResult r0 = f.aero.eval(in);
        in.elevator = 0.15; const AeroResult re = f.aero.eval(in);
        in.elevator = 0; in.rudder = 0.15; const AeroResult rr = f.aero.eval(in);
        const double dfz_e = re.stab_force.z - r0.stab_force.z, dfy_e = re.stab_force.y - r0.stab_force.y;
        const double dfz_r = rr.stab_force.z - r0.stab_force.z, dfy_r = rr.stab_force.y - r0.stab_force.y;
        std::printf("  V-хвіст: руль висоти 0.15 рад → ΔFz %+.2f Н, ΔFy %+.3f Н; руль напряму 0.15 рад → ΔFy %+.2f Н, ΔFz %+.3f Н, ΔMz %+.2f Н·м\n",
                    dfz_e, dfy_e, dfy_r, dfz_r, rr.moment.z - r0.moment.z);
        CHECK(dfz_e > 0.5 && std::fabs(dfy_e) < 0.02 * std::fabs(dfz_e), "руль висоти V-хвоста: ΔFz %.2f, ΔFy %.3f", dfz_e, dfy_e);
        // різницеве відхилення ±Δα на нелінійній cl(α) дає малий залишок підйомної сили (кілька відсотків від бічної)
        CHECK(dfy_r < -0.2 && std::fabs(dfz_r) < 0.1 * std::fabs(dfy_r) && rr.moment.z > r0.moment.z, "руль напряму V-хвоста: ΔFy %.2f, ΔFz %.3f", dfy_r, dfz_r);
    }
    {   // термічний потік: ядро вгору, кільце спадного потоку, далеко — нуль; дрейф із вітром
        WindConfig wc; wc.turbulence = 0; Thermal th; th.x = 0; th.y = 0; th.w_star = 2.5; th.zi = 1000; wc.thermals = {th};
        Wind w(wc, 1);
        const double core = w.thermalW(0, 0, 150, 0), ring = w.thermalW(70, 0, 150, 0), w_far = w.thermalW(600, 0, 150, 0), w_low = w.thermalW(0, 0, 10, 0);
        std::printf("  термік (w* 2.5 м/с, 150 м): ядро %+.2f м/с, на 70 м %+.2f, на 600 м %+.3f; на 10 м %+.2f\n", core, ring, w_far, w_low);
        CHECK(core > 1.5 && core < 6 && ring < 0 && std::fabs(w_far) < 0.01 && w_low < core, "профіль терміка");
    }
}

static void testQuadPlanePower() {
    SECTION("Гвинти, мотор + ESC, батарея конвертоплана");
    const AircraftFile a = loadQuadPlane();
    const double rho = 1.225, g = 9.80665, W = a.num("targets.takeoff_mass_kg") * g;
    const Propeller vp(a, "vtol_prop");
    // таблиці: CT спадає з J, нуль біля J0, зворотний потік піднімає тягу, вітряк має CP < 0
    CHECK(vp.ct(0.2) < vp.ct(0) && vp.ct(0.5) < vp.ct(0.2) && std::fabs(vp.ct(a.num("vtol_prop.j_zero_ct"))) < 0.01, "CT(J) VTOL-гвинта");
    {   // зупинений гвинт у потоці (задні в польоті, якщо тягнуть лише передні): опір, а не нуль
        const Propeller::Out st = vp.eval(0.0, 15.0, 0.0, rho), fw = vp.eval(40.0, 15.0, 0.0, rho);
        std::printf("  гвинт у потоці 15 м/с: зупинений — сила %+.2f Н, 40 об/с (вітряк) — %+.2f Н\n", st.thrust, fw.thrust);
        CHECK(st.thrust < -0.1 && st.thrust > -0.6, "зупинений гвинт має давати опір: %.2f Н", st.thrust);
    }
    CHECK(vp.ct(-0.2) > vp.ct(0) && vp.cp(1.0) < 0, "зворотний потік підсилює тягу, вітряк віддає потужність: CT(−0.2)=%.3f CP(1.0)=%.3f", vp.ct(-0.2), vp.cp(1.0));
    // висіння: тяга на гвинт = W/4 → оберти і потужність
    double n_hover = 0;
    for (double n = 40; n < 200; n += 0.05) if (vp.eval(n, 0, 0, rho).thrust < W / 4) n_hover = n;
    const Propeller::Out ph = vp.eval(n_hover, 0, 0, rho);
    std::printf("  гвинт VTOL у висінні: %.0f об/хв, тяга %.2f Н, вал %.0f Вт, момент %.3f Н·м\n", n_hover * 60, ph.thrust, ph.power, ph.torque);
    CHECK(n_hover * 60 > 4500 && n_hover * 60 < 8000 && ph.power > 40 && ph.power < 110, "оберти %.0f, потужність %.0f Вт", n_hover * 60, ph.power);
    // мотор + батарея: усталений режим висіння
    PackBattery bat(a);
    MotorEsc m(a, "vtol_motor", vp.inertia());
    double duty = 0.3, p_bus_total = 0, rpm = 0, i_bus = 0;
    for (int it = 0; it < 4000; ++it) {   // підбираємо duty, що дає потрібну тягу (інтегральний регулятор), 4 с
        const Propeller::Out po = vp.eval(m.rps(), 0, 0, rho);
        const MotorEsc::Step s = m.step(0.001, duty, bat.voltage(4 * i_bus), po.torque);
        i_bus = s.bus_current;
        duty += 2e-6 * (W / 4 - po.thrust) * 1000;
        duty = std::clamp(duty, 0.0, 1.0);
    }
    rpm = m.rps() * 60; p_bus_total = 4 * i_bus * bat.voltage(4 * i_bus);
    std::printf("  один мотор у висінні: duty %.2f, %.0f об/хв, %.1f А; усі 4: %.0f Вт від батареї (%.1f В)\n", duty, rpm, i_bus, p_bus_total, bat.voltage(4 * i_bus));
    CHECK(std::fabs(rpm - n_hover * 60) < 150, "мотор не вийшов на оберти висіння: %.0f vs %.0f", rpm, n_hover * 60);
    CHECK(p_bus_total > 230 && p_bus_total < 420 && duty > 0.15 && duty < 0.6, "потужність висіння %.0f Вт, duty %.2f", p_bus_total, duty);
    // розкрутка від нуля до висіння — десятки мс, не секунди
    MotorEsc m2(a, "vtol_motor", vp.inertia());
    double t90 = -1;
    for (int it = 0; it < 2000 && t90 < 0; ++it) {
        m2.step(0.001, duty, bat.voltage(30), vp.eval(m2.rps(), 0, 0, rho).torque);
        if (m2.rps() > 0.9 * n_hover) t90 = it * 0.001;
    }
    std::printf("  розкрутка до 90%% обертів висіння: %.0f мс\n", 1000 * t90);
    CHECK(t90 > 0.02 && t90 < 0.4, "час розкрутки %.3f с", t90);
    // батарея: повна, просідання, холод, енергія
    PackBattery full(a);
    CHECK(std::fabs(full.ocv() - 6 * 4.15) < 0.01, "OCV повного пакета %.2f", full.ocv());
    const double sag = full.voltage(0) - full.voltage(25);
    PackBattery cold(a); cold.setTemperature(0);
    const double sag_cold = cold.voltage(0) - cold.voltage(25);
    std::printf("  просідання на 25 А: %.2f В при 25 °C, %.2f В при 0 °C\n", sag, sag_cold);
    CHECK(sag > 0.8 && sag < 2.0 && sag_cold > 1.4 * sag, "просідання %.2f / %.2f В", sag, sag_cold);
    PackBattery b2(a);
    double t = 0, e_wh = 0, tmax = 0;
    while (!b2.depleted() && t < 7200) {
        b2.step(1.0, 10.0); t += 1.0; tmax = std::fmax(tmax, b2.temperature());
    }
    e_wh = b2.usedWh();
    std::printf("  розряд 10 А до відсічки: %.0f хв, %.0f Вт·год, макс. температура %.0f °C\n", t / 60, e_wh, tmax);
    CHECK(e_wh > 140 && e_wh < 185 && t > 600, "енергія %.0f Вт·год за %.0f с", e_wh, t);
    CHECK(tmax > 25 && tmax < 80, "температура %.0f °C", tmax);
    CHECK(!full.depleted() && PackBattery(a).capacityAh() > 7.0, "повний пакет не розряджений, ємність %.2f А·год", PackBattery(a).capacityAh());
    // некоректні таблиці в файлі відхиляються
    AircraftFile bad = a; bad.values["vtol_prop.ct_table"].pop_back();
    bool threw = false; try { Propeller p(bad, "vtol_prop"); } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw, "таблиці різної довжини мають кидати помилку");
}

// ---- Повна модель QuadPlane: середовище, земля, скидання, енергія ----
static void testQuadPlaneSim() {
    SECTION("Повна модель конвертоплана (6DOF + середовище + земля + поворот гондол)");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    const double kDeg = 3.14159265358979323846 / 180;
    {   // атмосфера ISA
        Atmosphere isa, hot; hot.temp_offset_c = 20;
        std::printf("  ISA: ρ(0) %.4f, ρ(1000 м) %.4f, ρ(0, ISA+20) %.4f кг/м³\n", isa.density(0), isa.density(1000), hot.density(0));
        CHECK(std::fabs(isa.density(0) - 1.225) < 0.002 && std::fabs(isa.density(1000) - 1.112) < 0.004 && hot.density(0) < 0.95 * isa.density(0), "ISA");
    }
    {   // турбулентність Драйдена: σ_w = 0.1·W20, σ_u = σ_w / (0.177 + 0.000823 h_ft)^0.4
        WindConfig wc; wc.mean20ft = Vec3d{5, 0, 0};
        Wind w(wc, 7);
        const double h = 50, sw_expect = 0.5, su_expect = sw_expect / std::pow(0.177 + 0.000823 * h * 3.28084, 0.4);
        double su2 = 0, sw2 = 0; long n = 0;
        const Vec3d mean = w.mean(h);
        for (int i = 0; i < 40000; ++i) {
            const Vec3d v = w.step(0.02, h, 17);
            if (i > 2000) { su2 += (v.x - mean.x) * (v.x - mean.x); sw2 += v.z * v.z; ++n; }
        }
        const double su = std::sqrt(su2 / n), sw = std::sqrt(sw2 / n);
        std::printf("  Драйден на 50 м, W20 = 5 м/с: σ_u %.2f (очікується %.2f), σ_w %.2f (очікується %.2f)\n", su, su_expect, sw, sw_expect);
        CHECK(std::fabs(su - su_expect) < 0.3 * su_expect && std::fabs(sw - sw_expect) < 0.3 * sw_expect, "статистика турбулентності");
        WindConfig calm; Wind w0(calm, 1);
        CHECK(w0.step(0.01, 30, 17).norm() == 0, "без середнього вітру турбулентності немає");
    }
    {   // стоїть на землі з вимкненими моторами: шасі тримає, нічого не дрейфує
        QuadPlaneSim s(a, tb);
        const std::vector<double> gp = a.nums("ground.gear_points");
        double gz = 1e9; for (size_t k = 2; k < gp.size(); k += 3) gz = std::fmin(gz, gp[k]);
        const double leg_h = s.derived().cg_z - gz;   // низ ніжки під ЦМ
        s.place(Vec3d{0, 0, leg_h + 0.005}, Vec3d{}, 0, 0, 0);
        for (int i = 0; i < 4000; ++i) s.step(0.001, QpControls{});
        std::printf("  на землі 4 с: висота ЦМ %.3f м (ніжки %.3f), швидкість %.3f м/с, нахил %.2f°, аварія %d\n", s.state().pos.z, leg_h, s.velWorld().norm(),
                    std::acos(s.state().q.rotate(Vec3d{0, 0, 1}).z) / kDeg, s.crashed());
        CHECK(!s.crashed() && !s.diverged() && s.velWorld().norm() < 0.02 && std::fabs(s.state().pos.z - (leg_h - 0.003)) < 0.002 &&
              std::acos(s.state().q.rotate(Vec3d{0, 0, 1}).z) < 0.5 * kDeg, "посадка на ніжки TPU: стиснення ≈ 3 мм, рівно");
    }
    {   // падіння без моторів, штиль: спершу a ≈ g, далі опір (крило плиткою ≈ 0.45 м², Cd ≈ 1.2) дає усталену швидкість ≈ 8–10 м/с
        QuadPlaneSim s(a, tb);
        s.place(Vec3d{0, 0, 200}, Vec3d{}, 0, 0, 0);
        for (int i = 0; i < 300; ++i) s.step(0.001, QpControls{});
        const double v03 = s.velWorld().z;
        for (int i = 0; i < 700; ++i) s.step(0.001, QpControls{});
        const double v1 = s.velWorld().z;
        for (int i = 0; i < 5000; ++i) s.step(0.001, QpControls{});
        std::printf("  падіння: vz(0.3 с) %.2f, vz(1 с) %.2f м/с; через 6 с без керування літак переходить у пікірування: %.1f м/с\n", v03, v1, s.velWorld().norm());
        CHECK(v03 < -2.7 && v03 > -3.0, "початкове прискорення ≈ g: vz(0.3 с) = %.2f", v03);
        CHECK(v1 < -6.5 && v1 > -9.5, "опір плоского крила вже гальмує: vz(1 с) = %.2f", v1);
        CHECK(!s.diverged() && s.velWorld().norm() < 60, "некероване падіння: швидкість %.1f м/с", s.velWorld().norm());
    }
    {   // планерування 17 м/с без моторів: енергія тільки спадає, жодних NaN, пікірування стабільне
        QuadPlaneSim s(a, tb);
        s.place(Vec3d{0, 0, 200}, Vec3d{17, 0, -0.5}, 0, 0.5 * kDeg, 0);
        auto energy = [&]() { return 0.5 * s.mass() * s.velWorld().dot(s.velWorld()) + s.mass() * 9.80665 * s.state().pos.z; };
        double e0 = energy(), e_prev = e0, worst_gain = 0, t_end = 0;
        QpControls u; u.elevator = -0.05;
        for (int i = 0; i < 8000; ++i) {
            s.step(0.001, u);
            if (i % 100 == 99) { const double e = energy(); worst_gain = std::fmax(worst_gain, e - e_prev); e_prev = e; }
            t_end = s.time();
        }
        const double sink = (200 - s.state().pos.z) / t_end;
        std::printf("  планер 8 с: швидкість %.1f м/с, зниження %.2f м/с, втрата енергії %.0f Дж, найбільший приріст за 0.1 с %.2f Дж\n", s.airspeed(), sink, e0 - energy(), worst_gain);
        CHECK(!s.diverged() && s.state().pos.finite() && energy() < e0, "енергія має спадати");
        // без тяги й вітру повна енергія не може зростати; 1 Дж — лише на обмін з енергією обертання (тут не врахована)
        CHECK(worst_gain < 1.0, "приріст енергії за 0.1 с %.2f Дж", worst_gain);
    }
    {   // поворот гондол: привід ≈ 90°/с, тяга повертається разом з гондолою (висіння → уперед)
        QuadPlaneSim s(a, tb);
        s.place(Vec3d{0, 0, 100}, Vec3d{}, 0, 0, 0);
        QpControls u; for (int i = 0; i < 4; ++i) { u.duty_vtol[i] = 0.25; u.tilt[i] = 3.14159265358979323846 / 2; }
        double t_half = -1;
        for (int k = 0; k < 2000; ++k) { s.step(0.001, u); if (t_half < 0 && s.tilt(0) > 0.25 * 3.14159265358979323846) t_half = s.time(); }
        Vec3d F, M; s.forcesAndMoments(s.state(), F, M);
        std::printf("  поворот гондол 0 → 90°: половина за %.2f с, через 2 с %.1f° %.1f° %.1f° %.1f°; сила гвинтів уздовж X тіла %.1f Н\n", t_half,
                    s.tilt(0) / kDeg, s.tilt(1) / kDeg, s.tilt(2) / kDeg, s.tilt(3) / kDeg, F.x);
        CHECK(t_half > 0.4 && t_half < 0.65 && std::fabs(s.tilt(0) - 3.14159265358979323846 / 2) < 0.01, "привід нахилу: половина ходу за %.2f с", t_half);
        CHECK(F.x > 5, "гондоли на 90°: гвинти тягнуть уперед (%.1f Н)", F.x);
        CHECK(!s.diverged(), "модель стабільна");
    }
    {   // слід переднього гвинта (гондола на 90°) підсилює підйомну силу смуг крила за ним
        const AeroFixture f;
        AeroInputs in; in.v_body = Vec3d{10, 0, -0.5}; in.rho = 1.225;
        const double l0 = f.aero.eval(in).wing_force.z;
        in.wash = [](const Vec3d& r) { return std::fabs(std::fabs(r.y) - 0.40) < 0.11 ? Vec3d{-6, 0, 0} : Vec3d{}; };
        const double l1 = f.aero.eval(in).wing_force.z;
        CHECK(l1 > l0 + 0.5, "слід гвинта має підсилювати підйомну силу: %.2f → %.2f Н", l0, l1);
    }
}

// ---- Аналіз QuadPlane: тримування, моди, місія ----
static void testQuadPlaneAnalysis() {
    SECTION("Аналіз конвертоплана: тримування, моди, потужність, перехід, місія");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    QuadPlaneSim s(a, tb);
    const double kDeg = 3.14159265358979323846 / 180;
    // тримування на 10/14/15/16/20 м/с — гондоли на 90°; у рівному польоті тягнуть передні (cruise_rear = 2), задні стоять
    TrimResult tr[5]; const double vs[5] = {10, 14, 15, 16, 20};
    bool all_ok = true;
    for (int i = 0; i < 5; ++i) { tr[i] = trimCruise(s, vs[i]); all_ok = all_ok && tr[i].ok && tr[i].residual < 1e-3; }
    std::printf("  тримування: 10 м/с α %.1f° / %.0f Вт; 14 м/с α %.1f° / %.0f Вт; 15 м/с α %.1f°, кермо %.1f°, %.0f об/хв, %.0f Вт; 20 м/с %.0f Вт\n",
                tr[0].alpha / kDeg, tr[0].elec_w, tr[1].alpha / kDeg, tr[1].elec_w, tr[2].alpha / kDeg, tr[2].elevator / kDeg, tr[2].cruise_rps * 60, tr[2].elec_w, tr[4].elec_w);
    CHECK(all_ok, "тримування на 10/14/15/16/20 м/с має збігатися");
    CHECK(tr[1].alpha > 0.5 * kDeg && tr[1].alpha < 6 * kDeg && tr[2].alpha > 0 && tr[2].alpha < 6 * kDeg, "α на крейсері %.1f° / %.1f°", tr[1].alpha / kDeg, tr[2].alpha / kDeg);
    CHECK(std::fabs(tr[1].elevator) < 0.5 * 25 * kDeg && std::fabs(tr[2].elevator) < 0.5 * 25 * kDeg, "кермо висоти %.1f° / %.1f°", tr[1].elevator / kDeg, tr[2].elevator / kDeg);
    // гвинти для висіння на малому навантаженні — низький ККД у польоті (J ≈ 0.6, CT ≈ 0.02); 2 передні економніші за 4 (testTiltrotor)
    CHECK(tr[2].elec_w > 100 && tr[2].elec_w < 260, "потужність крейсеру 15 м/с %.0f Вт", tr[2].elec_w);
    CHECK(tr[0].alpha > tr[1].alpha && tr[1].alpha > tr[2].alpha && tr[2].alpha > tr[4].alpha, "α має спадати зі швидкістю");
    CHECK(tr[4].elec_w > tr[2].elec_w && tr[2].elec_w > tr[1].elec_w, "потужність на крейсері зростає зі швидкістю вище мінімуму");
    // моди на 15 м/с
    const LinearModes lm = linearizeCruise(s, tr[2]);
    int unstable_osc = 0; double zsp = 0, wsp = 0, zph = 0, ph_w = 1e9, spiral = 0, roll = 0, zdr = 0, wdr = 0;
    for (auto z : lm.long_eig) {
        if (z.imag() > 1e-6) { if (std::abs(z) > wsp) { wsp = std::abs(z); zsp = -z.real() / std::abs(z); } if (std::abs(z) < ph_w) { ph_w = std::abs(z); zph = -z.real() / std::abs(z); } }
        if (z.real() > 1e-6 && std::fabs(z.imag()) > 1e-6) ++unstable_osc;
    }
    for (auto z : lm.lat_eig) {
        if (z.imag() > 1e-6) { wdr = std::abs(z); zdr = -z.real() / wdr; if (z.real() > 0) ++unstable_osc; }
        else if (std::fabs(z.imag()) <= 1e-6) { if (z.real() < roll) roll = z.real(); if (std::fabs(z.real()) < std::fabs(spiral) || spiral == 0) spiral = z.real(); }
    }
    std::printf("  моди 15 м/с: короткий період ωn %.1f ζ %.2f; фугоїда ωn %.2f ζ %.2f; голландський крок ωn %.1f ζ %.2f; крен %.1f 1/с; спіраль %+.3f 1/с\n", wsp, zsp, ph_w, zph, wdr, zdr, roll, spiral);
    CHECK(unstable_osc == 0, "нестійкі коливальні моди: %d", unstable_osc);
    CHECK(wsp > 6 && wsp < 20 && zsp > 0.3 && zsp < 1.0, "короткий період ωn %.1f ζ %.2f", wsp, zsp);
    // фугоїда: 4 гвинти з тягою, що падає зі швидкістю, демпфують і «підтягують» її (ωn вище за √2·g/V ≈ 0.9)
    CHECK(zph > 0.03 && ph_w > 0.3 && ph_w < 1.6, "фугоїда ωn %.2f ζ %.2f", ph_w, zph);
    CHECK(wdr > 1.5 && wdr < 9 && zdr > 0.05, "голландський крок ωn %.1f ζ %.2f", wdr, zdr);
    CHECK(roll < -5 && roll > -40, "крен-демпфування %.1f 1/с", roll);
    CHECK(std::fabs(spiral) < 0.3, "спіральна мода %+.3f 1/с (повільна)", spiral);
    // висіння, перехід: крило розвантажує гвинти, гондоли нахиляються вперед зі швидкістю
    const TrimResult hov = hoverPower(s);
    QuadPlaneSim s5(a, tb), s8(a, tb);
    const TrimResult t5 = trimTransition(s5, 5, 4), t8 = trimTransition(s8, 8, 4);
    std::printf("  висіння %.0f Вт; перехід 5 м/с (α 4°): гондоли %.0f°, %.0f Вт; 8 м/с: %.0f°, %.0f Вт\n", hov.elec_w, t5.tilt / kDeg, t5.elec_w, t8.tilt / kDeg, t8.elec_w);
    CHECK(hov.ok && hov.elec_w > 230 && hov.elec_w < 420, "висіння %.0f Вт", hov.elec_w);
    CHECK(t5.ok && t8.ok && t5.elec_w < hov.elec_w && t8.elec_w < t5.elec_w, "крило розвантажує гвинти: %.0f → %.0f → %.0f Вт", hov.elec_w, t5.elec_w, t8.elec_w);
    CHECK(t5.tilt > 0 && t8.tilt > t5.tilt && t8.tilt < 60 * kDeg, "нахил гондол росте зі швидкістю: %.0f° → %.0f°", t5.tilt / kDeg, t8.tilt / kDeg);
    // місія: 5 км туди, висіння 60 с, 5 км назад
    const MissionEstimate m = estimateMission(a, tb);
    std::printf("  місія: %.1f Вт·год за %.1f хв (запас %.0f%%), крейсер %.0f Вт; тільки крейсер: %.0f хв, %.0f км\n", m.total_wh, m.total_min,
                100 * (1 - m.total_wh / m.usable_wh), m.cruise_w, m.endurance_min, m.range_km);
    CHECK(m.ok && m.total_wh < 0.6 * m.usable_wh, "місія вимагає %.1f Вт·год з %.0f", m.total_wh, m.usable_wh);
    CHECK(m.endurance_min >= 30 && m.range_km >= 25, "тривалість %.0f хв, дальність %.0f км", m.endurance_min, m.range_km);
}

// ---- Контролер QuadPlane: L1, TECS, мікшер, зліт, переходи, місія зі скиданням, вітер, шум ----
static std::vector<fc::QpStep> qpMission(float leg_m) {
    using S = fc::QpStep;
    S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30};
    S hv; hv.type = S::HoverAt; hv.p = {leg_m, 0, 40}; hv.hold_s = 12;   // висіння над точкою (огляд/камера) 12 с
    S ld; ld.type = S::Land; ld.p = {0, 0, 0};
    return {tk, hv, ld};
}

struct FlightStats {
    bool landed = false, crashed = false; bool hovered = false;
    double end_x = 0, end_y = 0, touch_vz = 0, max_roll = 0, min_z_transfwd = 1e9, z_cruise_min = 1e9, z_cruise_max = -1e9, v_cruise_min = 1e9, v_cruise_max = -1e9, time = 0, used_wh = 0;
    double max_hover_err = 0; bool saw_cruise = false, saw_transback = false, assisted = false;
    int phases_seen = 0;
};

static FlightStats flyMission(const AircraftFile& a, const AeroTables& tb, float leg, double wind, unsigned seed, const SensorNoise& noise, double limit_s = 600) {
    QpFlightRig rig(a, tb, seed);
    rig.noise = noise;
    rig.setMission(qpMission(leg));
    if (wind > 0) { WindConfig w; w.mean20ft = Vec3d{wind, 0.3 * wind, 0}; rig.setWind(w, seed + 10); }
    FlightStats s;
    unsigned mask = 0; double prev_vz = 0;   // prev_vz: мін. вертикальна швидкість при заходженні на посадку
    rig.run(limit_s, [&](const QpFlightRig& r) {
        const fc::QpPhase ph = r.ctl.phase();
        mask |= 1u << int(ph);
        const double z = r.sim.state().pos.z, va = r.sim.airspeed();
        const Vec3d eu = r.sim.eulerRad();
        s.max_roll = std::fmax(s.max_roll, std::fabs(eu.x));
        if (ph == fc::QpPhase::TransFwd) s.min_z_transfwd = std::fmin(s.min_z_transfwd, z);
        if (ph == fc::QpPhase::Cruise && r.ctl.stepIndex() == 1 && r.ctl.vtolWeight() == 0 && r.ctl.phaseTime() > 12.0f) { s.z_cruise_min = std::fmin(s.z_cruise_min, z); s.z_cruise_max = std::fmax(s.z_cruise_max, z); s.v_cruise_min = std::fmin(s.v_cruise_min, va); s.v_cruise_max = std::fmax(s.v_cruise_max, va); }
        if (r.ctl.assisting()) s.assisted = true;
        if (ph == fc::QpPhase::VtolHold && r.ctl.stepIndex() == 1 && std::hypot(r.sim.state().pos.x - leg, r.sim.state().pos.y) < 4.0 && r.ctl.phaseTime() > 10.0f) s.hovered = true;
        if (ph == fc::QpPhase::Landing && z < 1.5) prev_vz = std::fmin(prev_vz, r.sim.velWorld().z);   // найшвидше зниження біля землі
        return ph == fc::QpPhase::Landed;
    });
    const QpState& st = rig.sim.state();
    s.landed = rig.ctl.phase() == fc::QpPhase::Landed; s.crashed = rig.sim.crashed() || rig.sim.diverged();
    s.end_x = st.pos.x; s.end_y = st.pos.y; s.time = rig.sim.time(); s.used_wh = rig.sim.battery().usedWh();
    s.saw_cruise = mask & (1u << int(fc::QpPhase::Cruise)); s.saw_transback = mask & (1u << int(fc::QpPhase::TransBack));
    for (unsigned m = mask; m; m >>= 1) s.phases_seen += m & 1;
    // вертикальна швидкість перед посадкою: останній рядок журналу
    s.touch_vz = prev_vz;   // найбільша швидкість зниження в Landing нижче 1.5 м (від'ємна)
    return s;
}

static void testQuadPlaneController() {
    SECTION("Контролер конвертоплана: L1, TECS, мікшер, переходи, місія з висінням над точкою");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    {   // L1: літак праворуч від лінії AB (на схід, y < 0) → потрібно повернути ліворуч (+a_left); на лінії — нуль
        fc::L1Guidance l1;
        const fc::Vec3 A{0, 0, 50}, B{1000, 0, 50};
        const float right = l1.update({100, -30, 50}, {17, 0, 0}, A, B), left = l1.update({100, 30, 50}, {17, 0, 0}, A, B), on = l1.update({100, 0, 50}, {17, 0, 0}, A, B);
        CHECK(right > 0.3f && left < -0.3f && std::fabs(on) < 1e-3f, "L1: праворуч %.2f, ліворуч %.2f, на лінії %.3f м/с²", right, left, on);
        CHECK(std::fabs(right) < 0.5f * 9.81f * 4, "L1: прискорення в межах крену");
    }
    {   // TECS: нижче цілі — тангаж і тяга зростають, вище — спадають
        fc::Tecs t; t.setTrim({9, 17, 22}, {4.3f, 5.3f, 8.2f}, {0.18f, 0.03f, -0.004f});
        t.reset(60, 17);
        for (int i = 0; i < 500; ++i) t.update(0.002f, 3.5f, 50, 0, 17, 60, 17);   // на 10 м нижче
        const float p_lo = t.pitchUpDemand(), th_lo = t.thrustDemand();
        fc::Tecs t2; t2.setTrim({9, 17, 22}, {4.3f, 5.3f, 8.2f}, {0.18f, 0.03f, -0.004f});
        t2.reset(70, 17);
        for (int i = 0; i < 500; ++i) t2.update(0.002f, 3.5f, 70, 0, 17, 60, 17);  // на 10 м вище
        std::printf("  TECS: нижче цілі — тангаж %.1f°, тяга %.1f Н; вище — %.1f°, %.1f Н\n", p_lo * 57.3f, th_lo, t2.pitchUpDemand() * 57.3f, t2.thrustDemand());
        CHECK(p_lo > t2.pitchUpDemand() + 0.05f && th_lo > t2.thrustDemand(), "TECS має підіймати знизу й опускати зверху");
    }
    {   // мікшер: розподіл тяги й моментів, задні мотори ближче до осі (кінці V-хвоста), ніж передні
        fc::MixerConfig m; m.arm_x = 0.50f; m.arm_y = 0.40f; m.arm_y_rear = 0.295f; m.k_yaw = 0.0164f; m.max_thrust = 25; m.idle = 0.04f;
        const fc::Vec3 tq{0.5f, -0.3f, 0.05f};
        const fc::MixerResult r = fc::mixQuadX(22.3f, tq, m);
        fc::Vec3 back; float T = 0;
        for (int i = 0; i < 4; ++i) { const float dy = fc::kQuadX[i].sx > 0 ? m.dy() : m.dyRear(); back.x += fc::kQuadX[i].sy * dy * r.thrust[i]; back.y += -fc::kQuadX[i].sx * m.dx() * r.thrust[i]; back.z += fc::kQuadX[i].yaw_sign * m.k_yaw * r.thrust[i]; T += r.thrust[i]; }
        CHECK(std::fabs(T - 22.3f) < 0.01f && std::fabs(back.x - tq.x) < 0.01f && std::fabs(back.y - tq.y) < 0.01f, "мікшер: T %.2f, моменти (%.3f %.3f %.3f)", T, back.x, back.y, back.z);
    }
    SensorNoise none;
    {   // зліт і утримання в штиль/вітрі
        QpFlightRig rig(a, tb, 1);
        using S = fc::QpStep; S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30};
        rig.setMission({tk});
        WindConfig w; w.mean20ft = Vec3d{4, 1.5, 0}; rig.setWind(w, 5);
        rig.run(60);
        double ez = 0, exy = 0;
        for (const FlightRow& r : rig.log) if (r.t > 30) { ez = std::fmax(ez, std::fabs(r.z - 30)); exy = std::fmax(exy, std::hypot(r.x, r.y)); }
        std::printf("  зліт на 30 м і утримання у вітрі 4 м/с (+пориви): макс. відхилення по висоті %.2f м, по горизонталі %.2f м\n", ez, exy);
        CHECK(!rig.sim.crashed() && ez < 1.5 && exy < 2.5, "висіння у вітрі: dz %.2f dxy %.2f", ez, exy);
    }
    FlightStats m0 = flyMission(a, tb, 600, 0, 1, none);
    std::printf("  місія 600 м у штиль: час %.0f с, висіння над точкою %s, посадка (%.2f, %.2f) м, мін. висота в переході вперед %.1f м, крейсер: висота %.1f…%.1f м, V %.1f…%.1f м/с, макс. крен %.0f°, %.1f Вт·год\n",
                m0.time, m0.hovered ? "так" : "НІ", m0.end_x, m0.end_y, m0.min_z_transfwd, m0.z_cruise_min, m0.z_cruise_max, m0.v_cruise_min, m0.v_cruise_max, m0.max_roll * 57.3, m0.used_wh);
    CHECK(m0.landed && !m0.crashed && m0.hovered, "місія з висінням над точкою: посадка %d, аварія %d, висіння %d", m0.landed, m0.crashed, m0.hovered);
    CHECK(std::hypot(m0.end_x, m0.end_y) < 3.0 && m0.touch_vz > -1.2 && m0.touch_vz < -0.1, "посадка біля дому: відхилення %.1f м, vz %.2f", std::hypot(m0.end_x, m0.end_y), m0.touch_vz);
    CHECK(m0.saw_cruise && m0.saw_transback && m0.phases_seen >= 6, "переходи в літак і назад пройдені: фаз %d", m0.phases_seen);
    CHECK(m0.min_z_transfwd > 30 - 3.0, "перехід вперед без втрати висоти понад 3 м: мін. %.1f м", m0.min_z_transfwd);
    CHECK(m0.z_cruise_max > m0.z_cruise_min && m0.z_cruise_min > 36 && m0.z_cruise_max < 44 && m0.v_cruise_min > 13.5 && m0.v_cruise_max < 19.5, "крейсер: висота %.1f…%.1f, V %.1f…%.1f", m0.z_cruise_min, m0.z_cruise_max, m0.v_cruise_min, m0.v_cruise_max);
    CHECK(m0.max_roll < 45 * 3.14159 / 180 && m0.used_wh < 40, "крен %.0f°, енергія %.1f Вт·год", m0.max_roll * 57.3, m0.used_wh);
    FlightStats m6 = flyMission(a, tb, 600, 6, 2, none);
    std::printf("  місія 600 м, вітер 6 м/с на 6 м (≈7.8 м/с на 40 м) з поривами Драйдена: час %.0f с, посадка (%.1f, %.1f) м, %.1f Вт·год, аварія %d, висіння %d\n", m6.time, m6.end_x, m6.end_y, m6.used_wh, m6.crashed, m6.hovered);
    CHECK(m6.landed && !m6.crashed && m6.hovered && std::hypot(m6.end_x, m6.end_y) < 4.0, "місія у вітрі 6 м/с");
    SensorNoise nz; nz.gyro_rad_s = 0.01; nz.airspeed_mps = 0.3; nz.pos_m = 0.4; nz.vel_mps = 0.08; nz.att_rad = 0.004;
    int ok_noise = 0;
    for (unsigned sd = 3; sd < 6; ++sd) { const FlightStats n = flyMission(a, tb, 600, 3, sd, nz); ok_noise += n.landed && !n.crashed && n.hovered && std::hypot(n.end_x, n.end_y) < 5.0; }
    std::printf("  місія з шумом датчиків і вітром 3 м/с: успішно %d з 3\n", ok_noise);
    CHECK(ok_noise == 3, "шум датчиків: успішно %d з 3", ok_noise);
    {   // Q_ASSIST: заниження повітряної швидкості датчиком на 9 м/с → коптерні мотори підхоплюють, без аварії
        SensorNoise bias; bias.airspeed_bias = -9.0;
        QpFlightRig rig(a, tb, 7);
        rig.setMission(qpMission(900));
        bool assisted = false;
        double z_end = 0;
        rig.run(90, [&](const QpFlightRig& r) { if (r.sim.time() > 55) rig.noise = bias; if (r.sim.time() > 55) assisted = assisted || r.ctl.assisting(); z_end = r.sim.state().pos.z; return false; });   // датчик «бреше» вже в крейсері
        CHECK(!rig.sim.crashed() && !rig.sim.diverged(), "хибна швидкість -9 м/с: аварія");
        std::printf("  хибна (занижена) повітряна швидкість: Q_ASSIST %s, аварія %d, висота %.1f м\n", assisted ? "спрацював" : "НЕ спрацював", rig.sim.crashed(), rig.sim.state().pos.z);
        CHECK(assisted, "Q_ASSIST мав спрацювати при V < %.0f м/с", rig.ctl.cfg.assist_speed);
        CHECK(z_end > 20, "після спрацювання допомоги апарат не втратив висоту: %.1f м", z_end);
    }
}

// Сильний вітер (крок 3): висіння носом проти вітру (Q_WVANE) з маршовим гвинтом замість нахилу носом униз (Q_FWD_THR_USE)
static void testQuadPlaneStrongWind() {
    SECTION("Сильний вітер: місія 600 м при 9 і 10 м/с (на 6 м), Q_WVANE + тяга нахилом гондол");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    SensorNoise nz; nz.gyro_rad_s = 0.01; nz.airspeed_mps = 0.3; nz.pos_m = 0.5; nz.vel_mps = 0.1; nz.att_rad = 0.004; nz.delay_att_s = 0.02; nz.delay_gps_s = 0.15; nz.gps_hz = 10; nz.rpm_frac = 0.01;
    for (unsigned sd : {41u, 42u}) for (double w : {9.0, 10.0}) {   // два різні вітри (seed) — не підігнано під один
        const FlightStats s = flyMission(a, tb, 600, w, sd, nz, 700);
        std::printf("  вітер %.0f м/с: посадка %s, висіння над точкою %s, похибка посадки %.1f м, час %.0f с, %.1f Вт·год\n", w, s.landed && !s.crashed ? "так" : "НІ", s.hovered ? "так" : "НІ",
                    std::hypot(s.end_x, s.end_y), s.time, s.used_wh);
        CHECK(s.landed && !s.crashed && s.hovered && std::hypot(s.end_x, s.end_y) < 5.0, "місія у вітрі %.0f м/с має виконуватись повністю", w);
    }
}

// Оцінювач (пункт 5): сирі датчики → EKF на двох IMU → контролер. Точність, відмови IMU, вібрації, втрата GPS
static void testQuadPlaneEkf() {
    SECTION("EKF на сирих датчиках: точність у місії, відмови IMU, вібрації, втрата GPS");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    struct Err { double att_rms = 0, att_max = 0, pos_rms = 0, tilt_rms = 0; long n = 0; };
    auto track = [](const QpFlightRig& r, Err& e) {
        if (!r.nav.ready() || r.sim.time() < 5) return;
        const fc::Eskf& L = r.nav.primary();
        const QpState& s = r.sim.state();
        const fc::Quat qt(float(s.q.w), float(s.q.x), float(s.q.y), float(s.q.z));
        const fc::Quat dq = qt.conj() * L.att();
        const double ang = 2 * std::acos(std::fmin(1.0, std::fabs(double(dq.w)))) * 57.29578;
        const double dp2 = std::pow(L.pos().x - s.pos.x, 2) + std::pow(L.pos().y - s.pos.y, 2) + std::pow(L.pos().z - s.pos.z, 2);
        // нахил окремо від курсу: кут між осями Z тіла (оцінка проти правди)
        const fc::Vec3 zt = qt.rotate({0, 0, 1}), ze = L.att().rotate({0, 0, 1});
        const double tilt = std::acos(std::fmin(1.0, double(zt.dot(ze)))) * 57.29578;
        e.att_rms += ang * ang; e.att_max = std::fmax(e.att_max, ang); e.pos_rms += dp2; e.tilt_rms += tilt * tilt; ++e.n;
    };
    auto fin = [](Err& e) { if (e.n) { e.att_rms = std::sqrt(e.att_rms / double(e.n)); e.pos_rms = std::sqrt(e.pos_rms / double(e.n)); e.tilt_rms = std::sqrt(e.tilt_rms / double(e.n)); } };
    auto makeRig = [&](double wind, unsigned seed) {
        auto r = std::make_unique<QpFlightRig>(a, tb, seed);
        r->noise.use_ekf = true; r->noise.airspeed_mps = 0.3; r->noise.rpm_frac = 0.01;
        if (wind > 0) { WindConfig w; w.mean20ft = Vec3d{wind, 0.3 * wind, 0}; r->setWind(w, seed + 10); }
        return r;
    };
    {   // повна місія з EKF
        auto rig = makeRig(3, 41);
        rig->setMission(qpMission(600));
        Err e;
        rig->run(500, [&](const QpFlightRig& r) { track(r, e); return r.ctl.phase() == fc::QpPhase::Landed; });
        fin(e);
        std::printf("  місія 600 м, вітер 3 м/с: посадка %s, похибка нахилу RMS %.2f°, орієнтації RMS %.2f° (макс. %.1f°), позиції RMS %.2f м\n",
                    rig->ctl.phase() == fc::QpPhase::Landed ? "так" : "НІ", e.tilt_rms, e.att_rms, e.att_max, e.pos_rms);
        CHECK(rig->ctl.phase() == fc::QpPhase::Landed && !rig->sim.crashed(), "місія з EKF має виконуватись");
        CHECK(e.att_rms < 3.0 && e.att_max < 12.0 && e.pos_rms < 1.2, "точність EKF: орієнтація %.2f°/%.1f°, позиція %.2f м", e.att_rms, e.att_max, e.pos_rms);
    }
    {   // відмови IMU №1 у висінні (t = 30 с): смуга має перемкнутись на IMU №2, апарат — висіти рівно
        const char* names[] = {"", "стрибок біасу гіро 14°/с", "стрибок біасу акселерометра 2.5 м/с²", "вихід завис", "IMU мертвий"};
        for (int f = 1; f <= 4; ++f) {
            auto rig = makeRig(3, 43);
            fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig->setMission({tk});
            Err e; double drift = 0; bool inj = false;
            rig->run(55, [&](const QpFlightRig& r) {
                if (!inj && r.sim.time() > 30) { rig->sensors.imu_fault[0] = ImuFault(f); inj = true; }
                if (r.sim.time() > 30) { track(r, e); drift = std::fmax(drift, std::hypot(r.sim.state().pos.x, r.sim.state().pos.y)); }
                return false;
            });
            fin(e);
            std::printf("  IMU №1: %-38s → основний IMU №%d, похибка орієнтації макс. %.1f°, знесення %.1f м%s\n", names[f], rig->nav.primaryIndex() + 1, e.att_max, drift,
                        rig->sim.crashed() ? ", АВАРІЯ" : "");
            CHECK(!rig->sim.crashed() && rig->nav.primaryIndex() == 1 && e.att_max < 10.0 && drift < 4.0, "відмова IMU (%s) має перемикати смугу без втрати керування", names[f]);
        }
    }
    {   // вібрації ×4 (≈ 35 м/с², «погано» за шкалою ArduPilot): висіння тримається, EKF збільшує шум акселерометра
        auto rig = makeRig(3, 45);
        rig->sensors.vib_scale = 4.0;
        fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig->setMission({tk});
        Err e; double drift = 0;
        rig->run(55, [&](const QpFlightRig& r) { track(r, e); if (r.sim.time() > 25) drift = std::fmax(drift, std::hypot(r.sim.state().pos.x, r.sim.state().pos.y)); return false; });
        fin(e);
        std::printf("  вібрації ×4: пік %.0f м/с², оцінка EKF %.1f м/с²; похибка нахилу RMS %.2f°, орієнтації з курсом %.2f°, знесення %.1f м\n", rig->sensors.vibrationPeak(), rig->nav.vibration(0), e.tilt_rms, e.att_rms, drift);
        CHECK(!rig->sim.crashed() && e.tilt_rms < 3.0 && drift < 3.0, "вібрації ×4: нахил %.2f°, знесення %.1f м", e.tilt_rms, drift);
    }
    {   // втрата GPS у висінні з EKF: аварійна вертикальна посадка на інерціальній навігації
        auto rig = makeRig(3, 47);
        fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig->setMission({tk});
        bool inj = false;
        rig->run(120, [&](const QpFlightRig& r) { if (!inj && r.sim.time() > 30) { rig->sfaults.gps_lost = true; inj = true; } return r.ctl.phase() == fc::QpPhase::Landed; });
        const double d = std::hypot(rig->sim.state().pos.x, rig->sim.state().pos.y);
        std::printf("  GPS втрачено у висінні (EKF без GPS): %s, знесення %.1f м\n", rig->ctl.phase() == fc::QpPhase::Landed ? "посадка" : "НЕ сів", d);
        CHECK(rig->ctl.phase() == fc::QpPhase::Landed && !rig->sim.crashed(), "втрата GPS з EKF має закінчуватись посадкою");
    }
}

static void testQuadPlaneFailsafes() {
    SECTION("Контролер QuadPlane: аварійні режими");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    {   // датчик швидкості «бреше» вже на переході вперед: тайм-аут → посадка, а не нескінченний політ
        SensorNoise bias; bias.airspeed_bias = -9.0;
        QpFlightRig rig(a, tb, 11);
        rig.noise = bias;
        rig.setMission(qpMission(900));
        rig.run(420, [](const QpFlightRig& r) { return r.ctl.phase() == fc::QpPhase::Landed; });
        std::printf("  швидкість -9 м/с з початку переходу (контроль достовірності Піто): фаза %s, висота %.1f м, відстань від дому %.0f м\n", rig.ctl.statusText(), rig.sim.state().pos.z, std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y));
        CHECK(rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed(), "тайм-аут переходу має закінчуватись посадкою");
    }
    {   // місія закінчується в польоті ({Takeoff, FlyTo}): повернення додому й посадка
        using S = fc::QpStep; S tk; tk.type = S::Takeoff; tk.p = {0, 0, 30}; S fl; fl.type = S::FlyTo; fl.p = {500, 0, 60};
        QpFlightRig rig(a, tb, 12);
        rig.setMission({tk, fl});
        rig.run(300, [](const QpFlightRig& r) { return r.ctl.phase() == fc::QpPhase::Landed; });
        std::printf("  місія {Takeoff, FlyTo}: фаза %s, посадка (%.1f, %.1f) м, час %.0f с\n", rig.ctl.statusText(), rig.sim.state().pos.x, rig.sim.state().pos.y, rig.sim.time());
        CHECK(rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed() && std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y) < 4.0, "місія без кінцевої посадки: апарат має сам повернутись і сісти");
    }
    {   // низька напруга шини: RTL і посадка (батарея заздалегідь розряджена)
        QpFlightRig rig(a, tb, 13);
        rig.setMission(qpMission(500));
        {   // з розрядженою батареєю автопілот не армується
            QpFlightRig g(a, tb, 13);
            g.setMission(qpMission(500));
            g.sim.battery().setSoc(0.07);
            g.run(15);
            std::printf("  заряд 7%% на землі: %s, «%s»\n", g.ctl.armed() ? "АРМОВАНО" : "не армується", fc::preflightText(g.ctl.preflightFailures()).c_str());
            CHECK(!g.ctl.armed() && (g.ctl.preflightFailures() & fc::kPfBattery), "з розрядженою батареєю зліт заборонено");
        }
        rig.run(20);
        rig.sim.battery().setSoc(0.07);   // батарея «сіла» в польоті
        rig.run(420, [](const QpFlightRig& r) { return r.ctl.phase() == fc::QpPhase::Landed; });
        std::printf("  заряд 7%% у польоті: фаза %s, висота %.1f м, відстань від дому %.0f м, SoC %.2f\n", rig.ctl.statusText(), rig.sim.state().pos.z, std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y), rig.sim.battery().soc());
        CHECK(!rig.sim.crashed() && rig.ctl.phase() == fc::QpPhase::Landed, "розряджена батарея: апарат має сісти сам, а не висіти до нуля");
    }
}

// ---- Монте-Карло QuadPlane: вітер, ЦМ ±15 мм, маса контролера ±8%, шуми датчиків ----
static void testQuadPlaneMonteCarlo(int runs) {
    SECTION("Монте-Карло конвертоплана: вітер 0–7 м/с, ЦМ ±15 мм, помилка маси ±8%, інерція ±20%, тяга моторів ±4%, приводи гондол −20…+10%, шум датчиків (половина — повний реальний ланцюжок), місія 600 м з висінням");
    const AircraftFile base = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    std::mt19937 rng(2026);
    prng::uniform<double> U(0, 1);
    auto u = [&](double a, double b) { return a + (b - a) * U(rng); };
    int ok = 0; double worst_err = 0, worst_roll = 0, max_wh = 0;
    const char* only = std::getenv("MC_ONLY");   // MC_ONLY=N — прогнати лише прогін N (для налагодження)
    for (int i = 0; i < runs; ++i) {
        AircraftFile f = base;
        const double dcg = u(-0.015, 0.015);                          // зсув ЦМ по X, м
        for (MassItem& it : f.items) if (it.name == "battery") it.x += dcg * f.num("targets.takeoff_mass_kg") / it.m;
        const double dm = u(-0.08, 0.08), wind = u(0, 7), wdir = u(0, 360);
        QpFlightRig rig(f, tb, 100 + i);
        rig.ctl.setMass(float(rig.sim.mass() * (1 + dm)), rig.ctl.cfg.af.inertia * float(u(0.8, 1.2)));   // контролер помиляється в масі й інерції (±20%)
        for (int m = 0; m < 4; ++m) { rig.sim.faults.vtol_scale[m] = u(0.96, 1.04); rig.sim.tiltServo(m).spec().rate *= u(0.8, 1.1); }   // модель ≠ апарат: тяга кожного мотора ±4%, швидкість приводів гондол
        rig.noise = SensorNoise::legacy();
        rig.noise.gyro_rad_s = u(0.002, 0.012); rig.noise.airspeed_mps = u(0.1, 0.4); rig.noise.pos_m = u(0.1, 0.5); rig.noise.vel_mps = u(0.03, 0.1); rig.noise.att_rad = u(0.001, 0.005);
        rig.noise.airspeed_bias = u(-0.5, 0.5);
        rig.noise.delay_att_s = u(0.01, 0.03); rig.noise.delay_gps_s = u(0.1, 0.2); rig.noise.gps_hz = 10;   // реалістичні затримки й GPS 10 Гц
        if (i % 2 == 1) { const double b = rig.noise.airspeed_bias; rig.noise = SensorNoise::realistic(); rig.noise.airspeed_bias = b; rig.sensors.vib_scale = u(0.5, 3.0); }   // непарні прогони — повний реальний ланцюжок (EKF, Піто, ESC, енкодери, затримка), вібрації ×0.5…3
        WindConfig w; w.mean20ft = Vec3d{wind * std::cos(wdir * 3.14159 / 180), wind * std::sin(wdir * 3.14159 / 180), 0}; w.turbulence = u(0.5, 1.2);
        rig.setWind(w, 200 + i);
        rig.setMission(qpMission(600));
        if (only && std::atoi(only) != i) continue;
        double roll_max = 0;
        int lp = -1;
        rig.run(700, [&](const QpFlightRig& r) {
            roll_max = std::fmax(roll_max, std::fabs(r.sim.eulerRad().x));
            if (only && std::getenv("MC_FINE") && r.ctl.phase() == fc::QpPhase::TransFwd && long(r.sim.time() * 10) % 20 == 0) {
                const fc::Eskf& L = r.nav.primary(); const Vec3d e = r.sim.eulerRad();
                std::printf("        t %.0f z %.1f (EKF %.1f) V %.1f pitch %.1f° Vz %.1f tilt %.0f° wv %.2f lim %.2f vib %.0f\n", r.sim.time(), r.sim.state().pos.z, r.nav.ready() ? L.pos().z : 0.0f,
                            r.sim.airspeed(), e.y * 57.3, r.sim.velWorld().z, r.sim.tilt(0) * 57.3, r.ctl.vtolWeight(), r.ctl.currentLimit(), r.sensors.vibrationPeak());
            }
            if (only && int(r.ctl.phase()) != lp) { lp = int(r.ctl.phase()); std::printf("      t %.0f: %s, x %.0f z %.0f V %.1f\n", r.sim.time(), r.ctl.statusText(), r.sim.state().pos.x, r.sim.state().pos.z, r.sim.airspeed()); }
            return r.ctl.phase() == fc::QpPhase::Landed; });
        const double err = std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y);
        const bool pass = rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed() && !rig.sim.diverged() && err < 6.0;
        ok += pass; worst_err = std::fmax(worst_err, err); worst_roll = std::fmax(worst_roll, roll_max); max_wh = std::fmax(max_wh, rig.sim.battery().usedWh());
        if (!pass) {
            std::printf("  ✗ run %d: вітер %.1f м/с з %.0f°, ЦМ %+.1f мм, маса %+.0f%%, фаза %s, відхилення %.1f м, аварія %d, EKF %d\n", i, wind, wdir, dcg * 1000, dm * 100, rig.ctl.statusText(), err, rig.sim.crashed(), int(rig.noise.use_ekf));
            for (const std::string& ev : rig.report.events) std::printf("      %s\n", ev.c_str());
        }
    }
    std::printf("  Успішних: %d/%d, найгірше відхилення при посадці %.1f м, макс. крен %.0f°, макс. витрата %.1f Вт·год\n", ok, runs, worst_err, worst_roll * 57.3, max_wh);
    CHECK(ok >= int(0.9 * runs), "Монте-Карло конвертоплана: успішних %d з %d (<90%%)", ok, runs);
}
static void testQuadPlaneManual() {
    SECTION("Ручне керування конвертоплана (коптер): зліт, політ, поворот, посадка");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    QpFlightRig rig(a, tb, 21);
    rig.ctl.manual.on = true;
    rig.run(3);   // на землі без команди підйому: мотори вимкнено
    const double z_idle = rig.sim.state().pos.z, z_rest = -a.nums("ground.gear_points")[2] + 0.01;   // стоїть на ніжках (висота ЦМ = висота шасі)
    CHECK(z_idle < z_rest && rig.sim.busCurrent() < 3.0, "на землі без команди мотори не крутяться: z %.2f, %.1f А", z_idle, rig.sim.busCurrent());
    rig.ctl.manual.climb = 1.0f; rig.run(12);
    const double z_up = rig.sim.state().pos.z;
    rig.ctl.manual.climb = 0; rig.ctl.manual.fwd = 1.0f; rig.run(10);
    const double x_fwd = rig.sim.state().pos.x, vx = rig.sim.velWorld().x;
    rig.ctl.manual.fwd = 0; rig.ctl.manual.left = 1.0f; rig.run(6);
    const double y_left = rig.sim.state().pos.y;
    rig.ctl.manual.left = 0; rig.ctl.manual.yaw_rate = 1.0f; rig.run(3); rig.ctl.manual.yaw_rate = 0;
    const double yaw_after = rig.sim.eulerRad().z;
    std::printf("  ручний політ: підйом до %.1f м, вперед %.0f м (V %.1f м/с), вліво %.0f м, курс після повороту %.0f°\n", z_up, x_fwd, vx, y_left, yaw_after * 57.3);
    CHECK(z_up > 18 && z_up < 45, "підйом з R: %.1f м", z_up);
    CHECK(x_fwd > 25 && vx > 3, "W рухає вперед: x %.1f, V %.1f", x_fwd, vx);
    CHECK(y_left > 15, "A рухає вліво (+Y): y %.1f", y_left);
    CHECK(std::fabs(yaw_after) > 0.5, "Q повертає: курс %.2f рад", yaw_after);
    rig.ctl.manual.climb = -0.35f; rig.ctl.manual.left = rig.ctl.manual.fwd = 0;
    rig.run(120, [](const QpFlightRig& r) { return r.sim.state().pos.z < 0.25; });
    CHECK(!rig.sim.crashed() && !rig.sim.diverged() && rig.sim.state().pos.z < 1.0, "посадка вручну без аварії: z %.2f", rig.sim.state().pos.z);
}
// ---- Відмови: що буде з апаратом (випробування за кресленням: до збирання знати наслідки) ----
struct FaultOutcome { bool landed, crashed, diverged; double x, y, z, t; };
static FaultOutcome flyWithFault(const AircraftFile& a, const AeroTables& tb, double fault_t, const std::function<void(QpFlightRig&)>& inject, bool takeoff_only = false, double limit = 420, double wind = 3) {
    QpFlightRig rig(a, tb, 31);
    rig.noise = SensorNoise::realistic();
    WindConfig w; w.mean20ft = Vec3d{wind, wind / 3, 0}; rig.setWind(w, 77);
    if (takeoff_only) { fc::QpStep tk; tk.type = fc::QpStep::Takeoff; tk.p = {0, 0, 30}; rig.setMission({tk}); } else rig.setMission(qpMission(600));
    bool done = false;
    rig.run(limit, [&](const QpFlightRig& r) {
        if (!done && r.sim.time() > fault_t) { inject(rig); done = true; }
        return r.ctl.phase() == fc::QpPhase::Landed;
    });
    const QpState& s = rig.sim.state();
    return {rig.ctl.phase() == fc::QpPhase::Landed, rig.sim.crashed(), rig.sim.diverged(), s.pos.x, s.pos.y, s.pos.z, rig.sim.time()};
}

// Повністю ручний політ (QSTABILIZE → QACRO → ручний розгін маршовим → FBWA → MANUAL → автопілот повертає й садить).
// «Пілот» — прості П-закони на ручках, як людина, що тримає висоту й горизонт.
static void testQuadPlaneManualFull() {
    SECTION("Повністю ручне керування: QSTABILIZE, QACRO, ручний перехід у літак, FBWA, MANUAL, повернення автопілоту");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    QpFlightRig rig(a, tb, 23);
    WindConfig w; w.mean20ft = Vec3d{3, 1, 0}; rig.setWind(w, 33);
    fc::QpManual& m = rig.ctl.manual;
    m.on = true; m.mode = fc::QpManual::QStabilize; m.throttle = 0;
    rig.run(2);
    CHECK(rig.sim.state().pos.z < -a.nums("ground.gear_points")[2] + 0.01 && rig.sim.busCurrent() < 3.0, "QSTABILIZE, газ 0: мотори не крутяться");
    double tilt_max = 0;
    auto pilotHover = [&](double z_t) {   // газ тримає висоту, ручки — горизонт (QSTABILIZE тримає кути сам)
        const double z = rig.sim.state().pos.z, vz = rig.sim.velWorld().z;
        m.throttle = float(std::clamp(0.27 + 0.05 * (z_t - z) - 0.08 * vz, 0.0, 0.8));
    };
    auto tilt = [&] { const Vec3d e = rig.sim.eulerRad(); return std::hypot(e.x, e.y) * 57.3; };
    for (int k = 0; k < 2500; ++k) { pilotHover(30); rig.run(0.01); if (rig.sim.time() > 15) tilt_max = std::fmax(tilt_max, tilt()); }
    const double z1 = rig.sim.state().pos.z;
    std::printf("  QSTABILIZE: висота %.1f м, макс. нахил у висінні %.1f°\n", z1, tilt_max);
    CHECK(std::fabs(z1 - 30) < 4 && tilt_max < 8, "QSTABILIZE: висіння на 30 м з ручним газом");
    m.mode = fc::QpManual::QAcro;   // кутові швидкості: «пілот» сам вирівнює горизонт
    double acro_tilt = 0;
    for (int k = 0; k < 800; ++k) {
        const Vec3d e = rig.sim.eulerRad();
        m.roll = float(std::clamp(-1.5 * e.x, -1.0, 1.0)); m.pitch = float(std::clamp(-1.5 * e.y, -1.0, 1.0));
        pilotHover(30); rig.run(0.01); acro_tilt = std::fmax(acro_tilt, tilt());
    }
    std::printf("  QACRO: макс. нахил %.1f°\n", acro_tilt);
    CHECK(!rig.sim.crashed() && acro_tilt < 15, "QACRO з пілотом, що вирівнює горизонт");
    m.roll = m.pitch = 0; m.mode = fc::QpManual::QStabilize; m.pusher = 0.85f;   // ручний перехід: «маршовий газ» нахиляє гондоли вперед
    double t0 = rig.sim.time();
    while (rig.sim.airspeed() < 14 && rig.sim.time() - t0 < 25) { pilotHover(30); rig.run(0.01); }
    const double v_tr = rig.sim.airspeed();
    m.mode = fc::QpManual::Fbwa; m.pusher = 0; m.throttle = 0.65f;   // літак: гондоли на 90°, газ — тяга вперед
    double zmin = 1e9, zmax = -1e9;
    t0 = rig.sim.time();
    while (rig.sim.time() - t0 < 25) {
        const double z = rig.sim.state().pos.z, vz = rig.sim.velWorld().z;
        m.pitch = float(std::clamp(-(0.08 * (35 - z) - 0.15 * vz), -1.0, 1.0));   // ручка на себе (−) — ніс угору
        m.roll = rig.sim.time() - t0 > 10 && rig.sim.time() - t0 < 16 ? 0.5f : 0.0f;   // віраж
        rig.run(0.01);
        if (rig.sim.time() - t0 > 5) { zmin = std::fmin(zmin, z); zmax = std::fmax(zmax, z); }
    }
    std::printf("  ручний перехід на %.1f м/с; FBWA: висота %.1f…%.1f м, V %.1f м/с, курс після віражу %.0f°\n", v_tr, zmin, zmax, rig.sim.airspeed(), rig.sim.eulerRad().z * 57.3);
    CHECK(v_tr >= 14 && !rig.sim.crashed() && zmin > 15 && zmax < 60 && rig.sim.airspeed() > 11, "FBWA: літак тримається в повітрі");
    m.mode = fc::QpManual::Manual;   // рулі напряму: пілот тримає тангаж і крен
    t0 = rig.sim.time(); double man_min = 1e9;
    while (rig.sim.time() - t0 < 10) {
        const Vec3d e = rig.sim.eulerRad(); const double z = rig.sim.state().pos.z;
        const double pitch_up_t = std::clamp(0.02 * (35 - z), -0.1, 0.15) + 0.03;
        m.pitch = float(std::clamp(2.0 * (-pitch_up_t - e.y) - 0.3 * rig.sim.state().omega.y, -1.0, 1.0));
        m.roll = float(std::clamp(-2.0 * e.x - 0.3 * rig.sim.state().omega.x, -1.0, 1.0));
        rig.run(0.01); man_min = std::fmin(man_min, z);
    }
    std::printf("  MANUAL (рулі напряму): мін. висота %.1f м, V %.1f м/с\n", man_min, rig.sim.airspeed());
    CHECK(!rig.sim.crashed() && man_min > 10, "MANUAL: керований політ без стабілізації");
    m.on = false;   // автопілот підхоплює: повернення додому й посадка
    rig.run(400, [](const QpFlightRig& r) { return r.ctl.phase() == fc::QpPhase::Landed; });
    const double d = std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y);
    std::printf("  автопілот після ручного польоту: %s, %.1f м від дому\n", rig.ctl.phase() == fc::QpPhase::Landed ? "посадка" : rig.ctl.statusText(), d);
    CHECK(rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed() && d < 5, "вимкнення ручного режиму в повітрі: повернення й посадка вдома");
}

static void testQuadPlaneFaults() {
    SECTION("Відмови конвертоплана: мотори, приводи гондол, рулі, GPS, труба Піто, батарея");
    const AircraftFile a = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    {   // LQR з обертанням: лінійна модель із замкненим контуром збігається для кожного мотора й швидкостей обертання 2…20 рад/с
        // геометрія, інерція й тяга — ті самі, що знає контролер цього апарата (не сталі з часів QuadPlane)
        const QpFlightRig geo(a, tb, 1);
        const fc::QpAirframe& af = geo.ctl.cfg.af;
        const fc::Vec3 I = af.inertia;
        const float dx = af.mixer.dx(), dy = af.mixer.dy(), w_hover = af.mass * 9.80665f;
        bool all_ok = true;
        for (int f = 0; f < 4; ++f) {
            fc::SpinController sc; sc.configure(f, dx, dy, I, af.mixer.max_thrust, 0.002f);
            CHECK(sc.partner() >= 0 && fc::kQuadX[sc.partner()].yaw_sign == fc::kQuadX[f].yaw_sign && sc.partner() != f, "партнер мотора %d має бути на тій самій діагоналі", f);
            for (float r : {2.0f, 6.0f, 12.0f, 20.0f}) {
                const float rr = r * sc.spinSign();
                // стан: p, q, n (бажаний напрям тяги в тілі); інтегруємо лінійну модель, тяги — з контролера
                float p = 0.0f, q = 0.0f, nx = 0.08f, ny = -0.05f;
                for (int k = 0; k < 4000; ++k) {
                    const fc::Vec3 n{nx, ny, std::sqrt(std::fmax(0.0f, 1 - nx * nx - ny * ny))};
                    // q = одиничний кватерніон → n у тілі дорівнює d_world
                    const fc::SpinController::Out o = sc.update(fc::Quat(), fc::Vec3{p, q, rr}, n, w_hover);
                    float tx = 0, ty = 0;
                    for (int i = 0; i < 4; ++i) { tx += o.thrust[i] * fc::kQuadX[i].sy * dy; ty -= o.thrust[i] * fc::kQuadX[i].sx * dx; }
                    const float dp = ((I.y - I.z) * rr * q + tx) / I.x, dq = ((I.z - I.x) * rr * p + ty) / I.y;
                    const float dnx = rr * ny - q, dny = -rr * nx + p;
                    p += dp * 0.002f; q += dq * 0.002f; nx += dnx * 0.002f; ny += dny * 0.002f;
                }
                if (!(std::hypot(nx, ny) < 0.01f && std::isfinite(p))) { all_ok = false; std::printf("  LQR: мотор %d, r=%.0f — не збігся (|n_xy| = %.3f)\n", f, rr, std::hypot(nx, ny)); }
            }
        }
        CHECK(all_ok, "LQR з обертанням має стабілізувати лінійну модель для всіх моторів");
    }
    struct Case { const char* name; double t; std::function<void(QpFlightRig&)> inj; bool takeoff_only; bool must_land_safe; double wind = 3; };
    const std::vector<Case> cases = {
        {"GPS зникло в крейсері (t=45 с)", 45, [](QpFlightRig& r) { r.sfaults.gps_lost = true; }, false, true},
        {"GPS зникло під час висіння (t=25 с)", 25, [](QpFlightRig& r) { r.sfaults.gps_lost = true; }, true, true},
        {"труба Піто мертва з початку польоту", 2, [](QpFlightRig& r) { r.sfaults.pitot_dead = true; }, false, true},
        {"слабка батарея (опір ×2, ємність ×0.8)", 2, [](QpFlightRig& r) { r.sim.faults.battery_r_scale = 2; r.sim.faults.battery_cap_scale = 0.8; }, false, true},
        {"дуже слабка батарея (опір ×3, ємність ×0.7) — взліт", 2, [](QpFlightRig& r) { r.sim.faults.battery_r_scale = 3; r.sim.faults.battery_cap_scale = 0.7; }, false, true},
        {"привід нахилу гондоли №1 заклинив у крейсері (90°, t=45 с)", 45, [](QpFlightRig& r) { r.sim.faults.tilt_stuck[0] = true; }, false, true},
        {"привід нахилу гондоли №2 заклинив вертикально (0°) перед місією", 2, [](QpFlightRig& r) { r.sim.faults.tilt_stuck[1] = true; }, false, true},
        // заклинений привід гондоли: виявлення за датчиком положення сервоприводу — переривання переходу й посадка коптером або посадка літаком
        // відмова підйомного мотора: виявлення за телеметрією обертів, далі посадка з обертанням (висіння) або літаком (крейсер) — docs/CONTROL.md
        {"підйомний мотор №1 відмовив у висінні (t=25 с)", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[0] = 0; }, true, true},
        {"підйомний мотор №2 відмовив у висінні (t=25 с)", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[1] = 0; }, true, true},
        {"підйомний мотор №3 відмовив у висінні (t=25 с)", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[2] = 0; }, true, true},
        {"підйомний мотор №4 відмовив у висінні (t=25 с)", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[3] = 0; }, true, true},
        {"підйомний мотор №3 відмовив на зльоті, 6 м (t=4 с)", 4, [](QpFlightRig& r) { r.sim.faults.vtol_scale[2] = 0; }, true, true},
        {"підйомний мотор №1 відмовив у крейсері (t=45 с)", 45, [](QpFlightRig& r) { r.sim.faults.vtol_scale[0] = 0; }, false, true},
        {"підйомний мотор №1 на 60% у висінні (t=25 с)", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[0] = 0.6; }, true, true},
        {"підйомний мотор №3 відмовив у висінні, вітер 6 м/с", 25, [](QpFlightRig& r) { r.sim.faults.vtol_scale[2] = 0; }, true, false, 6},
        {"елерон заклинило в крейсері (t=45 с)", 45, [](QpFlightRig& r) { r.sim.faults.servo_stuck[0] = true; }, false, false},
        {"кермо висоти заклинило в крейсері (t=45 с)", 45, [](QpFlightRig& r) { r.sim.faults.servo_stuck[1] = true; }, false, false},
    };
    int safe_ok = 0, safe_total = 0;
    // FAULT_FROM / FAULT_TO (номери випадків з 0) — прогнати частину списку, коли весь не вкладається в ліміт часу оболонки
    const size_t from = std::getenv("FAULT_FROM") ? size_t(std::atoi(std::getenv("FAULT_FROM"))) : 0, to = std::getenv("FAULT_TO") ? size_t(std::atoi(std::getenv("FAULT_TO"))) : cases.size();
    for (size_t ci = from; ci < std::min(to, cases.size()); ++ci) {
        const Case& c = cases[ci];
        const FaultOutcome o = flyWithFault(a, tb, c.t, c.inj, c.takeoff_only, 420, c.wind);
        const bool ok = o.landed && !o.crashed;
        std::printf("  %-60s → %s (z %.1f м, t %.0f с)\n", c.name, o.crashed ? "АВАРІЯ" : (o.landed ? "посадка" : "не сів за ліміт"), o.z, o.t);
        CHECK(!o.diverged, "модель розійшлась: %s", c.name);
        if (c.must_land_safe) { ++safe_total; safe_ok += ok; CHECK(ok, "відмова має закінчуватись безпечною посадкою: %s", c.name); }
    }
    std::printf("  Безпечно оброблено: %d з %d обов'язкових; інші випадки — інформативні (як апарат поводиться без резервування)\n", safe_ok, safe_total);
}
// Конвертоплан: що дає схема з поворотними гондолами й чим відрізняються варіанти задніх гвинтів
static void testTiltrotor() {
    SECTION("Конвертоплан: гондоли у висінні/переході/польоті, рискання нахилом, штовхальні проти тягнучих задніх, крейсер 4 чи 2 гвинтами");
    const AircraftFile base = loadQuadPlane();
    const AeroTables tb = loadAeroTables();
    const double kDeg = 3.14159265358979323846 / 180;
    auto variant = [&](const char* key, const char* val) { AircraftFile f = base; f.values[key] = {val}; return f; };
    struct Run { bool ok = false; double t = 0, wh = 0, hover_tilt = 0, cruise_tilt = 0, trans_fwd_s = 0, trans_back_s = 0, hover_w = 0; };
    auto fly = [&](const AircraftFile& f, double wind) {
        QpFlightRig rig(f, tb, 61);
        if (wind > 0) { WindConfig w; w.mean20ft = Vec3d{wind, wind / 3, 0}; rig.setWind(w, 71); }
        rig.setMission(qpMission(600));
        Run r; double hv_sum = 0, cr_sum = 0, hw = 0; long hv_n = 0, cr_n = 0;
        rig.run(600, [&](const QpFlightRig& g) {
            const fc::QpPhase ph = g.ctl.phase();
            double tm = 0; for (int i = 0; i < 4; ++i) tm += std::fabs(g.sim.tilt(i)) / 4;
            if (ph == fc::QpPhase::VtolHold && g.ctl.phaseTime() > 3) { hv_sum += tm; ++hv_n; hw += g.sim.busPower(); }
            if (ph == fc::QpPhase::Cruise && g.ctl.vtolWeight() == 0 && g.ctl.phaseTime() > 5) { cr_sum += tm; ++cr_n; }
            if (ph == fc::QpPhase::TransFwd) r.trans_fwd_s += 0.1;
            if (ph == fc::QpPhase::TransBack) r.trans_back_s += 0.1;
            return ph == fc::QpPhase::Landed;
        });
        r.ok = rig.ctl.phase() == fc::QpPhase::Landed && !rig.sim.crashed() && std::hypot(rig.sim.state().pos.x, rig.sim.state().pos.y) < 4;
        r.t = rig.sim.time(); r.wh = rig.sim.battery().usedWh();
        r.hover_tilt = hv_n ? hv_sum / hv_n : 0; r.cruise_tilt = cr_n ? cr_sum / cr_n : 0; r.hover_w = hv_n ? hw / hv_n : 0;
        return r;
    };
    // 1) розподіл: у висінні гондоли майже вертикальні, у польоті літаком — на 90°
    const Run p = fly(base, 0);
    std::printf("  штовхальні задні: місія %s за %.0f с, %.1f Вт·год; гондоли у висінні %.1f°, у польоті %.1f°; обидва переходи вперед %.0f с, назад %.0f с; висіння %.0f Вт\n",
                p.ok ? "виконана" : "НЕ виконана", p.t, p.wh, p.hover_tilt / kDeg, p.cruise_tilt / kDeg, p.trans_fwd_s, p.trans_back_s, p.hover_w);
    CHECK(p.ok, "місія конвертоплана (штовхальні задні)");
    CHECK(p.hover_tilt < 8 * kDeg && std::fabs(p.cruise_tilt - 90 * kDeg) < 3 * kDeg, "гондоли: висіння %.1f°, політ %.1f°", p.hover_tilt / kDeg, p.cruise_tilt / kDeg);
    CHECK(p.trans_fwd_s < 25, "два переходи вперед разом %.0f с", p.trans_fwd_s);
    // 2) тягнучі задні (rear_pusher = 0): та сама місія
    const Run tr = fly(variant("tilt.rear_pusher", "0"), 0);
    std::printf("  тягнучі задні:    місія %s за %.0f с, %.1f Вт·год; висіння %.0f Вт (штовхальні %+.0f%%)\n", tr.ok ? "виконана" : "НЕ виконана", tr.t, tr.wh, tr.hover_w,
                100 * (p.hover_w / std::fmax(tr.hover_w, 1) - 1));
    CHECK(tr.ok, "місія конвертоплана (тягнучі задні)");
    // Два протилежні ефекти: штовхальний гвинт у затіненні гондоли (−6% тяги) — дорожчий; але тягнучий задній гвинт стоїть НАД кінцем панелі V-хвоста
    // й дме на неї (притискна сила сліду), а штовхальний — під кінцем панелі, слід іде повз неї. У цій геометрії вони майже компенсуються.
    CHECK(std::fabs(p.hover_w / tr.hover_w - 1) < 0.05, "висіння: штовхальні %.0f Вт, тягнучі %.0f Вт", p.hover_w, tr.hover_w);
    {   // без сліду гвинтів на крилі/хвості лишається тільки затінення штовхального — він дорожчий
        QuadPlaneSim ps(variant("aero_model.rotor_wash_on_wing", "0"), tb);
        AircraftFile ft = variant("tilt.rear_pusher", "0"); ft.values["aero_model.rotor_wash_on_wing"] = {"0"};
        QuadPlaneSim ts(ft, tb);
        const double hp = hoverPower(ps).elec_w, ht = hoverPower(ts).elec_w;
        std::printf("  висіння без сліду гвинтів (лише затінення гондолою): штовхальні %.0f Вт, тягнучі %.0f Вт (%+.1f%%)\n", hp, ht, 100 * (hp / ht - 1));
        CHECK(hp > ht * 1.01 && hp < ht * 1.08, "затінення штовхального гвинта: %.0f проти %.0f Вт", hp, ht);
    }
    // 3) крейсер: 4 гвинти на малому навантаженні проти лише передніх (задні стоять — опір зупинених лопатей).
    //    Типово (cruise_rear = 2) у рівному польоті тягнуть передні, задні вмикаються на допомогу при наборі й розгоні.
    {
        const AircraftFile f4 = variant("tilt.cruise_rear", "1"), f2 = variant("tilt.cruise_rear", "0");
        QuadPlaneSim s4(f4, tb), s2(f2, tb);
        const TrimResult c4 = trimCruise(s4, 15), c2 = trimCruise(s2, 15);
        std::printf("  крейсер 15 м/с: усі 4 гвинти %.0f Вт (%.0f об/хв, duty %.2f), лише передні %.0f Вт (%.0f об/хв, duty %.2f) → економія %.0f%%\n",
                    c4.elec_w, c4.cruise_rps * 60, c4.duty, c2.elec_w, c2.cruise_rps * 60, c2.duty, 100 * (1 - c2.elec_w / c4.elec_w));
        CHECK(c4.ok && c2.ok && c2.elec_w < 0.85 * c4.elec_w, "крейсер передніми гвинтами має бути економнішим: %.0f проти %.0f Вт", c2.elec_w, c4.elec_w);
        const Run all4 = fly(f4, 0);
        std::printf("  місія, у польоті тягнуть усі 4: %s, %.1f Вт·год; типово (передні + задні на допомогу): %.1f Вт·год\n", all4.ok ? "виконана" : "НЕ виконана", all4.wh, p.wh);
        // місія 600 м здебільшого висіння й переходи — різниця в енергії мала; на довгому польоті вона ≈ різниці потужностей крейсеру.
        // З гвинтами в косому потоці (елемент лопаті) «усі 4» у переходах трохи вигідніші — на короткій місії допуск 5%, а крейсер передніми
        // однаково на ≈ 25% економніший (перевірка вище)
        CHECK(all4.ok && p.wh < all4.wh * 1.05, "передні + задні на допомогу не набагато гірші за «усі 4»: %.1f проти %.1f Вт·год", p.wh, all4.wh);
    }
    // 4) рискання у висінні: різницевий нахил гондол проти лише реактивного моменту гвинтів.
    //    Статичний запас: момент рискання при нахилі ±12° (ліві вперед, праві назад) проти найбільшої різниці тяг діагоналей (від холостих до вдвічі більшої)
    {
        QuadPlaneSim s(base, tb);
        s.place(Vec3d{0, 0, 100}, Vec3d{}, 0, 0, 0);
        const TrimResult h = hoverPower(s);
        const double rps[4] = {h.vtol_rps_front, h.vtol_rps_rear, h.vtol_rps_front, h.vtol_rps_rear};
        s.presetDrive(0, 0, 0, 0, rps, 0, 0);
        Vec3d F0, M0; s.forcesAndMoments(s.state(), F0, M0);
        double tl[4];
        for (int i = 0; i < 4; ++i) tl[i] = (s.pivot(i).y > 0 ? -1 : 1) * 12 * kDeg;   // +Mz (ніс ліворуч) — ліві назад, праві вперед
        s.presetTilts(tl);
        Vec3d F1, M1; s.forcesAndMoments(s.state(), F1, M1);
        s.presetTilt(0);
        const double q = s.vtolProp().eval(h.vtol_rps_front, 0, 0, 1.225).torque;   // реактивний момент одного гвинта
        const double m_react = 2 * q;   // одна діагональ удвічі, інша на холостих — грубо ±q на кожен з 4 моторів
        std::printf("  запас рискання у висінні: нахил гондол ±12° → %.2f Н·м, реактивний момент гвинтів ≈ %.2f Н·м (у %.1f раза менше)\n", M1.z - M0.z, m_react, (M1.z - M0.z) / m_react);
        CHECK(M1.z - M0.z > 2 * m_react, "векторне рискання сильніше за реактивне: %.2f проти %.2f Н·м", M1.z - M0.z, m_react);
    }
    auto yawTurn = [&](const AircraftFile& f) {   // асистований ручний режим: курс на 180° на повній ручці, час до 170°
        QpFlightRig rig(f, tb, 63);
        rig.ctl.manual.on = true; rig.ctl.manual.climb = 1.0f; rig.run(12); rig.ctl.manual.climb = 0; rig.run(4);
        const double y0 = rig.sim.eulerRad().z, t0 = rig.sim.time();
        rig.ctl.manual.yaw_rate = 1.0f;
        double t170 = -1, rmax = 0;
        rig.run(8, [&](const QpFlightRig& g) {
            rmax = std::fmax(rmax, std::fabs(g.sim.state().omega.z));
            const double d = std::fabs(std::remainder(g.sim.eulerRad().z - y0, 2 * 3.14159265358979323846));
            if (t170 < 0 && d > 170 * kDeg) t170 = g.sim.time() - t0;
            return t170 > 0;
        });
        return std::pair<double, double>{t170, rmax};
    };
    const auto yv = yawTurn(base), yr = yawTurn(variant("tilt.yaw_vector", "0"));
    std::printf("  розворот на 170° у висінні: нахилом гондол %.1f с (макс. %.0f°/с), лише реактивним моментом %.1f с (%.0f°/с)\n", yv.first, yv.second / kDeg, yr.first, yr.second / kDeg);
    CHECK(yv.first > 0 && yr.first > 0 && yv.first <= yr.first * 1.05, "рискання нахилом гондол не повільніше: %.1f проти %.1f с", yv.first, yr.first);
    // 5) вітер 8 м/с: висіння носом проти вітру з тягою нахилом гондол, місія повністю
    const Run w = fly(base, 8);
    std::printf("  вітер 8 м/с: місія %s за %.0f с, %.1f Вт·год, гондоли у висінні в середньому %.1f°\n", w.ok ? "виконана" : "НЕ виконана", w.t, w.wh, w.hover_tilt / kDeg);
    CHECK(w.ok, "місія конвертоплана у вітрі 8 м/с");
}

// Паралельний запуск: `run_tests 12 -j 8` — кожна група окремим процесом, до N одночасно (0 — усі ядра).
// Вивід груп — у tests_out/<група>.txt, на екран — по черзі груп; тривалість кожної запам'ятовується (tests_out/times.txt),
// і наступного разу найдовші стартують першими.
template <class G>
static int runParallel(const char* self, int mc, int jobs, const G& groups) {
    namespace fs = std::filesystem;
    fs::create_directories("tests_out");
    if (jobs <= 0) jobs = int(std::max(1u, std::thread::hardware_concurrency()));
    std::map<std::string, double> dur;
    { std::ifstream f("tests_out/times.txt"); std::string n; double t; while (f >> n >> t) dur[n] = t; }
    std::vector<std::string> order;
    for (const auto& g : groups) order.push_back(g.first);
    std::stable_sort(order.begin(), order.end(), [&](const std::string& a, const std::string& b) { return dur[a] > dur[b]; });
    std::atomic<size_t> next{0};
    std::mutex mu;
    int done = 0;
    const auto t0 = std::chrono::steady_clock::now();
    auto worker = [&] {
        for (;;) {
            const size_t i = next++;
            if (i >= order.size()) return;
            const std::string& n = order[i];
            const auto ts = std::chrono::steady_clock::now();
            const std::string cmd = std::string("\"\"") + self + "\" " + std::to_string(mc) + " =" + n + " > \"tests_out/" + n + ".txt\" 2>&1\"";
#ifdef _WIN32
            std::system(cmd.c_str());
#else
            std::system(cmd.substr(1, cmd.size() - 2).c_str());
#endif
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - ts).count();
            std::lock_guard<std::mutex> lk(mu);
            dur[n] = t; ++done;
            std::printf("[%d/%zu] %s — %.0f с\n", done, order.size(), n.c_str(), t);
            std::fflush(stdout);
        }
    };
    std::vector<std::thread> th;
    for (int k = 0; k < jobs; ++k) th.emplace_back(worker);
    for (auto& x : th) x.join();
    int pass = 0, fail = 0;
    for (const auto& g : groups) {
        std::ifstream f(std::string("tests_out/") + g.first + ".txt");
        std::string line, last;
        bool any = false;
        while (std::getline(f, line)) {
            if (line.find("ПІДСУМОК") != std::string::npos) { last = line; continue; }
            if (!line.empty() || any) { std::printf("%s\n", line.c_str()); any = true; }
        }
        int p = 0, q = 0;
        const size_t k = last.find("ПІДСУМОК: ");
        if (k == std::string::npos) { std::printf("  FAIL група %s не завершилась (аварія процесу?)\n", g.first); ++fail; continue; }
        std::sscanf(last.c_str() + k + std::strlen("ПІДСУМОК: "), "%d", &p);
        const size_t c = last.find(", ");
        if (c != std::string::npos) std::sscanf(last.c_str() + c + 2, "%d", &q);
        pass += p; fail += q;
    }
    { std::ofstream f("tests_out/times.txt"); for (const auto& d : dur) f << d.first << " " << d.second << "\n"; }
    std::printf("\n==== ПІДСУМОК: %d пройдено, %d провалено (паралельно на %d ядрах, %.0f с) ====\n", pass, fail, jobs,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    return fail ? 1 : 0;
}

int main(int argc, char** argv) {
    enableUtf8Console();
    int mc_runs = argc > 1 ? std::atoi(argv[1]) : 100;
    // другий аргумент — фільтр: запускаються лише групи, у назві яких є цей рядок (напр. `run_tests.exe 100 QuadPlane`)
    const std::string filter = argc > 2 ? argv[2] : "";
    const std::vector<std::pair<const char*, std::function<void()>>> groups = {
        {"testMath", [&] { testMath(); }},
        {"testMixer", [&] { testMixer(); }},
        {"testPid", [&] { testPid(); }},
        {"testDrivers", [&] { testDrivers(); }},
        {"testEstimators", [&] { testEstimators(); }},
        {"testArming", [&] { testArming(); }},
        {"testScenarios", [&] { testScenarios(); }},
        {"testTracking", [&] { testTracking(); }},
        {"testFailsafe", [&] { testFailsafe(); }},
        {"testGps", [&] { testGps(); }},
        {"testAltitude", [&] { testAltitude(); }},
        {"testSport", [&] { testSport(); }},
        {"testNoGps", [&] { testNoGps(); }},
        {"testBattery", [&] { testBattery(); }},
        {"testEco", [&] { testEco(); }},
        {"testEnergy", [&] { testEnergy(); }},
        {"testWeather", [&] { testWeather(); }},
        {"testAirframe", [&] { testAirframe(); }},
        {"testPhysicsRealism", [&] { testPhysicsRealism(); }},
        {"testAircraftData", [&] { testAircraftData(); }},
        {"testAeroTables", [&] { testAeroTables(); }},
        {"testQuadPlaneAero", [&] { testQuadPlaneAero(); }},
        {"testQuadPlaneAeroEffects", [&] { testQuadPlaneAeroEffects(); }},
        {"testQuadPlanePower", [&] { testQuadPlanePower(); }},
        {"testQuadPlaneSim", [&] { testQuadPlaneSim(); }},
        {"testQuadPlaneAnalysis", [&] { testQuadPlaneAnalysis(); }},
        {"testQuadPlaneController", [&] { testQuadPlaneController(); }},
        {"testQuadPlaneFailsafes", [&] { testQuadPlaneFailsafes(); }},
        {"testQuadPlaneStrongWind", [&] { testQuadPlaneStrongWind(); }},
        {"testTiltrotor", [&] { testTiltrotor(); }},
        {"testPhysicsDetail", [&] { testPhysicsDetail(); }},
        {"testAi", [&] { testAi(); }},
        {"testTricks", [&] { testTricks(); }},
        {"testPayload", [&] { testPayload(); }},
        {"testRealSensors", [&] { testRealSensors(); }},
        {"testSafety", [&] { testSafety(); }},
        {"testHealth", [&] { testHealth(); }},
        {"testFrames", [&] { testFrames(); }},
        {"testEnergyAi", [&] { testEnergyAi(); }},
        {"testRoute", [&] { testRoute(); }},
        {"testPlannerTuneAnomaly", [&] { testPlannerTuneAnomaly(); }},
        {"testStructureRotors", [&] { testStructureRotors(); }},
        {"testWingHover", [&] { testWingHover(); }},
        {"testQp5", [&] { testQp5(); }},
        {"testQuadPlaneEkf", [&] { testQuadPlaneEkf(); }},
        {"testQuadPlaneManual", [&] { testQuadPlaneManual(); }},
        {"testQuadPlaneManualFull", [&] { testQuadPlaneManualFull(); }},
        {"testQuadPlaneFaults", [&] { testQuadPlaneFaults(); }},
        {"testQuadPlaneMonteCarlo", [&] { testQuadPlaneMonteCarlo(std::min(mc_runs, 12)); }},
        {"testMonteCarlo", [&] { testMonteCarlo(mc_runs); }},
    };
    if (filter == "-j") return runParallel(argv[0], mc_runs, argc > 3 ? std::atoi(argv[3]) : 0, groups);
    for (const auto& g : groups)
        if (filter.empty() || (filter[0] == '=' ? filter.substr(1) == g.first : std::string(g.first).find(filter) != std::string::npos)) g.second();
    std::printf("\n==== ПІДСУМОК: %d пройдено, %d провалено ====\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
