// sensors_sim.h — віртуальні сенсори, які імітують реальне залізо:
//   SimRawImuBus  — цифровий IMU (сирі int16 як у MPU6050), шум, дрейф, вібрації, квантування, насичення
//   SimAdc        — аналогові сенсори через 12-бітний АЦП
//   SimBaro       — барометр 50 Гц з шумом і повільним дрейфом
//   SimMotors     — приймач команд моторів
// + інжекція збоїв (відвал IMU на проміжок часу).
#pragma once
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include "../fc/hal.h"
#include "../fc/power.h"
#include "quad_sim.h"
#include "portable_rng.h"

namespace sim {

struct ImuErrors {
    float gyro_noise_dps = 0.15f;     // СКВ шуму на відлік
    Vec3 gyro_bias_dps{0.5f, -0.3f, 0.4f};
    float accel_noise_g = 0.01f;
    Vec3 accel_bias_g{0.01f, -0.01f, 0.02f};
    float vibration_g = 0.05f;        // вібрації від моторів (пропорційні тязі)
    float vibration_gyro_dps = 0.5f;
    float gyro_bias_walk = 0.005f;   // °/с/√с — повільний дрейф нуля гіроскопа (bias instability)
};

struct FaultWindow { float t0 = -1, t1 = -1; bool active(float t) const { return t >= t0 && t < t1; } };

class ImuTruth {
public:
    ImuTruth(const QuadSim& q, const ImuErrors& e, unsigned seed) : q_(q), e_(e), rng_(seed), rng_bias_(seed * 7919u + 13u) {}
    // Повертає «виміряні» значення у фізичних одиницях (°/с і g)
    void measure(Vec3& gyro_dps, Vec3& accel_g) {
        prng::normal<float> n(0, 1);
        {   // випадкове блукання нуля гіроскопа
            const float dtm = last_t_ < 0 ? 0.0f : fc::clampf(q_.t - last_t_, 0.0f, 0.1f);
            last_t_ = q_.t;
            for (int i = 0; i < 3; ++i) e_.gyro_bias_dps[i] += e_.gyro_bias_walk * std::sqrt(dtm) * n(rng_bias_);
        }
        float load = (q_.thrust[0] + q_.thrust[1] + q_.thrust[2] + q_.thrust[3]) / (4 * q_.p.max_thrust);
        Vec3 w = q_.s.omega * fc::kRad2Deg;
        Vec3 a = q_.specific_force_body / fc::kGravity;
        for (int i = 0; i < 3; ++i) {
            gyro_dps[i] = w[i] + e_.gyro_bias_dps[i] + e_.gyro_noise_dps * n(rng_) + e_.vibration_gyro_dps * load * n(rng_);
            accel_g[i] = a[i] + e_.accel_bias_g[i] + e_.accel_noise_g * n(rng_) + e_.vibration_g * load * n(rng_);
        }
    }
    ImuErrors& errors() { return e_; }
private:
    const QuadSim& q_;
    ImuErrors e_;
    std::mt19937 rng_, rng_bias_;   // дрейф нуля — окремий генератор, щоб не змінювати решту шумів
    float last_t_ = -1;
};

// ---- Цифровий IMU ----
class SimRawImuBus : public fc::IRawImuBus {
public:
    fc::DigitalImu::Config cfg;
    FaultWindow fault;
    SimRawImuBus(const QuadSim& q, const ImuErrors& e, unsigned seed) : q_(q), truth_(q, e, seed) {}
    bool readRaw(int16_t g[3], int16_t a[3]) override {
        if (fault.active(q_.t)) return false;
        Vec3 gd, ag;
        truth_.measure(gd, ag);
        for (int i = 0; i < 3; ++i) {
            g[i] = sat16(gd[i] * cfg.gyro_lsb_per_dps);  // квантування + насичення діапазону
            a[i] = sat16(ag[i] * cfg.accel_lsb_per_g);
        }
        return true;
    }
    ImuErrors& errors() { return truth_.errors(); }
private:
    static int16_t sat16(float v) { v = std::round(v); return int16_t(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); }
    const QuadSim& q_;
    ImuTruth truth_;
};

// ---- Аналоговий IMU через АЦП ----
class SimAdc : public fc::IAdc {
public:
    fc::AnalogImu::Config cfg;
    FaultWindow fault;
    float adc_noise_lsb = 1.5f;
    SimAdc(const QuadSim& q, const ImuErrors& e, unsigned seed) : q_(q), truth_(q, e, seed), rng_(seed + 7) {}
    bool sample(uint16_t* c, int n) override {
        if (fault.active(q_.t) || n < 6) return false;
        Vec3 gd, ag;
        truth_.measure(gd, ag);
        prng::normal<float> nn(0, adc_noise_lsb);
        const float maxc = float((1 << cfg.adc_bits) - 1);
        const float per_v = maxc / cfg.vref;
        for (int i = 0; i < 3; ++i) {
            float vg = cfg.gyro_zero_v + gd[i] * cfg.gyro_v_per_dps;
            float va = cfg.accel_zero_v + ag[i] * cfg.accel_v_per_g;
            c[cfg.ch_gyro[i]] = toCounts(vg * per_v + nn(rng_), maxc);
            c[cfg.ch_accel[i]] = toCounts(va * per_v + nn(rng_), maxc);
        }
        return true;
    }
    ImuErrors& errors() { return truth_.errors(); }
private:
    static uint16_t toCounts(float v, float maxc) { v = std::round(v); return uint16_t(v < 0 ? 0 : (v > maxc ? maxc : v)); }
    const QuadSim& q_;
    ImuTruth truth_;
    std::mt19937 rng_;
};

// ---- Барометр ----
class SimBaro : public fc::IBaro {
public:
    float rate_hz = 50, noise_m = 0.08f, drift_m_per_s = 0.002f, offset_m = 112.0f;  // offset: висота над морем
    FaultWindow fault;
    SimBaro(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    bool read(float& alt) override {
        if (q_.t + 1e-6f < next_t_ || fault.active(q_.t)) return false;
        next_t_ = q_.t + 1.0f / rate_hz;
        prng::normal<float> n(0, noise_m);
        alt = offset_m + q_.s.pos.z + drift_m_per_s * q_.t + n(rng_);
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
    float next_t_ = 0;
};

// ---- GPS: 10 Гц, затримка 0.1 с, шум + повільний «блукаючий» зсув (як у реальних приймачів) ----
class SimGps : public fc::IGps {
public:
    double home_lat = 51.2465, home_lon = 22.5684;   // точка старту (Люблін)
    float rate_hz = 10, delay_s = 0.1f;
    float noise_m = 0.10f, walk_m = 0.6f, walk_tau = 20.0f, vel_noise = 0.05f;  // стрибки між відліками малі, «блукання» — повільне
    int sats = 14;
    float h_acc = 0.9f;
    FaultWindow fault;   // повна втрата сигналу
    SimGps(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    bool read(fc::GpsFix& out) override {
        hist_.push_back({q_.t, q_.s.pos, q_.s.vel});
        while (hist_.size() > 2 && hist_[1].t <= q_.t - delay_s) hist_.erase(hist_.begin());
        if (q_.t < 0.5f || q_.t + 1e-6f < next_t_) return false;
        float dt = q_.t - last_t_;
        next_t_ = q_.t + 1.0f / rate_hz;
        last_t_ = q_.t;
        if (fault.active(q_.t)) return false;
        prng::normal<float> n(0, 1);
        float a = std::fmin(dt / walk_tau, 1.0f);
        walk_ = walk_ * (1 - a) + Vec3{n(rng_), n(rng_), 0} * (walk_m * std::sqrt(2 * a));
        const Snap& s = hist_.front();  // відлік із затримкою
        Vec3 p = s.pos + walk_ + Vec3{n(rng_), n(rng_), 0} * noise_m;
        fc::GeoRef g;
        g.setHome(home_lat, home_lon);
        g.toGeo(p, out.lat_deg, out.lon_deg);
        out.alt_m = s.pos.z + 112.0f + 0.8f * n(rng_);
        out.vel = s.vel + Vec3{n(rng_), n(rng_), n(rng_)} * vel_noise;
        out.sats = sats;
        out.h_acc = h_acc;
        out.fix3d = true;
        return true;
    }
private:
    struct Snap { float t; Vec3 pos, vel; };
    const QuadSim& q_;
    std::mt19937 rng_;
    std::vector<Snap> hist_;
    Vec3 walk_{0, 0, 0};
    float next_t_ = 0, last_t_ = 0;
};

// ---- Магнітометр: поле Землі в системі тіла + шум + наводка від струму моторів ----
class SimMag : public fc::IMag {
public:
    Vec3 field_world{0.0f, 0.20f, -0.45f};  // гаус: північ (+Y) і вниз
    float noise = 0.004f, motor_interference = 0.01f;
    FaultWindow fault;
    SimMag(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    bool read(Vec3& m) override {
        if (fault.active(q_.t)) return false;
        prng::normal<float> n(0, noise);
        float load = (q_.thrust[0] + q_.thrust[1] + q_.thrust[2] + q_.thrust[3]) / (4 * q_.p.max_thrust);
        m = q_.s.q.rotateInv(field_world) + Vec3{n(rng_), n(rng_), n(rng_)} + Vec3{0.3f, 0.2f, 1.0f} * (motor_interference * load);
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
};

// ---- Оптичний потік (камера вниз), 50 Гц ----
// Шум кутовий (рад/с), тож похибка швидкості росте з висотою: σ_v ≈ σ_rate · висота.
// Якість падає над одноманітною поверхнею (вода, сніг), у темряві та вище max_height.
class SimFlow : public fc::IFlow {
public:
    float rate_hz = 50, noise_rad_s = 0.004f, scale_error = 0.02f;
    float max_height = 200;     // м: вище текстура «розпливається» (камерна одометрія на бортовому комп'ютері)
    float texture = 1.0f;       // 1 — трава/поле, 0.2 — вода/сніг
    FaultWindow fault;
    SimFlow(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    void setPowered(bool on) { if (on && !powered_) boot_until_ = q_.t + 0.3f; powered_ = on; }  // після ввімкнення — 0.3 с запуск
    bool powered() const { return powered_; }
    bool read(fc::FlowSample& out) override {
        if (q_.t + 1e-6f < next_t_) return false;
        next_t_ = q_.t + 1.0f / rate_hz;
        if (fault.active(q_.t) || !powered_ || q_.t < boot_until_) return false;
        float ct = q_.s.q.cosTilt();
        float h = q_.s.pos.z;
        if (h < 0.05f || ct < 0.3f) { out = fc::FlowSample{}; return true; }
        float d = h / ct;
        Vec3 vb = q_.s.q.rotateInv(q_.s.vel);
        Vec3 w = q_.s.omega;
        prng::normal<float> n(0, noise_rad_s * (1.5f - 0.5f * texture));
        out.rate_x = vb.x / d * (1 + scale_error) - w.y + n(rng_);
        out.rate_y = vb.y / d * (1 + scale_error) + w.x + n(rng_);
        out.quality = fc::clampf(texture * (h < max_height ? 1.0f : 1.0f - (h - max_height) / 50.0f), 0, 1);
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
    float next_t_ = 0, boot_until_ = 0;
    bool powered_ = true;
};

// ---- Лазерний далекомір вниз, 50 Гц (TF02-Pro: до 40 м) ----
class SimRangefinder : public fc::IRangefinder {
public:
    float rate_hz = 50, max_range = 40, noise = 0.02f;
    FaultWindow fault;
    SimRangefinder(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    void setPowered(bool on) { if (on && !powered_) boot_until_ = q_.t + 0.2f; powered_ = on; }
    bool powered() const { return powered_; }
    bool read(float& d) override {
        if (q_.t + 1e-6f < next_t_) return false;
        next_t_ = q_.t + 1.0f / rate_hz;
        if (fault.active(q_.t) || !powered_ || q_.t < boot_until_) return false;
        float ct = q_.s.q.cosTilt();
        if (ct < 0.5f) return false;
        float dist = q_.s.pos.z / ct;
        if (dist > max_range || dist < 0.1f) return false;
        prng::normal<float> n(0, noise + 0.005f * dist);
        d = dist + n(rng_);
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
    float next_t_ = 0, boot_until_ = 0;
    bool powered_ = true;
};

// ---- Модуль живлення: напруга й струм, 20 Гц ----
class SimBattery : public fc::IBattery {
public:
    float rate_hz = 20, v_noise = 0.03f, a_noise = 0.3f;
    SimBattery(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    bool read(float& v, float& a) override {
        if (q_.t + 1e-6f < next_t_) return false;
        next_t_ = q_.t + 1.0f / rate_hz;
        prng::normal<float> n(0, 1);
        v = q_.volts + v_noise * n(rng_);
        a = std::fmax(0.0f, q_.amps + a_noise * n(rng_));
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
    float next_t_ = 0;
};

// ---- Ключі живлення периферії: вимикають сенсори і рахують споживання електроніки ----
class SimPower : public fc::IPowerSwitch {
public:
    fc::AvionicsSpec spec;
    fc::PowerState state;
    SimPower(QuadSim& q, SimFlow& f, SimRangefinder& r) : q_(q), f_(f), r_(r) { q_.bat.avionics_w = fc::avionicsWatts(state, spec); }
    void apply(const fc::PowerState& s) override {
        state = s;
        f_.setPowered(s.on[fc::PD_FLOW]);
        r_.setPowered(s.on[fc::PD_RANGE]);
        q_.bat.avionics_w = fc::avionicsWatts(s, spec);
    }
private:
    QuadSim& q_;
    SimFlow& f_;
    SimRangefinder& r_;
};

class SimMotors : public fc::IMotorOutput {
public:
    float cmd[4] = {0, 0, 0, 0};
    float aux = 0;
    explicit SimMotors(QuadSim& q) : q_(q) {}
    void write(const float c[4]) override { for (int i = 0; i < 4; ++i) cmd[i] = c[i]; }
    void writeAux(float a) override { aux = a; q_.pusher_cmd = a; }
    void writeSurfaces(float a, float e, float r) override { q_.surf_cmd = Vec3{a, e, r}; }
private:
    QuadSim& q_;
};

// ---- Датчик швидкості повітря (трубка Піто), 20 Гц ----
// Міряє динамічний тиск q = ρV²/2 з шумом ~1.5 Па, тож на малій швидкості (< 3 м/с) показує майже шум —
// як справжній: трубка Піто погано бачить слабкий вітер.
class SimAirspeed : public fc::IAirspeed {
public:
    float rate_hz = 20, noise_pa = 1.5f;
    SimAirspeed(const QuadSim& q, unsigned seed) : q_(q), rng_(seed) {}
    bool read(float& ias) override {
        if (q_.t + 1e-6f < next_t_) return false;
        next_t_ = q_.t + 1.0f / rate_hz;
        float u = q_.s.q.rotateInv(q_.s.vel - q_.wind_now).x;   // потік у трубку спереду
        float qd = 0.5f * fc::kRho * u * std::fabs(u);
        qd += prng::normal<float>(0, noise_pa)(rng_);
        ias = std::sqrt(std::fmax(qd, 0.0f) * 2 / fc::kRho);
        return true;
    }
private:
    const QuadSim& q_;
    std::mt19937 rng_;
    float next_t_ = 0;
};

}  // namespace sim
