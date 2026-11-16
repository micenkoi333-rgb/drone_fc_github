// qp_sensors.h — сирі датчики QuadPlane для EKF: два IMU (гіро + акселерометр), барометр, магнітометр, GPS.
// IMU: питома сила f = Rᵀ(a − g) з похідної швидкості моделі (IMU у ЦМ), біас при ввімкненні + випадкове блукання, білий шум,
//   вібрації від дисбалансу гвинтів (амплітуда ∝ (n/n_висіння)², частота — оберти; окремі фази й напрямки на кожен мотор),
//   насичення ±16 g / ±2000 °/с. Магнітометр: поле Землі (Польща ≈ 50 мкТл, нахил 67°, схилення +6°) + наведення від струму моторів.
//   GPS: 10 Гц, затримка 150 мс, біла похибка + повільний дрейф (Гаусс–Марков, τ = 60 с). Барометр: 50 Гц, шум + дрейф.
// Числа — типові для MEMS-IMU класу ICM-42688/BMI088, u-blox M8/M9, BMP388 [О]. Відмови IMU: стрибок біасу гіро/акселерометра, «завис», мертвий.
#pragma once
#include <array>
#include <cmath>
#include <deque>
#include <random>
#include "../fc/qp_ekf.h"
#include "qp_sim.h"
#include "portable_rng.h"

namespace sim {

struct SensorSuiteConfig {
    double gyro_noise = 0.003, accel_noise = 0.05;            // σ на відлік 1 кГц: рад/с, м/с²
    double gyro_bias0 = 0.005, accel_bias0 = 0.15;            // σ біасу при ввімкненні
    double gyro_bias_rw = 1e-4, accel_bias_rw = 1e-3;         // /√с
    double accel_range = 16 * 9.80665, gyro_range = 34.9;     // насичення
    double vib_accel = 3.0, vib_gyro = 0.02;                  // амплітуда вібрацій на обертах висіння: м/с², рад/с
    double hover_rps = 115;                                   // оберти висіння для масштабу вібрацій
    double baro_noise = 0.15, baro_drift_rw = 0.01;
    double mag_noise = 0.3, mag_field = 50, mag_incl_deg = 67, mag_decl_deg = 6, mag_motor_ut_per_a = 0.03;   // компас на щоглі; без щогли ≈ 0.1 (похибка курсу 5–10°)
    double gps_vel_bias = 0.02, gps_vel_bias_tau = 30;   // м/с: повільний зсув доплерівської швидкості (багатопроменевість, зміна сузір'я)
    double gps_noise = 0.3, gps_walk = 0.7, gps_walk_tau = 60, gps_vel_noise = 0.08, gps_delay = 0.15, gps_hz = 10;
};

enum class ImuFault { None, GyroBiasJump, AccelBiasJump, Stuck, Dead };

class SensorSuite {
public:
    SensorSuiteConfig cfg;
    ImuFault imu_fault[2] = {ImuFault::None, ImuFault::None};
    double vib_scale = 1.0;            // ×: пошкоджений гвинт або погане демпфування
    std::array<double, 5> motor_vib{{1, 1, 1, 1, 1}};   // × вібрацій окремого мотора: розбалансований гвинт, розбитий підшипник
    double mag_interference_scale = 1.0;

    explicit SensorSuite(unsigned seed = 7) : rng_(seed * 977u + 5u), rng_v_(seed * 131u + 17u) {
        for (int k = 0; k < 2; ++k) {
            imu_[size_t(k)].bg = Vec3d{n(cfg.gyro_bias0), n(cfg.gyro_bias0), n(cfg.gyro_bias0)};
            imu_[size_t(k)].ba = Vec3d{n(cfg.accel_bias0), n(cfg.accel_bias0), n(cfg.accel_bias0)};
        }
        prng::uniform<double> u(0, 6.283185307179586);
        for (int i = 0; i < 5; ++i) {
            phase_[size_t(i)] = u(rng_);
            const Vec3d d{n(1), n(1), n(1)};
            dir_[size_t(i)] = d.norm() > 1e-6 ? d.normalized() : Vec3d{0, 0, 1};
        }
    }

    // викликати після кожного кроку фізики (dt = 0.001 с): формує відліки й накопичує їх до циклу оцінювача
    void sample(const QuadPlaneSim& s, double dt) {
        t_ += dt;
        const QpState& st = s.state();
        const Vec3d vw = s.velWorld();
        if (!have_prev_) { prev_vw_ = vw; have_prev_ = true; }
        const Vec3d aw = (vw - prev_vw_) / dt;
        prev_vw_ = vw;
        const Vec3d f_b = st.q.rotateInv(aw + Vec3d{0, 0, QuadPlaneSim::kG});
        // вібрації: кожен мотор — синус на своїх обертах
        Vec3d va, vg;
        for (int i = 0; i < 5; ++i) {
            const double rps = i < 4 ? s.vtolRps(i) : s.cruiseRps(), k = std::pow(rps / cfg.hover_rps, 2);
            const double sn = std::sin(2 * 3.14159265358979 * rps * t_ + phase_[size_t(i)]);
            va += dir_[size_t(i)] * (cfg.vib_accel * vib_scale * motor_vib[size_t(i)] * k * sn);
            vg += dir_[size_t((i + 2) % 5)] * (cfg.vib_gyro * vib_scale * motor_vib[size_t(i)] * k * sn);
        }
        for (int k = 0; k < 2; ++k) {
            Imu& m = imu_[size_t(k)];
            m.bg += Vec3d{n(cfg.gyro_bias_rw), n(cfg.gyro_bias_rw), n(cfg.gyro_bias_rw)} * std::sqrt(dt);
            m.ba += Vec3d{n(cfg.accel_bias_rw), n(cfg.accel_bias_rw), n(cfg.accel_bias_rw)} * std::sqrt(dt);
            Vec3d g = st.omega + s.flexRate() + m.bg + vg + Vec3d{n(cfg.gyro_noise), n(cfg.gyro_noise), n(cfg.gyro_noise)};
            Vec3d a = f_b + m.ba + va + Vec3d{n(cfg.accel_noise), n(cfg.accel_noise), n(cfg.accel_noise)};
            switch (imu_fault[k]) {
                case ImuFault::GyroBiasJump: g += Vec3d{0.0, 0.25, 0.0}; break;   // ≈ 14°/с на осі тангажу
                case ImuFault::AccelBiasJump: a += Vec3d{2.5, 0.0, 0.0}; break;
                case ImuFault::Stuck: g = m.last_g; a = m.last_a; break;
                case ImuFault::Dead: g = Vec3d{}; a = Vec3d{}; break;
                default: break;
            }
            for (int ax = 0; ax < 3; ++ax) { g[ax] = std::clamp(g[ax], -cfg.gyro_range, cfg.gyro_range); a[ax] = std::clamp(a[ax], -cfg.accel_range, cfg.accel_range); }
            if (std::fabs(a.x) >= cfg.accel_range || std::fabs(a.y) >= cfg.accel_range || std::fabs(a.z) >= cfg.accel_range) ++clips_;
            if (imu_fault[k] != ImuFault::Stuck) { m.last_g = g; m.last_a = a; }
            m.sum_g += g; m.sum_a += a; ++m.n;
        }
        vib_peak_ = std::fmax(vib_peak_ * 0.999, va.norm());
        // GPS: історія для затримки
        hist_.push_back({st.pos, vw});
        if (hist_.size() > 400) hist_.pop_front();
        gps_t_ += dt; baro_t_ += dt; mag_t_ += dt;
        const double a_gm = std::exp(-dt / cfg.gps_walk_tau);
        gps_walk_ = gps_walk_ * a_gm + Vec3d{n(cfg.gps_walk), n(cfg.gps_walk), n(cfg.gps_walk * 1.5)} * std::sqrt(1 - a_gm * a_gm);
        {
            const double a_v = std::exp(-dt / cfg.gps_vel_bias_tau), sv = cfg.gps_vel_bias * std::sqrt(1 - a_v * a_v);
            prng::normal<double> nv(0, 1);
            gps_vb_ = gps_vb_ * a_v + Vec3d{sv * nv(rng_v_), sv * nv(rng_v_), 1.5 * sv * nv(rng_v_)};
        }
        if (gps_t_ >= 1.0 / cfg.gps_hz - 1e-9) {
            gps_t_ = 0;
            const size_t back = std::min(hist_.size() - 1, size_t(cfg.gps_delay / dt + 0.5));
            const auto& h = hist_[hist_.size() - 1 - back];
            gps_.pos = toF(h.p + gps_walk_ + Vec3d{n(cfg.gps_noise), n(cfg.gps_noise), n(cfg.gps_noise * 1.5)});
            gps_.vel = toF(h.v + gps_vb_ + Vec3d{n(cfg.gps_vel_noise), n(cfg.gps_vel_noise), n(cfg.gps_vel_noise * 1.5)});
            gps_.fresh = true;
        }
        baro_drift_ += n(cfg.baro_drift_rw) * std::sqrt(dt);
        if (baro_t_ >= 0.02 - 1e-9) { baro_t_ = 0; baro_ = float(st.pos.z + baro_drift_ + n(cfg.baro_noise)); baro_fresh_ = true; }
        if (mag_t_ >= 0.02 - 1e-9) {
            mag_t_ = 0;
            const double inc = cfg.mag_incl_deg * 3.14159265358979 / 180, dec = cfg.mag_decl_deg * 3.14159265358979 / 180;
            const Vec3d mw{cfg.mag_field * std::cos(inc) * std::sin(dec), cfg.mag_field * std::cos(inc) * std::cos(dec), -cfg.mag_field * std::sin(inc)};
            const Vec3d interf = Vec3d{0.3, -0.2, 0.93} * (cfg.mag_motor_ut_per_a * mag_interference_scale * s.busCurrent());
            mag_ = toF(st.q.rotateInv(mw) + interf + Vec3d{n(cfg.mag_noise), n(cfg.mag_noise), n(cfg.mag_noise)});
            mag_fresh_ = true;
        }
    }

    // зібрати входи оцінювача за цикл (усереднені відліки IMU, свіжі GPS/баро/компас) і скинути накопичувачі
    fc::EkfInputs take(bool gps_available) {
        fc::EkfInputs in;
        for (int k = 0; k < 2; ++k) {
            Imu& m = imu_[size_t(k)];
            if (m.n > 0) { in.imu[k].gyro = toF(m.sum_g / double(m.n)); in.imu[k].accel = toF(m.sum_a / double(m.n)); }
            in.imu[k].valid = m.n > 0;
            m.sum_g = Vec3d{}; m.sum_a = Vec3d{}; m.n = 0;
        }
        in.gps = gps_; in.gps.fresh = gps_.fresh && gps_available; gps_.fresh = false;
        in.baro_alt = baro_; in.baro_fresh = baro_fresh_; baro_fresh_ = false;
        in.mag = mag_; in.mag_fresh = mag_fresh_; mag_fresh_ = false;
        return in;
    }
    double vibrationPeak() const { return vib_peak_; }   // м/с²: амплітуда вібрацій (для інтерфейсу, як VIBE в ArduPilot)
    long accelClips() const { return clips_; }
    double magDeclination() const { return cfg.mag_decl_deg * 3.14159265358979 / 180; }

private:
    struct Imu { Vec3d bg, ba, sum_g, sum_a, last_g, last_a; int n = 0; };
    struct H { Vec3d p, v; };
    std::mt19937 rng_, rng_v_;   // rng_v_ — окремо для зсуву доплерівської швидкості (не зсуває решту випадкових послідовностей)
    Vec3d gps_vb_;
    prng::normal<double> nd_{0.0, 1.0};
    std::array<Imu, 2> imu_{};
    std::array<double, 5> phase_{};
    std::array<Vec3d, 5> dir_{};
    std::deque<H> hist_;
    Vec3d prev_vw_, gps_walk_;
    bool have_prev_ = false, baro_fresh_ = false, mag_fresh_ = false;
    fc::GpsSample gps_;
    fc::Vec3 mag_;
    float baro_ = 0;
    double t_ = 0, gps_t_ = 1e9, baro_t_ = 1e9, mag_t_ = 1e9, baro_drift_ = 0, vib_peak_ = 0;
    long clips_ = 0;

    double n(double s) { return s > 0 ? s * nd_(rng_) : 0.0; }
    static fc::Vec3 toF(const Vec3d& v) { return fc::Vec3{float(v.x), float(v.y), float(v.z)}; }
};

}  // namespace sim
