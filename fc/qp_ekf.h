// qp_ekf.h — оцінювач стану QuadPlane: EKF зі стохастичною похибкою (error-state, 15 станів) на IMU + GPS + барометр + компас,
// дві «смуги» на два IMU з вибором кращої (як EKF3 lanes в ArduPilot), фільтри гіроскопа (ФНЧ + гармонічний режектор за обертами ESC).
//
// Номінальний стан: p, v (світ ENU), q (тіло FLU → світ), b_g, b_a. Похибка δx = [δp δv δθ δb_g δb_a], δθ — у тілі.
// Прогноз (кожен цикл IMU, 500 Гц): ω = ω_m − b_g, a = a_m − b_a; p += v·dt + ½(R·a + g)·dt², v += (R·a + g)·dt, q ← q ⊗ Exp(ω·dt);
//   F: δṗ = δv, δv̇ = −R[a]×δθ − R·δb_a, δθ̇ = −[ω]×δθ − δb_g, біаси — випадкове блукання.
// Корекції — скалярні (послідовно по осях): GPS позиція/швидкість (із затримкою: інновація проти збереженого стану t − τ_GPS),
//   барометр (висота), компас (курс із нахилом, компенсованим за поточною орієнтацією).
// Формули — стандартні для ESKF (J. Solà, «Quaternion kinematics for the error-state Kalman filter», 2017, §5–7).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "fc_math.h"
#include "filters.h"

namespace fc {

// Сирі виміри для циклу оцінювача (усереднені за цикл)
struct EkfImu { Vec3 gyro, accel; bool valid = true; };
struct GpsSample { Vec3 pos, vel; bool fresh = false; };
struct EkfInputs {
    EkfImu imu[2];
    GpsSample gps;
    float baro_alt = 0; bool baro_fresh = false;
    Vec3 mag; bool mag_fresh = false;   // мкТл, тіло
};

struct EkfParams {
    float gyro_noise = 0.015f;      // рад/с (σ на цикл)
    float accel_noise = 0.35f;      // м/с²
    float gyro_bias_rw = 2e-4f;     // рад/с/√с
    float accel_bias_rw = 3e-3f;    // м/с²/√с
    float gps_pos_r = 0.9f, gps_vel_r = 0.25f;   // σ вимірювань
    float baro_r = 0.6f;
    float mag_yaw_r = 0.08f;        // рад
    float gps_delay = 0.15f;        // с (як EK3_GPS_DELAY)
    float mag_decl = 0.0f;          // рад: схилення (східне +), якщо відоме
};

class Eskf {
public:
    static constexpr int N = 15;
    EkfParams prm;

    void init(const Vec3& pos, const Quat& q) {
        p_ = pos; v_ = {}; q_ = q; bg_ = {}; ba_ = {};
        for (auto& r : P_) r.fill(0.0f);
        const float d0[N] = {4, 4, 4, 0.5f, 0.5f, 0.5f, 0.03f, 0.03f, 0.3f, 1e-4f, 1e-4f, 1e-4f, 0.04f, 0.04f, 0.04f};
        for (int i = 0; i < N; ++i) P_[i][i] = d0[i];
        hist_n_ = 0; hist_i_ = 0; nis_ = 0; ok_ = true;
    }

    void predict(const EkfImu& s, float dt) {
        if (!ok_) return;
        const Vec3 w = s.gyro - bg_, a = s.accel - ba_;
        const Vec3 g{0, 0, -kGravity};
        const Vec3 aw = q_.rotate(a) + g;
        p_ += v_ * dt + aw * (0.5f * dt * dt);
        v_ += aw * dt;
        q_.integrate(w, dt);
        // F = I + A·dt (блоки 3×3)
        float R[3][3]; rotm(R);
        float Ra[3][3];   // −R·[a]×
        const float ax[3][3] = {{0, -a.z, a.y}, {a.z, 0, -a.x}, {-a.y, a.x, 0}};
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { Ra[i][j] = 0; for (int k = 0; k < 3; ++k) Ra[i][j] -= R[i][k] * ax[k][j]; }
        static thread_local float F[N][N];
        for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) F[i][j] = i == j ? 1.0f : 0.0f;
        for (int i = 0; i < 3; ++i) {
            F[i][3 + i] = dt;
            for (int j = 0; j < 3; ++j) {
                F[3 + i][6 + j] = Ra[i][j] * dt;
                F[3 + i][12 + j] = -R[i][j] * dt;
            }
            F[6 + i][9 + i] = -dt;
        }
        // −[ω]×·dt
        F[6][7] += w.z * dt; F[6][8] -= w.y * dt; F[7][6] -= w.z * dt; F[7][8] += w.x * dt; F[8][6] += w.y * dt; F[8][7] -= w.x * dt;
        // P = F P Fᵀ + Q (використовуємо розрідженість F: рядки з кількома ненульовими елементами)
        static thread_local float FP[N][N];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) {
                float s2 = 0;
                for (int k = 0; k < N; ++k) if (F[i][k] != 0.0f) s2 += F[i][k] * P_[k][j];
                FP[i][j] = s2;
            }
        for (int i = 0; i < N; ++i)
            for (int j = i; j < N; ++j) {
                float s2 = 0;
                for (int k = 0; k < N; ++k) if (F[j][k] != 0.0f) s2 += FP[i][k] * F[j][k];
                P_[i][j] = P_[j][i] = s2;
            }
        const float qv = prm.accel_noise * prm.accel_noise * dt * dt, qt = prm.gyro_noise * prm.gyro_noise * dt * dt;
        const float qbg = prm.gyro_bias_rw * prm.gyro_bias_rw * dt, qba = prm.accel_bias_rw * prm.accel_bias_rw * dt;
        for (int i = 0; i < 3; ++i) { P_[3 + i][3 + i] += qv; P_[6 + i][6 + i] += qt; P_[9 + i][9 + i] += qbg; P_[12 + i][12 + i] += qba; }
        // історія для GPS із затримкою
        hist_[size_t(hist_i_)] = {p_, v_};
        hist_i_ = (hist_i_ + 1) % kHist; hist_n_ = std::min(hist_n_ + 1, kHist);
        dt_ = dt;
        if (!std::isfinite(p_.x + v_.x + q_.w + P_[0][0])) ok_ = false;
    }

    void fuseGps(const GpsSample& g) {
        if (!ok_) return;
        const int back = std::min(hist_n_ - 1, int(prm.gps_delay / std::fmax(dt_, 1e-4f) + 0.5f));
        const Hist& h = hist_[size_t((hist_i_ - 1 - back + 2 * kHist) % kHist)];
        for (int k = 0; k < 3; ++k) scalar(k, g.pos[k] - h.p[k], prm.gps_pos_r * prm.gps_pos_r * (k == 2 ? 4.0f : 1.0f), 5.0f);
        for (int k = 0; k < 3; ++k) scalar(3 + k, g.vel[k] - h.v[k], prm.gps_vel_r * prm.gps_vel_r, 5.0f);
    }
    void fuseBaro(float alt) { if (ok_) scalar(2, alt - p_.z, prm.baro_r * prm.baro_r, 5.0f); }
    void fuseMag(const Vec3& m_b) {
        if (!ok_ || m_b.norm() < 1e-3f) return;
        // курс із магнітометра: поле повертаємо лише на крен/тангаж (горизонтальна система курсу), m_h ∝ (sin(ψ+D), cos(ψ+D)):
        // ENU, курс ψ від сходу проти годинникової, D — схилення (магнітна північ східніше географічної — D > 0)
        const Vec3 e = q_.euler();
        const float yaw_m = headingFromMag(m_b, e.x, e.y, prm.mag_decl);
        float r = yaw_m - e.z; while (r > kPi) r -= 2 * kPi; while (r < -kPi) r += 2 * kPi;
        float R[3][3]; rotm(R);
        float H[N] = {}; H[6] = R[2][0]; H[7] = R[2][1]; H[8] = R[2][2];
        update(H, r, prm.mag_yaw_r * prm.mag_yaw_r, 5.0f);
    }

    static float headingFromMag(const Vec3& m_b, float roll, float pitch, float decl) {
        const Vec3 mh = Quat::fromEuler(roll, pitch, 0).rotate(m_b);
        return std::atan2(mh.x, mh.y) - decl;
    }

    // стан для контролера
    const Vec3& pos() const { return p_; }
    const Vec3& vel() const { return v_; }
    const Quat& att() const { return q_; }
    const Vec3& gyroBias() const { return bg_; }
    const Vec3& accelBias() const { return ba_; }
    float testRatio() const { return nis_; }      // згладжене нормоване квадратичне відхилення інновацій (≈ 1 — узгоджено)
    bool healthy() const { return ok_; }
    float posSigma() const { return std::sqrt(std::fmax(P_[0][0] + P_[1][1], 0.0f)); }

private:
    static constexpr int kHist = 128;
    struct Hist { Vec3 p, v; };
    Vec3 p_, v_, bg_, ba_;
    Quat q_;
    std::array<std::array<float, N>, N> P_{};
    std::array<Hist, kHist> hist_{};
    int hist_n_ = 0, hist_i_ = 0;
    float nis_ = 0, dt_ = 0.002f;
    bool ok_ = true;

    void rotm(float R[3][3]) const {
        const Vec3 c0 = q_.rotate({1, 0, 0}), c1 = q_.rotate({0, 1, 0}), c2 = q_.rotate({0, 0, 1});
        for (int i = 0; i < 3; ++i) { R[i][0] = c0[i]; R[i][1] = c1[i]; R[i][2] = c2[i]; }
    }
    void scalar(int idx, float r, float rv, float gate) { float H[N] = {}; H[idx] = 1; update(H, r, rv, gate); }

    // скалярна корекція з воротами (gate — у σ інновації) і впровадженням похибки в номінальний стан
    void update(const float H[N], float r, float rv, float gate) {
        float PH[N];
        for (int i = 0; i < N; ++i) { float s2 = 0; for (int k = 0; k < N; ++k) if (H[k] != 0.0f) s2 += P_[i][k] * H[k]; PH[i] = s2; }
        float S = rv; for (int k = 0; k < N; ++k) if (H[k] != 0.0f) S += H[k] * PH[k];
        if (S <= 0 || !std::isfinite(S)) return;
        const float ratio = r * r / S;
        nis_ += (ratio - nis_) * 0.02f;
        if (ratio > gate * gate) return;   // викид (стрибок GPS, удар) — не приймаємо
        float K[N], dx[N];
        for (int i = 0; i < N; ++i) { K[i] = PH[i] / S; dx[i] = K[i] * r; }
        for (int i = 0; i < N; ++i) for (int j = i; j < N; ++j) { const float v = P_[i][j] - K[i] * PH[j]; P_[i][j] = P_[j][i] = v; }
        for (int i = 0; i < N; ++i) P_[i][i] = std::fmax(P_[i][i], 1e-9f);
        p_ += Vec3{dx[0], dx[1], dx[2]}; v_ += Vec3{dx[3], dx[4], dx[5]};
        // та сама поправка — і в історії станів: інакше наступна (запізніла) інновація GPS знову побачить уже виправлену похибку
        for (int k = 0; k < hist_n_; ++k) { Hist& h = hist_[size_t(k)]; h.p += Vec3{dx[0], dx[1], dx[2]}; h.v += Vec3{dx[3], dx[4], dx[5]}; }
        const Vec3 dth{dx[6], dx[7], dx[8]};
        const float ang = dth.norm();
        if (ang > 1e-9f) { const float s2 = std::sin(ang / 2) / ang; q_ = q_ * Quat(std::cos(ang / 2), dth.x * s2, dth.y * s2, dth.z * s2); q_.normalize(); }
        bg_ += Vec3{dx[9], dx[10], dx[11]}; ba_ += Vec3{dx[12], dx[13], dx[14]};
        // обмеження біасів фізично можливими значеннями
        for (int k = 0; k < 3; ++k) { bg_[k] = clampf(bg_[k], -0.3f, 0.3f); ba_[k] = clampf(ba_[k], -2.5f, 2.5f); }
    }
};

// Дві смуги EKF (по одній на IMU) + фільтри гіроскопа для регуляторів
class QpNavigator {
public:
    EkfParams prm;
    float gyro_lpf_hz = 80.0f;          // INS_GYRO_FILTER
    bool harmonic_notch = true;         // INS_HNTCH: режектор на частоті обертів підйомних гвинтів (з телеметрії ESC)
    float struct_notch_hz[3] = {0, 0, 0};   // INS_NTCH: нерухомий режектор на власній частоті конструкції по осях (0 — вимкнено)
    float struct_notch2_hz[3] = {0, 0, 0};  // попередня оцінка (коливання в польоті «перескакують», коли першу частоту прибрано)
    float struct_q = 2.0f;              // ширина режектора конструкції: смуга ≈ f/Q (частота в польоті «пливе» від частоти конструкції)
    bool struct_detect = true;          // пошук власної частоти конструкції в польоті (спектр гіроскопа 10…45 Гц) і автоматичний режектор
    float structPeak(int k) const { return sp_amp_[k]; }
    float structSpec(int k, int b) const { return sp_[k][b]; }   // амплітуда знайденого піку, рад/с

    void reset() { inited_ = false; primary_ = 0; switches_ = 0; t_ = 0; imu_bad_[0] = imu_bad_[1] = false; for (auto& l : lane_) l.prm = prm; }

    // dt — період циклу (0.002 с); rps — середні оберти підйомних моторів (для режектора); still — апарат нерухомий на землі (ініціалізація)
    void update(const EkfInputs& in, float dt, float rps_mean) {
        t_ += dt;
        if (!inited_) {
            if (!in.gps.fresh || !in.mag_fresh) return;
            // початкова орієнтація: крен/тангаж з акселерометра, курс з компаса
            const Vec3 a = in.imu[0].accel;
            const float roll = std::atan2(a.y, a.z), pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
            const float yaw = Eskf::headingFromMag(in.mag, roll, pitch, prm.mag_decl);
            for (int k = 0; k < 2; ++k) { lane_[size_t(k)].prm = prm; lane_[size_t(k)].init(in.gps.pos, Quat::fromEuler(roll, pitch, yaw)); }
            for (int k = 0; k < 3; ++k) { lpf_[k].setup(gyro_lpf_hz, 1.0f / dt); }
            inited_ = true;
        }
        // перевірка самих IMU: завислий вихід (ідентичні відліки), «мертвий» (нулі), розбіжність гіроскопів двох IMU
        for (int k = 0; k < 2; ++k) {
            const EkfImu& m = in.imu[k];
            const bool same = m.gyro.x == last_[k].gyro.x && m.gyro.y == last_[k].gyro.y && m.accel.z == last_[k].accel.z;
            stuck_n_[k] = same ? stuck_n_[k] + 1 : 0;
            dead_n_[k] = (m.accel.norm() < 0.5f && m.gyro.norm() < 1e-6f) ? dead_n_[k] + 1 : 0;
            last_[k] = m;
            if (stuck_n_[k] > 10 || dead_n_[k] > 10 || !m.valid) imu_bad_[k] = true;
            // рівень вібрацій: середнє |a − ФНЧ(a)| → шум акселерометра в EKF (сильні вібрації — менше довіри прискоренню)
            acc_lp_[k] += (m.accel - acc_lp_[k]) * (dt / (0.05f + dt));
            vib_[k] += ((m.accel - acc_lp_[k]).norm() - vib_[k]) * (dt / (1.0f + dt));
        }
        {   // два IMU не згодні (гіро > 0.15 рад/с або згладжені акселерометри > 1.5 м/с² протягом 0.3 с):
            // поганий той, чия смуга гірше узгоджена з GPS/баро/компасом
            const Vec3 d = in.imu[0].gyro - in.imu[1].gyro, da = acc_lp_[0] - acc_lp_[1];
            disagree_t_ = (d.norm() > 0.15f || da.norm() > 1.5f) ? disagree_t_ + dt : 0.0f;
            if (disagree_t_ > 0.3f && !imu_bad_[0] && !imu_bad_[1]) imu_bad_[lane_[0].testRatio() > lane_[1].testRatio() ? 0 : 1] = true;
        }
        for (int k = 0; k < 2; ++k) {
            Eskf& L = lane_[size_t(k)];
            L.prm.accel_noise = std::fmax(prm.accel_noise, 0.6f * vib_[k]);
            if (in.imu[k].valid && !imu_bad_[k]) L.predict(in.imu[k], dt);
            if (in.gps.fresh) L.fuseGps(in.gps);
            if (in.baro_fresh) L.fuseBaro(in.baro_alt);
            if (in.mag_fresh) L.fuseMag(in.mag);
        }
        // вибір смуги: інша смуга помітно узгодженіша протягом 1 с або основна зламалась
        const int o = 1 - primary_;
        const bool better = lane_[size_t(o)].healthy() && lane_[size_t(o)].testRatio() < 0.5f * lane_[size_t(primary_)].testRatio() && lane_[size_t(primary_)].testRatio() > 1.5f;
        worse_t_ = better ? worse_t_ + dt : 0.0f;
        if ((worse_t_ > 1.0f || !lane_[size_t(primary_)].healthy() || imu_bad_[primary_]) && lane_[size_t(o)].healthy() && !imu_bad_[o]) { primary_ = o; worse_t_ = 0; ++switches_; }
        // гіроскоп для регуляторів: основний IMU мінус біас, ФНЧ 80 Гц + режектор на 1-й гармоніці обертів
        const Vec3 w = in.imu[primary_].gyro - lane_[size_t(primary_)].gyroBias();
        Vec3 f = {lpf_[0].apply(w.x), lpf_[1].apply(w.y), lpf_[2].apply(w.z)};
        if (harmonic_notch && rps_mean > 20.0f) {
            for (int k = 0; k < 3; ++k) { notch_[k].set(rps_mean, 1.0f / dt); f[k] = notch_[k].apply(f[k]); }
        }
        if (struct_detect && rps_mean > 20.0f) structDetect(f, dt);
        for (int k = 0; k < 3; ++k) {
            if (struct_notch_hz[k] > 1) {   // подвійний режектор (f·0.95 і f·1.05): глибока смуга, бо частота рами в польоті трохи «пливе» (земля, тяга, швидкість)
                snotch_[k].q = snotch2_[k].q = struct_q;
                snotch_[k].set(struct_notch_hz[k] * 0.95f, 1.0f / dt); snotch2_[k].set(struct_notch_hz[k] * 1.05f, 1.0f / dt);
                f[k] = snotch2_[k].apply(snotch_[k].apply(f[k]));
            }
        }
        omega_ = f;
    }

    bool ready() const { return inited_; }
    bool imuBad(int k) const { return imu_bad_[k]; }
    float vibration(int k) const { return vib_[k]; }   // м/с²: середня амплітуда високочастотного прискорення
    const Eskf& lane(int i) const { return lane_[size_t(i)]; }
    const Eskf& primary() const { return lane_[size_t(primary_)]; }
    int primaryIndex() const { return primary_; }
    int laneSwitches() const { return switches_; }
    const Vec3& omega() const { return omega_; }

private:
    // спектр крену й тангажу блоками по 0.5 с (Ґьорцель на 10…45 Гц через 1 Гц), згладжений між блоками;
    // різкий пік (≥ 6× медіани, ≥ 0.08 рад/с) кілька блоків поспіль — це пружні коливання конструкції, що замкнулися через регулятор
    static constexpr int kSb = 36;
    void structDetect(const Vec3& f, float dt) {
        const float x[2] = {f.x, f.y};
        if (sn_ == 0) for (int k = 0; k < 2; ++k) for (int b = 0; b < kSb; ++b) { gs1_[k][b] = gs2_[k][b] = 0; }
        for (int k = 0; k < 2; ++k) for (int b = 0; b < kSb; ++b) {
            const float c = 2 * std::cos(2 * kPi * float(10 + b) * dt), s0 = x[k] + c * gs1_[k][b] - gs2_[k][b];
            gs2_[k][b] = gs1_[k][b]; gs1_[k][b] = s0;
        }
        if (++sn_ < int(0.5f / dt + 0.5f)) return;
        for (int k = 0; k < 2; ++k) {
            float pw[kSb], srt[kSb];
            for (int b = 0; b < kSb; ++b) {
                const float c = 2 * std::cos(2 * kPi * float(10 + b) * dt);
                const float p = gs1_[k][b] * gs1_[k][b] + gs2_[k][b] * gs2_[k][b] - c * gs1_[k][b] * gs2_[k][b];
                const float a = 2 * std::sqrt(std::fmax(p, 0.0f)) / float(sn_);
                sp_[k][b] = sp_[k][b] > 0 ? sp_[k][b] + 0.5f * (a - sp_[k][b]) : a;
                pw[b] = srt[b] = sp_[k][b];
            }
            std::sort(srt, srt + kSb);
            int m = 0; for (int b = 1; b < kSb; ++b) if (pw[b] > pw[m]) m = b;
            const bool peak = m > 0 && m < kSb - 1 && pw[m] > 6 * srt[kSb / 2] && pw[m] > 0.08f;
            sp_n_[k] = peak ? sp_n_[k] + 1 : 0;
            if (sp_n_[k] >= 2) {   // уточнення по сусідніх бінах (парабола)
                float fr = float(10 + m);
                if (m > 0 && m < kSb - 1) { const float y0 = pw[m - 1], y1 = pw[m], y2 = pw[m + 1], d = y0 - 2 * y1 + y2; if (d < 0) fr += 0.5f * (y0 - y2) / d; }
                float& n1 = struct_notch_hz[k]; float& n2 = struct_notch2_hz[k];
                if (n1 < 1) { n1 = fr; sp_amp_[k] = pw[m]; sp_n_[k] = 0; for (int b = 0; b < kSb; ++b) sp_[k][b] = 0; }
                else if (sp_mv_[k] < 4 && std::fabs(fr - n1) > 0.7f) { n2 = n1; n1 = fr; ++sp_mv_[k]; sp_n_[k] = 0; for (int b = 0; b < kSb; ++b) sp_[k][b] = 0; }   // коливання перескочили — частота рами інша, переносимо режектор
            }
        }
        sn_ = 0;
    }
    float gs1_[2][kSb] = {}, gs2_[2][kSb] = {}, sp_[2][kSb] = {}, sp_amp_[3] = {0, 0, 0};
    int sn_ = 0, sp_n_[2] = {0, 0}, sp_mv_[2] = {0, 0};
    // режекторний біквадратний фільтр (RBJ), Q = 2; частота оновлюється плавно
    struct Notch {
        float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0, f0 = -1;
        float q = 2.0f;
        void set(float f, float fs) {
            if (std::fabs(f - f0) < 0.05f) return;
            f0 = f;
            const float w0 = 2 * kPi * clampf(f, 5.0f, 0.45f * fs) / fs, al = std::sin(w0) / (2 * q), c = std::cos(w0), a0 = 1 + al;
            b0 = 1 / a0; b1 = -2 * c / a0; b2 = 1 / a0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
        }
        float apply(float x) { const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2; x2 = x1; x1 = x; y2 = y1; y1 = y; return y; }
    };
    std::array<Eskf, 2> lane_;
    Biquad lpf_[3];
    Notch notch_[3], snotch_[3], snotch2_[3];
    Vec3 omega_;
    int primary_ = 0, switches_ = 0;
    float worse_t_ = 0, t_ = 0, disagree_t_ = 0;
    EkfImu last_[2];
    int stuck_n_[2] = {0, 0}, dead_n_[2] = {0, 0};
    bool imu_bad_[2] = {false, false};
    Vec3 acc_lp_[2];
    float vib_[2] = {0, 0};
    bool inited_ = false;
};

}  // namespace fc
