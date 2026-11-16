// qp_control.h — регулятори QuadPlane (float, без залежності від симулятора): оцінка → команди.
//   VtolController — висота/швидкість/позиція → нахил → швидкості → моменти → мікшер 4 моторів (fc::mixQuadX);
//   FwController   — крен/тангаж/рискання літака керми (з масштабуванням за швидкістю);
//   Tecs           — енергія: тяга маршового гвинта + тангаж (за формулами AP_TECS.cpp, прочитано в тексті);
//   L1             — бічна навігація (AP_L1_Control.cpp: K_L1 = 4ζ², L1 = ζ·T·Vg/π, a = K_L1·Vg²/L1·sin Nu).
// Система координат: світ ENU (Z вгору), тіло FLU; кут тангажа Ейлера «+ ніс ВНИЗ», команди керми — як у sim/qp_aero.h.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "fc_math.h"
#include "mixer.h"
#include "pid.h"
#include "qp_motor_fail.h"

namespace fc {

struct QpEstimate {
    Vec3 pos, vel;        // світ, м, м/с
    Quat q;               // тіло -> світ
    Vec3 omega;           // рад/с, тіло
    float airspeed = 0;   // м/с (скаляр, труба Піто)
    float v_bus = 24;     // В
    bool gps_ok = true;   // false — GPS втрачено (позиція/швидкість недостовірні)
    float bus_current = 0;   // А, струм батареї (телеметрія ESC/шунта) — для обмежувача струму
    float vtol_rps[4] = {-1, -1, -1, -1};   // об/с підйомних моторів (телеметрія ESC); < 0 — телеметрії немає
    float cruise_rps = -1;                  // об/с маршового мотора (телеметрія ESC)
    float tilt_meas[4] = {-9, -9, -9, -9};  // рад: положення гондол з датчика сервоприводу (конвертоплан); < −5 — датчика немає
    Vec3 accel{0, 0, 9.80665f};   // м/с²: питома сила з акселерометра (тіло), відфільтрована від вібрацій, без оціненого біасу
    bool est_ok = true;   // оцінювач (EKF) ініціалізований і здоровий
    bool rc_ok = true;    // є зв'язок з пультом
    float dt_hint = 0.002f;
};

// Коефіцієнти регуляторів (ім'я параметра у файлі — у fc/qp_params.h)
struct QpGains {
    PidGains vz{4.0f, 1.5f, 0.0f, 4.0f, 9.0f, 20.0f};             // вертикальна швидкість → прискорення
    PidGains vxy{2.0f, 0.5f, 0.0f, 3.0f, 5.0f, 0.0f};             // горизонтальна швидкість → прискорення
    PidGains rate_rp{28.0f, 25.0f, 0.4f, 60.0f, 400.0f, 40.0f};   // кутова швидкість крену/тангажу (коптер)
    PidGains rate_yaw{14.0f, 6.0f, 0.0f, 40.0f, 200.0f, 0.0f};    // кутова швидкість рискання (коптер)
    PidGains fw_roll{0.09f, 0.06f, 0.0f, 0.25f, 0.43f, 30.0f};    // літак: кутова швидкість крену → елерони
    PidGains fw_pitch{0.12f, 0.10f, 0.0f, 0.25f, 0.43f, 30.0f};   // літак: кутова швидкість тангажу → руль висоти
};

struct QpAirframe {       // що контролер знає про апарат
    float mass = 3.5f;
    Vec3 inertia{0.308f, 0.161f, 0.458f};
    MixerConfig mixer;    // arm_x, arm_y, k_yaw, max_thrust, idle
    float v_stall = 10.0f;
    float v_cruise = 17.0f;
    QpGains gains;
};

inline float wrapPi(float a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }

struct Euler { float roll, pitch_down, yaw; };
inline Euler eulerOf(const Quat& q) { Vec3 e = q.euler(); return {e.x, e.y, e.z}; }

// Таблиця тяга ↔ duty (з моделі мотора при V_шини = vref); зворотне відображення з компенсацією напруги
class ThrustMap {
public:
    void set(std::vector<float> duty, std::vector<float> thrust, float vref) { duty_ = std::move(duty); thr_ = std::move(thrust); vref_ = vref; }
    bool valid() const { return duty_.size() > 2; }
    float maxThrust() const { return thr_.empty() ? 0.f : thr_.back(); }
    // пряме відображення: duty → величина таблиці (тяга чи оберти) з урахуванням напруги шини
    float fromDuty(float duty, float v_bus) const {
        if (!valid()) return 0;
        const float d = clampf(duty * std::fmax(v_bus, 6.0f) / vref_, 0.f, duty_.back());
        size_t hi = size_t(std::upper_bound(duty_.begin(), duty_.end(), d) - duty_.begin());
        hi = std::min(std::max<size_t>(hi, 1), duty_.size() - 1);
        const float f = (d - duty_[hi - 1]) / std::fmax(duty_[hi] - duty_[hi - 1], 1e-6f);
        return thr_[hi - 1] + (thr_[hi] - thr_[hi - 1]) * clampf(f, 0.f, 1.f);
    }
    float toDuty(float thrust, float v_bus) const {
        if (!valid()) return 0;
        thrust = clampf(thrust, 0.f, thr_.back());
        size_t hi = size_t(std::upper_bound(thr_.begin(), thr_.end(), thrust) - thr_.begin());
        hi = std::min(std::max<size_t>(hi, 1), thr_.size() - 1);
        const float f = (thrust - thr_[hi - 1]) / std::fmax(thr_[hi] - thr_[hi - 1], 1e-6f);
        const float d = duty_[hi - 1] + (duty_[hi] - duty_[hi - 1]) * clampf(f, 0.f, 1.f);
        return clampf(d * vref_ / std::fmax(v_bus, 6.0f), 0.f, 1.f);
    }
private:
    std::vector<float> duty_, thr_;
    float vref_ = 24;
};

// ---------- VTOL ----------
struct VtolTargets {
    enum VMode { Position, Velocity } hmode = Position;      // горизонталь: точка або швидкість
    enum ZMode { Altitude, ClimbRate } zmode = Altitude;      // вертикаль: висота або швидкість підйому
    Vec3 pos;                 // ціль (x, y, z)
    Vec3 vel_xy;              // для hmode == Velocity (x, y), м/с; z ігнорується
    float climb_rate = 0;     // для zmode == ClimbRate
    float yaw = 0;
    float max_tilt = 25 * kDeg2Rad;
    float pitch_up_limit = 25 * kDeg2Rad;   // обмеження носа вгору (в переході — менше: крило не має зірватись)
    float v_xy_max = 5.0f;
    bool att_override = false;     // якщо true — кути крену/тангажу задаються напряму (перехід: узгоджено з керми літака)
    float roll_o = 0, pitch_down_o = 0;
    float att_blend = 0;           // 0…1: частка заданих кутів (roll_o, pitch_down_o) при вимкненому override — плавний вихід із переходу
    float fwd_pitch_lim = -1;      // рад: Q_FWD_THR_USE — ніс униз не більше за цей кут, решту прискорення вперед дає маршовий гвинт (< 0 — вимкнено)
    float wind_pitch_up = 0;       // рад: у сильному зустрічному вітрі — ніс трохи вгору, крило несе частину ваги, вперед тягнуть гондоли
    float fwd_neg_max = 0;         // Н: конвертоплан — гальмування нахилом гондол назад замість задирання носа (0 — лише вперед)
};

struct VtolCmd {
    float thrust = 0;         // сумарна тяга, Н
    Vec3 torque;              // Н·м
    float roll_sp = 0, pitch_down_sp = 0;   // цілі кутів, рад
    float cmd[4] = {};        // нормована тяга моторів
    float thrust_i[4] = {};   // Н
    bool sat = false;
    float fwd_thrust = 0;     // Н: потрібна тяга маршового гвинта (Q_FWD_THR_USE)
};

// ArduPilot-подібний «sqrt-контролер»: лінійний біля цілі, далі v = √(2·a·(|e| − a/(2k²))) — гальмування з обмеженим прискоренням
inline float sqrtCtrl(float err, float kp, float a_max, float v_max) {
    const float lin = a_max / (kp * kp), ae = std::fabs(err);
    float v = ae <= lin ? kp * ae : std::sqrt(2.0f * a_max * (ae - lin * 0.5f));
    v = std::fmin(v, v_max);
    return err < 0 ? -v : v;
}

class VtolController {
public:
    void configure(const QpAirframe& af) {
        af_ = af;
        vz_.setGains(af.gains.vz);
        vx_.setGains(af.gains.vxy); vy_.setGains(af.gains.vxy);
        rate_x_.setGains(af.gains.rate_rp); rate_y_.setGains(af.gains.rate_rp);
        rate_z_.setGains(af.gains.rate_yaw);
    }
    void reset() { vz_.reset(); vx_.reset(); vy_.reset(); rate_x_.reset(); rate_y_.reset(); rate_z_.reset(); prev_roll_ = prev_pitch_ = 0; }
    void setMass(float m, const Vec3& inertia) { af_.mass = m; af_.inertia = inertia; }
    Vec3 rate_inject{0, 0, 0};   // добавка до заданої кутової швидкості (самоналаштування)
    Vec3 lastAcc() const { return last_acc_; }   // кутове прискорення, яке просив регулятор швидкості на останньому циклі
    const Vec3& inertia() const { return af_.inertia; }
    void presetThrustIntegral(float az) { vz_.reset(az); }
    float verticalIntegral() const { return vz_.integral(); }
    void shiftVerticalIntegral(float d) { vz_.shiftIntegral(d); }
    // повна відмова підйомного мотора: далі — керування з обертанням (qp_motor_fail.h)
    void setFailedMotor(int i, float dt = 0.002f) { spin_.configure(i, af_.mixer.dx(), af_.mixer.dy(), af_.inertia, af_.mixer.max_thrust, dt); }
    bool motorFailed() const { return spin_.active(); }
    void holdIntegrators(bool h) { int_hold_ = h; }
    void setVelGainScale(float kp_s, float ki_s) { PidGains g = af_.gains.vxy; g.kp *= kp_s; g.ki *= ki_s; vx_.setGains(g); vy_.setGains(g); }
    // Передбачене зовнішнє прискорення (світ, м/с²) — аеродинаміка й вітер, оцінені ШІ-співпроцесором (ai/force_model.h):
    // контролер віднімає його від потрібного прискорення наперед, не чекаючи, поки похибка накопичиться в інтеграторах. 0 — вимкнено.
    Vec3 a_dist{0, 0, 0};

    // Ручне керування без контурів положення й висоти: acro=false — ручки задають кути (±max_angle), acro=true — кутові швидкості (±max_rate).
    // roll_in (+ правий крен), pitch_in (+ ніс униз), yaw_in (+ курс праворуч), −1…1; thr — колективна тяга 0…1 від максимуму всіх моторів.
    VtolCmd manualUpdate(float dt, const QpEstimate& e, bool acro, float roll_in, float pitch_in, float yaw_in, float thr, float max_angle = 0.4363f, float max_rate = 3.14f) {
        VtolCmd c;
        const Euler eu = eulerOf(e.q);
        float p_sp, q_sp;
        if (acro) { p_sp = max_rate * roll_in; q_sp = max_rate * pitch_in; }
        else {
            c.roll_sp = max_angle * roll_in; c.pitch_down_sp = max_angle * pitch_in;
            p_sp = clampf(7.0f * (c.roll_sp - eu.roll), -3.5f, 3.5f); q_sp = clampf(7.0f * (c.pitch_down_sp - eu.pitch_down), -3.5f, 3.5f);
        }
        const float r_sp = -1.5f * yaw_in;   // курс праворуч — обертання за годинниковою (−Z)
        c.torque = {rate_x_.update(p_sp, e.omega.x, dt) * af_.inertia.x, rate_y_.update(q_sp, e.omega.y, dt) * af_.inertia.y, rate_z_.update(r_sp, e.omega.z, dt) * af_.inertia.z};
        c.thrust = clampf(thr, 0.0f, 1.0f) * af_.mixer.totalMax();
        const MixerResult m = mixQuadX(c.thrust, c.torque, af_.mixer);
        for (int i = 0; i < 4; ++i) { c.cmd[i] = m.cmd[i]; c.thrust_i[i] = c.thrust > 0.05f ? m.thrust[i] : 0.0f; }
        c.sat = m.sat_attitude;
        prev_roll_ = eu.roll; prev_pitch_ = eu.pitch_down;   // плавний вихід в автоматичні режими
        return c;
    }
    const SpinController& spin() const { return spin_; }

    VtolCmd update(float dt, const QpEstimate& e, const VtolTargets& t) {
        VtolCmd c;
        const Euler eu = eulerOf(e.q);
        // --- вертикаль
        float vz_sp = t.zmode == VtolTargets::ClimbRate ? t.climb_rate : clampf(1.0f * (t.pos.z - e.pos.z), -1.5f, 2.5f);
        const float az = vz_.update(vz_sp, e.vel.z, dt, !int_hold_) - a_dist.z;
        // --- горизонталь
        Vec3 v_sp;
        if (t.hmode == VtolTargets::Position) {
            v_sp = {sqrtCtrl(t.pos.x - e.pos.x, 0.8f, 1.0f, t.v_xy_max), sqrtCtrl(t.pos.y - e.pos.y, 0.8f, 1.0f, t.v_xy_max), 0};
            const float n = std::hypot(v_sp.x, v_sp.y);
            if (n > t.v_xy_max) { v_sp.x *= t.v_xy_max / n; v_sp.y *= t.v_xy_max / n; }
        } else {
            v_sp = {t.vel_xy.x, t.vel_xy.y, 0};
        }
        const float ax = vx_.update(v_sp.x, e.vel.x, dt) - a_dist.x, ay = vy_.update(v_sp.y, e.vel.y, dt) - a_dist.y;
        const float g_eff = kGravity + az;
        if (spin_.active()) {   // без одного мотора: бажаний напрям тяги у світі → LQR з обертанням; нахил ≤ ≈ 19°
            float axl = ax, ayl = ay;
            const float an = std::hypot(axl, ayl), amax = 0.35f * std::fmax(g_eff, 3.0f);
            if (an > amax) { axl *= amax / an; ayl *= amax / an; }
            const Vec3 a{axl, ayl, std::fmax(g_eff, 3.0f)};
            const float an3 = a.norm();
            const Vec3 d = a / an3;
            const float nz = e.q.rotateInv(d).z;
            const float tt = clampf(af_.mass * an3 / std::fmax(nz, 0.7f), 0.f, 0.5f * af_.mixer.totalMax());
            const SpinController::Out so = spin_.update(e.q, e.omega, d, tt);
            c.thrust = tt;
            for (int i = 0; i < 4; ++i) { c.thrust_i[i] = so.thrust[i]; c.cmd[i] = so.thrust[i] / af_.mixer.maxOf(i); }
            return c;
        }
        // бажана тяга-вектор у світі → нахил відносно курсу
        const float cy = std::cos(eu.yaw), sy = std::sin(eu.yaw);
        const float a_fwd = ax * cy + ay * sy, a_left = -ax * sy + ay * cy;
        float pitch_down = std::atan2(a_fwd, g_eff), roll = -std::atan2(a_left, g_eff);
        // обмеження сумарного нахилу
        const float tilt = std::hypot(pitch_down, roll);
        if (tilt > t.max_tilt) { pitch_down *= t.max_tilt / tilt; roll *= t.max_tilt / tilt; }
        pitch_down = std::max(pitch_down, -t.pitch_up_limit);
        float fwd_lim = t.fwd_pitch_lim;
        if (fwd_lim >= 0 && !t.att_override && t.att_blend <= 0 && az < -1.5f) {
            // крило в зустрічному вітрі несе апарат угору, а гвинти вже майже на холостих: опускаємо ніс (α < 0), щоб крило перестало тягнути вгору
            const float unload = clampf((-az - 1.0f) * 0.05f, 0.0f, 0.21f);
            fwd_lim += unload;
        }
        // гвинти мають лишатися навантаженими (≥ ~50% ваги): інакше порив, що піднімає крило, нічим погасити — тягу нижче нуля не зменшиш
        load_lp_ += (g_eff / 9.81f - load_lp_) * std::fmin(1.0f, dt / 2.0f);
        const float sh = fwd_lim >= 0 ? clampf(t.wind_pitch_up, -3 * kDeg2Rad, 12 * kDeg2Rad) * clampf((load_lp_ - 0.45f) / 0.25f, 0.0f, 1.0f) : 0.0f;   // смуга «рівного корпусу» зсувається носом угору
        if (fwd_lim >= 0 && !t.att_override && t.att_blend <= 0 && pitch_down > fwd_lim - sh) {
            // як Q_FWD_THR_USE в ArduPilot: замість нахилу носом униз (крило тоді тисне вниз) тягне маршовий гвинт, крило лишається на α ≈ 0…3°
            c.fwd_thrust = std::fmax(0.0f, af_.mass * (a_fwd - g_eff * std::tan(fwd_lim - sh)));
            pitch_down = fwd_lim - sh;
        }
        if (fwd_lim >= 0 && t.fwd_neg_max > 0 && !t.att_override && t.att_blend <= 0 && pitch_down < -fwd_lim - sh) {
            // конвертоплан: назад «тягне» нахил гондол назад — корпус лишається майже рівним (ніс угору не більше fwd_lim)
            c.fwd_thrust = std::fmax(af_.mass * (a_fwd + g_eff * std::tan(fwd_lim + sh)), -t.fwd_neg_max);
            // гондоли вже на упорі назад — решту гальмування дає нахил корпусу
            pitch_down = std::fmin(-fwd_lim - sh, std::atan2(a_fwd - c.fwd_thrust / af_.mass, g_eff));
        }
        if (t.att_override) { roll = t.roll_o; pitch_down = t.pitch_down_o; }
        else if (t.att_blend > 0) { roll += (t.roll_o - roll) * t.att_blend; pitch_down += (t.pitch_down_o - pitch_down) * t.att_blend; }
        {   // обмеження швидкості зміни цільових кутів (35°/с): без ривків при перемиканні режимів
            const float lim = 0.61f * dt;
            roll = prev_roll_ + clampf(roll - prev_roll_, -lim, lim);
            pitch_down = prev_pitch_ + clampf(pitch_down - prev_pitch_, -lim, lim);
            prev_roll_ = roll; prev_pitch_ = pitch_down;
        }
        c.roll_sp = roll; c.pitch_down_sp = pitch_down;
        // --- тяга з компенсацією нахилу
        const float cos_tilt = clampf(std::cos(eu.roll) * std::cos(eu.pitch_down), 0.6f, 1.0f);
        c.thrust = clampf(af_.mass * g_eff / cos_tilt, 0.f, af_.mixer.totalMax());
        // --- кути → швидкості → моменти
        const float p_sp = clampf(7.0f * (roll - eu.roll), -3.5f, 3.5f), q_sp = clampf(7.0f * (pitch_down - eu.pitch_down), -3.5f, 3.5f);
        const float r_sp = clampf(2.5f * wrapPi(t.yaw - eu.yaw), -1.0f, 1.0f);
        const float acc_x = rate_x_.update(p_sp + rate_inject.x, e.omega.x, dt), acc_y = rate_y_.update(q_sp + rate_inject.y, e.omega.y, dt), acc_z = rate_z_.update(r_sp + rate_inject.z, e.omega.z, dt);
        last_acc_ = {acc_x, acc_y, acc_z};
        c.torque = {acc_x * af_.inertia.x, acc_y * af_.inertia.y, acc_z * af_.inertia.z};
        const MixerResult m = mixQuadX(c.thrust, c.torque, af_.mixer);
        for (int i = 0; i < 4; ++i) { c.cmd[i] = m.cmd[i]; c.thrust_i[i] = m.thrust[i]; }
        c.sat = m.sat_attitude;
        return c;
    }

private:
    Vec3 last_acc_{0, 0, 0};
    QpAirframe af_;
    Pid vz_, vx_, vy_, rate_x_, rate_y_, rate_z_;
    float prev_roll_ = 0, prev_pitch_ = 0, load_lp_ = 1;
    SpinController spin_;
    bool int_hold_ = false;
};

// ---------- L1 ----------
class L1Guidance {
public:
    float period = 12.0f, damping = 0.75f;
    // відрізок A→B (світ XY); повертає бічне прискорення (м/с², + вліво), nu_out — кут Nu
    float update(const Vec3& pos, const Vec3& vel, const Vec3& a, const Vec3& b, float* nu_out = nullptr) const {
        const float vg = std::fmax(std::hypot(vel.x, vel.y), 1.0f);
        const float l1 = std::fmax(0.3183099f * damping * period * vg, 20.0f);
        float abx = b.x - a.x, aby = b.y - a.y;
        const float ab = std::hypot(abx, aby);
        if (ab < 1.0f) { abx = 1; aby = 0; } else { abx /= ab; aby /= ab; }
        const float rx = pos.x - a.x, ry = pos.y - a.y;
        const float xtrack = rx * aby - ry * abx;           // (r × AB)_z: + коли літак праворуч від AB
        const float xtrack_vel = vel.x * aby - vel.y * abx;
        const float ltrack_vel = vel.x * abx + vel.y * aby;
        const float nu2 = std::atan2(xtrack_vel, ltrack_vel);
        const float sin_nu1 = clampf(xtrack / l1, -0.7071f, 0.7071f);
        const float nu = clampf(std::asin(sin_nu1) + nu2, -kPi / 2, kPi / 2);
        if (nu_out) *nu_out = nu;
        const float k_l1 = 4 * damping * damping;
        return k_l1 * vg * vg / l1 * std::sin(nu);          // a_left: xtrack > 0 — літак ПРАВОРУЧ від лінії AB → потрібен поворот ліворуч (+)
    }
};

// ---------- TECS ----------
struct TecsConfig {
    float time_const = 4.0f;       // с
    float spd_weight = 1.0f;       // 0 — тільки швидкість тягою, 2 — тільки тангажем
    float pitch_damp = 0.4f, thr_damp = 0.3f;
    float i_pitch = 0.05f, i_thr = 0.4f;     // інтегральні підсилення
    float climb_max = 3.0f, sink_max = 2.5f;
    float pitch_min = -15 * kDeg2Rad, pitch_max = 18 * kDeg2Rad;   // кут траєкторії + α
    float thrust_max = 12.0f;      // Н, маршовий
};

class Tecs {
public:
    TecsConfig cfg;
    // Таблиці (за швидкістю): тяга для рівного польоту й α_тримування — ідентифікація моделі (як TECS_THR_TRIM в ArduPilot)
    void setTrim(std::vector<float> v, std::vector<float> thrust, std::vector<float> alpha) { v_ = std::move(v); thr_ = std::move(thrust); alpha_ = std::move(alpha); }
    void reset(float h, float v) { h_dem_f_ = h; integ_thr_ = 0; integ_seb_ = 0; vdot_f_ = 0; v_prev_ = v; first_ = true; }
    float pitchUpDemand() const { return pitch_out_; }
    float thrustDemand() const { return thr_out_; }
    float flightPathDemand() const { return gamma_dem_; }
    // Таблиці тримування зняті для повної маси trim_mass_; α масштабується від нульової підйомної сили (CL ∝ m), тяга — паразитна + індукована ∝ m²
    void setTrimMass(float m) { trim_mass_ = m; mass_ = m; }
    void setMass(float m) { mass_ = m; }
    float alphaTrim(float v) const { return alpha_zero_ + (interp(alpha_, v) - alpha_zero_) * mass_ / trim_mass_; }
    float thrustTrim(float v) const { const float r = mass_ / trim_mass_; return interp(thr_, v) * (0.55f + 0.45f * r * r); }

    // h, vz — висота й швидкість підйому; v — повітряна швидкість; h_dem, v_dem — цілі; mass — для енергетичних відношень
    void update(float dt, float mass, float h, float vz, float v, float h_dem, float v_dem) {
        v = std::fmax(v, 5.0f);
        if (first_) { first_ = false; v_prev_ = v; }
        const float g = kGravity;
        // фільтрований прискорення за швидкістю
        const float vdot_raw = (v - v_prev_) / dt; v_prev_ = v;
        vdot_f_ += (vdot_raw - vdot_f_) * (dt / (0.25f + dt));
        // цілі: висота — перший порядок + ліміт швидкості, швидкість — ліміт прискорення
        const float tau = cfg.time_const;
        float dh = clampf((h_dem - h_dem_f_) / tau, -cfg.sink_max, cfg.climb_max);
        h_dem_f_ += dh * dt;
        h_dem_f_ = clampf(h_dem_f_, h - 20.0f, h + 20.0f);
        const float vdot_dem = clampf((v_dem - v) / tau, -1.5f, 1.5f);
        const float hdot_dem = clampf((h_dem_f_ - h) / tau, -cfg.sink_max, cfg.climb_max);
        // енергії (питомі)
        const float spe_err = (h_dem_f_ - h) * g, ske_err = 0.5f * (v_dem * v_dem - v * v);
        const float stedot_dem = hdot_dem * g + v * vdot_dem;   // питома швидкість зміни повної енергії
        const float stedot = vz * g + v * vdot_f_;
        const float ste_err = spe_err + ske_err;
        // тяга: тримування + енергетичний баланс
        const float thr_trim = thrustTrim(v), k_ste = mass / (v * tau);   // K_STE2Thr = 1/(τ·K_thr2STE), K_thr2STE = v/m
        integ_thr_ = clampf(integ_thr_ + cfg.i_thr * ste_err * dt * k_ste, -4.0f, 4.0f);
        const float thr = thr_trim + mass * stedot_dem / v + (ste_err + (stedot_dem - stedot) * cfg.thr_damp) * k_ste + integ_thr_;
        thr_out_ = clampf(thr, 0.0f, cfg.thrust_max);
        if (thr != thr_out_) integ_thr_ -= (thr - thr_out_) * 0.02f;   // антивіндап
        // тангаж: баланс енергії SEB = SPE·w_p − SKE·w_k
        const float w_k = clampf(cfg.spd_weight, 0.f, 2.f) > 1 ? 1.f : clampf(cfg.spd_weight, 0.f, 1.f);
        const float w_p = std::min(2.0f - clampf(cfg.spd_weight, 0.f, 2.f), 1.0f);
        const float seb_err = spe_err * w_p - ske_err * w_k;
        const float sebdot_dem = hdot_dem * g * w_p - v * vdot_dem * w_k + seb_err / tau;
        const float sebdot = vz * g * w_p - v * vdot_f_ * w_k;
        integ_seb_ = clampf(integ_seb_ + cfg.i_pitch * seb_err * dt / tau, -0.25f * (cfg.pitch_max - cfg.pitch_min) * v * g, 0.25f * (cfg.pitch_max - cfg.pitch_min) * v * g);
        float gamma = (sebdot_dem + (sebdot_dem - sebdot) * cfg.pitch_damp + integ_seb_) / (v * g);
        gamma_dem_ = clampf(gamma, cfg.pitch_min, cfg.pitch_max);
        pitch_out_ = gamma_dem_ + alphaTrim(v);   // тангаж корпусу = кут траєкторії + α
    }

private:
    float interp(const std::vector<float>& y, float v) const {
        if (v_.size() < 2 || y.size() != v_.size()) return y.empty() ? 0.f : y[0];
        if (v <= v_.front()) return y.front();
        if (v >= v_.back()) return y.back();
        size_t hi = size_t(std::upper_bound(v_.begin(), v_.end(), v) - v_.begin());
        const float f = (v - v_[hi - 1]) / (v_[hi] - v_[hi - 1]);
        return y[hi - 1] + (y[hi] - y[hi - 1]) * f;
    }
    std::vector<float> v_, thr_, alpha_;
    float trim_mass_ = 3.5f, mass_ = 3.5f, alpha_zero_ = -0.068f;   // α0 = −3.9° (нульова підйомна сила, Clark Y)
    float h_dem_f_ = 0, integ_thr_ = 0, integ_seb_ = 0, vdot_f_ = 0, v_prev_ = 0, thr_out_ = 0, pitch_out_ = 0, gamma_dem_ = 0;
    bool first_ = true;
};

// ---------- керми літака ----------
struct FwCmd { float aileron = 0, elevator = 0, rudder = 0; };

class FwController {
public:
    void configure(const QpAirframe& af) {
        af_ = af;
        roll_.setGains(af.gains.fw_roll);
        pitch_.setGains(af.gains.fw_pitch);
    }
    void reset() { roll_.reset(); pitch_.reset(); }
    void presetElevatorTrim(float de) { pitch_.reset(de); }
    // roll_sp — крен (+ правий), pitch_up_sp — тангаж корпусу (+ ніс угору); повертає керми
    FwCmd update(float dt, const QpEstimate& e, float roll_sp, float pitch_up_sp) {
        const Euler eu = eulerOf(e.q);
        const float v = std::fmax(e.airspeed, 6.0f);
        const float scale = clampf((af_.v_cruise * af_.v_cruise) / (v * v), 0.5f, 2.5f);   // ефективність керма ∝ V²
        FwCmd c;
        const float pitch_up = -eu.pitch_down, q_up = -e.omega.y;
        const float p_sp = clampf(3.0f * (roll_sp - eu.roll), -1.6f, 1.6f);
        c.aileron = roll_.update(p_sp * scale, e.omega.x * scale, dt);
        const float q_sp = clampf(2.5f * (pitch_up_sp - pitch_up), -1.0f, 1.0f);
        const float el_up = pitch_.update(q_sp * scale, q_up * scale, dt);   // + означає «ще ніс угору»
        c.elevator = -el_up;                                                 // elevator > 0 — ніс ВНИЗ
        const float r_coord = -(kGravity / v) * std::sin(clampf(eu.roll, -0.9f, 0.9f)) * std::cos(eu.pitch_down);   // ψ̇ = g·tgφ/V, r = ψ̇·cosφ
        c.rudder = clampf(-0.12f * (e.omega.z - r_coord) * scale, -0.43f, 0.43f);
        return c;
    }
private:
    QpAirframe af_;
    Pid roll_, pitch_;
};

}  // namespace fc
