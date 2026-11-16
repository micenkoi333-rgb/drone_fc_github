// estimator.h — оцінка орієнтації (фільтр Маґоні) і висоти (комплементарний фільтр 3-го порядку)
#pragma once
#include "fc_math.h"

namespace fc {

// Фільтр Маґоні: гіроскоп інтегрується, акселерометр повільно «тягне» нахил до вертикалі,
// інтегральна частина оцінює дрейф (bias) гіроскопа по roll/pitch.
class AttitudeEstimator {
public:
    struct Params {
        float kp = 0.4f;         // наскільки сильно довіряємо акселерометру
        float ki = 0.05f;        // швидкість оцінки дрейфу гіроскопа
        // Довіра до акселерометра падає до 0, коли |a| відрізняється від g більше ніж на acc_gate
        // (у польоті з нахилом/маневрами акселерометр міряє тягу, а не гравітацію)
        float acc_gate = 0.10f;
        float acc_lpf_hz = 5.0f;        // фільтр вібрацій перед корекцією
        float max_rate_for_acc = 3.0f;  // рад/с: під час швидких обертів акселерометру не віримо
        float kp_mag = 0.5f;            // 1/с, швидкість корекції курсу компасом
        float ki_mag = 0.02f;           // швидкість оцінки дрейфу гіроскопа по курсу
    };
    AttitudeEstimator() = default;
    explicit AttitudeEstimator(const Params& p) : p_(p) {}

    void reset(const Vec3& accel) {
        float roll = std::atan2(accel.y, accel.z);
        float pitch = std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
        q_ = Quat::fromEuler(roll, pitch, 0.0f);
        integ_ = Vec3{};
        acc_f_ = accel;
        init_ = true;
        mag_init_ = false;
    }

    void update(const Vec3& gyro, const Vec3& accel, float dt) {
        if (!init_) reset(accel);
        Vec3 w = gyro;
        float rc = 1.0f / (2 * kPi * p_.acc_lpf_hz);
        acc_f_ += (accel - acc_f_) * (dt / (rc + dt));
        float an = acc_f_.norm();
        float dev = std::fabs(an / kGravity - 1.0f);
        acc_weight_ = clampf(1.0f - dev / p_.acc_gate, 0.0f, 1.0f);
        if (gyro.norm() > p_.max_rate_for_acc) acc_weight_ = 0;
        if (acc_weight_ > 0 && an > 1e-3f) {
            Vec3 a = acc_f_ / an;
            Vec3 v = q_.rotateInv(Vec3{0, 0, 1});  // очікуваний «верх» у системі тіла
            Vec3 e = a.cross(v) * acc_weight_;
            // дрейф вчимо лише коли акселерометру можна вірити; по осі Z тіла (курс) акселерометр майже нічого
            // не бачить — там дрейф вчить компас, інакше він би «гуляв» і повільно розкручував курс
            if (acc_weight_ > 0.5f) { integ_.x += e.x * p_.ki * dt; integ_.y += e.y * p_.ki * dt; }
            const float lim = 0.1f;  // макс. оцінка дрейфу ~5.7 °/с
            integ_ = {clampf(integ_.x, -lim, lim), clampf(integ_.y, -lim, lim), clampf(integ_.z, -lim, lim)};
            w += e * p_.kp;
        }
        acc_used_ = acc_weight_ > 0;
        w += integ_;
        rates_ = gyro + integ_;
        q_.integrate(w, dt);
    }

    // Корекція курсу по компасу. ref_world — напрям поля у світі (північ +Y з нахилом вниз).
    void updateMag(const Vec3& mag_body, const Vec3& ref_world, float dt) {
        Vec3 mw = q_.rotate(mag_body);
        Vec3 mh{mw.x, mw.y, 0}, rh{ref_world.x, ref_world.y, 0};
        if (mh.norm() < 1e-6f || rh.norm() < 1e-6f) return;
        mh = mh.normalized(); rh = rh.normalized();
        float err = std::atan2(rh.x * mh.y - rh.y * mh.x, rh.dot(mh));  // наскільки оцінка курсу «перекрутила»
        if (!mag_init_) {  // перший відлік — одразу виставляємо курс
            q_ = Quat::fromEuler(0, 0, -err) * q_;
            q_.normalize();
            mag_init_ = true;
            return;
        }
        q_ = Quat::fromEuler(0, 0, -err * clampf(p_.kp_mag * dt, 0, 1)) * q_;
        q_.normalize();
        // дрейф гіроскопа по курсу: похибку курсу у світі переводимо в систему тіла
        Vec3 eb = q_.rotateInv(Vec3{0, 0, -err});
        integ_ += eb * (p_.ki_mag * dt);
        const float lim = 0.1f;
        integ_ = {clampf(integ_.x, -lim, lim), clampf(integ_.y, -lim, lim), clampf(integ_.z, -lim, lim)};
    }

    const Quat& q() const { return q_; }
    Vec3 euler() const { return q_.euler(); }
    Vec3 rates() const { return rates_; }  // кутові швидкості з корекцією дрейфу
    Vec3 gyroBias() const { return -integ_; }
    bool accUsed() const { return acc_used_; }
    bool initialized() const { return init_; }

private:
    Params p_;
    Quat q_;
    Vec3 integ_, rates_, acc_f_;
    float acc_weight_ = 0;
    bool init_ = false, acc_used_ = false, mag_init_ = false;
};

// Висота: прогноз по акселерометру (швидко) + корекція по барометру (повільно, без дрейфу)
class AltitudeEstimator {
public:
    explicit AltitudeEstimator(float tau = 0.8f) { setTau(tau); }
    void setTau(float tau) { k1_ = 3 / tau; k2_ = 3 / (tau * tau); k3_ = 1 / (tau * tau * tau); }
    void reset(float z) { z_ = z; vz_ = 0; ab_ = 0; baro_ = z; have_baro_ = true; }

    // az_world: вертикальне прискорення без гравітації (м/с²)
    void update(float az_world, float dt) {
        if (have_baro_) {
            float e = baro_ - z_;
            ab_ += k3_ * e * dt;
            vz_ += k2_ * e * dt;
            z_ += k1_ * e * dt;
        }
        float a = az_world + ab_;
        z_ += vz_ * dt + 0.5f * a * dt * dt;
        vz_ += a * dt;
    }
    void baro(float alt) { baro_ = alt; have_baro_ = true; }
    float z() const { return z_; }
    float vz() const { return vz_; }
    float accelBias() const { return ab_; }
private:
    float k1_, k2_, k3_;
    float z_ = 0, vz_ = 0, ab_ = 0, baro_ = 0;
    bool have_baro_ = false;
};

// Горизонтальна позиція/швидкість: прогноз по акселерометру + корекція по GPS (позиція і швидкість)
class PositionEstimator {
public:
    explicit PositionEstimator(float tau = 1.0f) { setTau(tau); }
    void setTau(float tau) { k1_ = 3 / tau; k2_ = 3 / (tau * tau); k3_ = 1 / (tau * tau * tau); }
    void reset(const Vec3& p, const Vec3& v) { p_ = {p.x, p.y, 0}; v_ = {v.x, v.y, 0}; ab_ = Vec3{}; gp_ = p_; gv_ = v_; have_ = true; }
    void gps(const Vec3& p, const Vec3& v) { gp_ = {p.x, p.y, 0}; gv_ = {v.x, v.y, 0}; have_ = true; }
    void lose() { have_ = false; }
    void setFlowGain(float k) { kvf_ = k; }
    // Лише швидкість (оптичний потік): позиція рахується інтегруванням і повільно «пливе», зате без GPS
    void velocity(const Vec3& v) { fv_ = {v.x, v.y, 0}; have_vel_ = true; vel_age_ = 0; }
    void loseVelocity() { have_vel_ = false; }
    void update(const Vec3& a_world, float dt) {
        Vec3 a{a_world.x, a_world.y, 0};
        vel_age_ += dt;
        const bool vel = have_vel_ && vel_age_ < 0.2f;
        if (vel) {  // швидкість з оптичного потоку: точна на короткому відрізку
            Vec3 ev = fv_ - v_;
            ab_ += ev * (0.4f * dt);
            v_ += ev * (kvf_ * dt);
        }
        if (have_ && vel) {
            // GPS + потік разом: швидкість беремо з потоку (вона не «блукає»), а GPS лише повільно
            // підтягує позицію, щоб не накопичувався дрейф. Дрон не смикається за шумом GPS.
            Vec3 ep = gp_ - p_;
            p_ += ep * (kf1_ * dt);
            v_ += ep * (kf2_ * dt);
        } else if (have_) {
            Vec3 ep = gp_ - p_, ev = gv_ - v_;
            ab_ += ep * (k3_ * dt);
            v_ += (ep * k2_ + ev * kv_) * dt;
            p_ += ep * (k1_ * dt);
        }
        a += ab_;
        p_ += v_ * dt + a * (0.5f * dt * dt);
        v_ += a * dt;
    }
    Vec3 pos() const { return p_; }
    Vec3 vel() const { return v_; }
private:
    float k1_, k2_, k3_, kv_ = 2.0f, kvf_ = 15.0f;
    float kf1_ = 2.0f / 8.0f, kf2_ = 1.0f / 64.0f;  // GPS у парі з потоком: стала ~8 с
    float vel_age_ = 1e9f;
    Vec3 p_, v_, ab_, gp_, gv_, fv_;
    bool have_ = false, have_vel_ = false;
};

}  // namespace fc
