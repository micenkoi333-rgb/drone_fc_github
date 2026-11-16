// qp_motor_fail.h — керування після повної відмови одного підйомного мотора (квадро X).
// Фізика: без мотора f моменти крену/тангажу врівноважуються лише при T_partner = 0 (мотор на тій самій діагоналі), а дві робочі
// (однакового напрямку обертання) тягнуть по ≈ mg/2 — їхні реактивні моменти не компенсовані, апарат НЕМИНУЧЕ обертається по рисканню.
// Рішення — «relaxed hover» (M. W. Mueller, R. D'Andrea, «Stability and control of a quadrocopter despite the complete loss of one, two,
// or three propellers», ICRA 2014): навмисно крутитись навколо вертикалі й керувати лише напрямом тяги. Стан у тілі x = [p, q, n_x, n_y],
// де n — бажаний напрям тяги (світ) у осях тіла; лінеаризація навколо обертання r̄:
//   ṗ = (Iyy−Izz)·r̄·q/Ixx + τx/Ixx,   q̇ = (Izz−Ixx)·r̄·p/Iyy + τy/Iyy,   ṅx = r̄·n_y − q,   ṅy = −r̄·n_x + p.
// Входи: u1 = T_a − T_b (робоча діагональ, τ = u1·(y_a, −x_a)), u2 = T_partner ≥ 0 (односторонній). Підсилення LQR (дискретне рівняння Ріккаті)
// рахуються наперед для сітки |r̄| і вибираються за виміряною швидкістю рискання. При r̄ → 0 система некерована — тому сітка від 0.5 рад/с.
#pragma once
#include <array>
#include <cmath>
#include <vector>
#include "fc_math.h"

namespace fc {

class SpinController {
public:
    struct Out { float thrust[4] = {0, 0, 0, 0}; Vec3 n_body; float r_used = 0; };

    // failed — індекс мертвого мотора; dx, dy — плечі; inertia — головні моменти; t_max — максимум тяги мотора
    void configure(int failed, float dx, float dy, const Vec3& inertia, float t_max, float dt) {
        failed_ = failed; t_max_ = t_max; partner_ = a_ = b_ = -1;
        const MotorGeom& f = kQuadX[failed];
        for (int i = 0; i < 4; ++i) {
            if (i == failed) continue;
            if (kQuadX[i].yaw_sign == f.yaw_sign) partner_ = i;           // та сама діагональ
            else if (a_ < 0) a_ = i; else b_ = i;
        }
        spin_sign_ = kQuadX[a_].yaw_sign;                                   // реакція робочої пари: M.z += yaw_sign·Q
        const float xa = kQuadX[a_].sx * dx, ya = kQuadX[a_].sy * dy, xp = kQuadX[partner_].sx * dx, yp = kQuadX[partner_].sy * dy;
        I_ = inertia;
        gains_.clear();
        for (int k = 0; k < kGrid; ++k) {
            const double r = spin_sign_ * (kRmin + k * kRstep);
            gains_.push_back(lqr(r, xa, ya, xp, yp, dt));
        }
    }
    bool active() const { return failed_ >= 0; }
    int failed() const { return failed_; }
    int partner() const { return partner_; }
    float spinSign() const { return spin_sign_; }

    // d_world — бажаний напрям тяги (одиничний, світ); thrust_total — потрібна сумарна тяга вздовж нього, Н; q — орієнтація, omega — гіро (тіло)
    Out update(const Quat& q, const Vec3& omega, const Vec3& d_world, float thrust_total) {
        Out o;
        const Vec3 n = q.rotateInv(d_world);
        o.n_body = n;
        const float r_abs = std::fabs(omega.z) * (omega.z * spin_sign_ > 0 ? 1.0f : 0.0f);
        const float kf = clampf((r_abs - kRmin) / kRstep, 0.0f, float(kGrid - 1));
        const int k0 = int(kf), k1 = std::min(k0 + 1, kGrid - 1);
        const float w = kf - float(k0);
        o.r_used = spin_sign_ * (kRmin + kf * kRstep);
        const float x[4] = {omega.x, omega.y, n.x, n.y};
        float u[2] = {0, 0};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 4; ++j) u[i] -= (gains_[size_t(k0)][size_t(i * 4 + j)] * (1 - w) + gains_[size_t(k1)][size_t(i * 4 + j)] * w) * x[j];
        // тяга: сумарна поділена між робочою парою й партнером; диференціал u1 обмежується, щоб жоден мотор не вийшов за [0, t_max]
        const float tp = clampf(u[1], 0.0f, 0.3f * thrust_total);
        const float base = clampf(0.5f * (thrust_total - tp), 0.0f, t_max_);
        const float d = clampf(u[0], -2.0f * std::fmin(base, t_max_ - base), 2.0f * std::fmin(base, t_max_ - base));
        o.thrust[a_] = base + 0.5f * d; o.thrust[b_] = base - 0.5f * d; o.thrust[partner_] = tp; o.thrust[failed_] = 0;
        return o;
    }

private:
    static constexpr int kGrid = 80;
    static constexpr float kRmin = 0.5f, kRstep = 0.5f;   // 0.5…40 рад/с
    int failed_ = -1, partner_ = -1, a_ = -1, b_ = -1;
    float spin_sign_ = 1, t_max_ = 30;
    Vec3 I_;
    std::vector<std::array<float, 8>> gains_;

    // дискретний LQR 4×2: Q = diag(qp, qp, qn, qn), R = diag(r1, r2)
    std::array<float, 8> lqr(double r, double xa, double ya, double xp, double yp, double dt) const {
        const double Ix = I_.x, Iy = I_.y, Iz = I_.z;
        double A[4][4] = {{0, (Iy - Iz) * r / Ix, 0, 0}, {(Iz - Ix) * r / Iy, 0, 0, 0}, {0, -1, 0, r}, {1, 0, -r, 0}};
        double B[4][2] = {{ya / Ix, yp / Ix}, {-xa / Iy, -xp / Iy}, {0, 0}, {0, 0}};
        for (int i = 0; i < 4; ++i) { for (int j = 0; j < 4; ++j) A[i][j] = (i == j ? 1.0 : 0.0) + A[i][j] * dt; for (int j = 0; j < 2; ++j) B[i][j] *= dt; }
        const double Qd[4] = {0.2, 0.2, 40.0, 40.0}, Rd[2] = {1.0 / 36.0, 1.0 / 9.0};
        double P[4][4] = {}; for (int i = 0; i < 4; ++i) P[i][i] = Qd[i];
        double K[2][4] = {};
        for (int it = 0; it < 20000; ++it) {
            double PB[4][2] = {}, PA[4][4] = {};
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 2; ++j) for (int k = 0; k < 4; ++k) PB[i][j] += P[i][k] * B[k][j];
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) for (int k = 0; k < 4; ++k) PA[i][j] += P[i][k] * A[k][j];
            double S[2][2] = {{Rd[0], 0}, {0, Rd[1]}}, BPA[2][4] = {};
            for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) for (int k = 0; k < 4; ++k) S[i][j] += B[k][i] * PB[k][j];
            for (int i = 0; i < 2; ++i) for (int j = 0; j < 4; ++j) for (int k = 0; k < 4; ++k) BPA[i][j] += B[k][i] * PA[k][j];
            const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
            const double Si[2][2] = {{S[1][1] / det, -S[0][1] / det}, {-S[1][0] / det, S[0][0] / det}};
            for (int i = 0; i < 2; ++i) for (int j = 0; j < 4; ++j) K[i][j] = Si[i][0] * BPA[0][j] + Si[i][1] * BPA[1][j];
            double Pn[4][4], diff = 0;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j) {
                    double v = (i == j ? Qd[i] : 0.0);
                    for (int k = 0; k < 4; ++k) v += A[k][i] * PA[k][j];
                    for (int k = 0; k < 2; ++k) v -= (A[0][i] * PB[0][k] + A[1][i] * PB[1][k] + A[2][i] * PB[2][k] + A[3][i] * PB[3][k]) * K[k][j];
                    Pn[i][j] = v; diff = std::fmax(diff, std::fabs(v - P[i][j]));
                }
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) P[i][j] = 0.5 * (Pn[i][j] + Pn[j][i]);
            if (diff < 1e-9 * (1 + std::fabs(P[2][2]))) break;
        }
        std::array<float, 8> g{};
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 4; ++j) g[size_t(i * 4 + j)] = float(K[i][j]);
        return g;
    }
};

}  // namespace fc
