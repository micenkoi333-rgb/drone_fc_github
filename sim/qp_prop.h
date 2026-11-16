// qp_prop.h — гвинт (таблиці CT/CP за осьовим J, Patience & Nahon) і мотор + ESC (модель Kv, Rm, I0) для QuadPlane.
// Гвинт: J = V_осьова/(nD), CT = T/(ρ n² D⁴), CP = P/(ρ n³ D⁵). Косий обдув: за Patience & Nahon (прочитано, їх похибка нижче 5% для більшості
// режимів) поперечна складова на тягу не впливає; поперечна сила H (в площині диска, проти поперечного потоку) — H = k·CT0·ρ·n·D³·V_perp,
// k = [О] (першоджерело Khan/Selig за H не отримано). Зворотний потік і вітряк — лінійна екстраполяція таблиць (CT, CP < 0).
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "aircraft_params.h"

namespace sim {

// лінійна інтерполяція з лінійною екстраполяцією по двох крайніх точках
inline double interpExtrap(const std::vector<double>& x, const std::vector<double>& y, double v) {
    const size_t n = x.size();
    if (n == 0) return 0;
    if (n == 1) return y[0];
    size_t hi = size_t(std::upper_bound(x.begin(), x.end(), v) - x.begin());
    hi = std::clamp<size_t>(hi, 1, n - 1);
    const size_t lo = hi - 1;
    return y[lo] + (y[hi] - y[lo]) * (v - x[lo]) / (x[hi] - x[lo]);
}

// те саме, але без екстраполяції (значення на краях)
inline double interpClamp(const std::vector<double>& x, const std::vector<double>& y, double v) {
    if (x.empty()) return 0;
    return interpExtrap(x, y, std::clamp(v, x.front(), x.back()));
}

class Propeller {
public:
    struct Out { double thrust = 0, torque = 0, power = 0, h_force = 0; };   // Н, Н·м (навантаження на вал), Вт, Н

    Propeller() = default;
    Propeller(const AircraftFile& a, const std::string& section) {
        d_ = a.num(section + ".diameter_m");
        j_ = a.nums(section + ".j_table"); ct_ = a.nums(section + ".ct_table"); cp_ = a.nums(section + ".cp_table");
        if (j_.size() < 2 || j_.size() != ct_.size() || j_.size() != cp_.size()) throw std::runtime_error(section + ": таблиці J, CT, CP різної довжини");
        for (size_t i = 1; i < j_.size(); ++i) if (!(j_[i] > j_[i - 1])) throw std::runtime_error(section + ": J має зростати");
        inertia_ = a.num(section + ".rotor_inertia");
        h_k_ = a.num(section + ".h_force_k");
        j_zero_ = a.has(section + ".j_zero_ct") ? a.num(section + ".j_zero_ct") : j_.back();
        stop_cda_ = a.has(section + ".stopped_cda_m2") ? a.num(section + ".stopped_cda_m2") : 0.0;
    }
    double diameter() const { return d_; }
    double inertia() const { return inertia_; }
    double ct(double j) const { return std::clamp(interpExtrap(j_, ct_, j), -0.15, 1.5 * ct_[0]); }
    double cp(double j) const { return std::clamp(interpExtrap(j_, cp_, j), -0.05, 1.6 * cp_[0] + 0.02); }

    // n — оберти, об/с (≥ 0); v_axial — швидкість гвинта ВЗДОВЖ тяги відносно повітря (м/с, + коли летить у бік тяги); v_perp — поперечна
    Out eval(double n, double v_axial, double v_perp, double rho) const {
        Out o;
        if (!std::isfinite(n + v_axial + v_perp + rho)) return o;
        // гвинт, що стоїть або крутиться повільніше за вільне обертання (J > J_нульової тяги), — опір лопатей у потоці (зупинені задні гвинти в польоті)
        const double v_free = std::fmax(v_axial, 0.0);
        const double stall_frac = v_free > 0.5 ? std::clamp(1.0 - n * d_ * j_zero_ / v_free, 0.0, 1.0) : 0.0;
        const double drag_stop = stall_frac * 0.5 * rho * stop_cda_ * v_free * v_free;
        if (!(n > 0.5)) { o.thrust = -drag_stop; return o; }
        const double j = v_axial / (n * d_);
        const double d2 = d_ * d_, d4 = d2 * d2;
        o.thrust = ct(j) * rho * n * n * d4;
        o.power = cp(j) * rho * n * n * n * d4 * d_;
        o.torque = o.power / (2 * 3.14159265358979323846 * n);
        o.h_force = h_k_ * ct_[0] * rho * n * d2 * d_ * std::fabs(v_perp);
        o.thrust -= drag_stop;
        return o;
    }

private:
    double d_ = 0.254, inertia_ = 2e-5, h_k_ = 0.5, j_zero_ = 0.65, stop_cda_ = 0;
    std::vector<double> j_, ct_, cp_;
};

// Теорія елемента лопаті з рівномірно-лінійним індукованим потоком (Glauert, Drees) — гвинт у косому потоці.
// Лопать: хорда й крутка типового APC SF (крутка — з кроку, β = atan(P/2πr)), профіль — лінійна CL з обмеженням і пластина за зривом.
// Використовується «дельтою» до таблиць UIUC: осьові тяга й момент — з таблиць (виміряні), а косий потік дає множники
// тяги/моменту, силу в площині диска (H і бічну) та моменти від асиметрії лопаті, що йде вперед і назад.
class BemRotor {
public:
    struct Out { double thrust = 0, torque = 0, fx = 0, fy = 0, mx = 0, my = 0, vi = 0; };

    BemRotor() = default;
    BemRotor(const AircraftFile& a, const std::string& sec) {
        r_ = a.num(sec + ".diameter_m") / 2;
        pitch_ = a.num(sec + ".pitch_m");
        if (a.has(sec + ".blades")) blades_ = int(a.num(sec + ".blades"));
        for (int k = 0; k < kNr; ++k) {
            const double x = kRoot + (k + 0.5) * (1 - kRoot) / kNr;
            x_[k] = x;
            chord_[k] = r_ * (0.08 + 0.45 * x * (1 - x) * (1 - 0.35 * x));
            beta_[k] = std::atan(pitch_ / (2 * 3.14159265358979323846 * x * r_)) - kAlphaZero;
        }
        for (int j = 0; j < kNpsi; ++j) { const double p = 2 * 3.14159265358979323846 * (j + 0.5) / kNpsi; sn_[j] = std::sin(p); cs_[j] = std::cos(p); }
    }
    bool valid() const { return r_ > 0; }

    // n — об/с; v_ax — рух гвинта вздовж осі тяги відносно повітря; v_ip — рух у площині диска (уздовж осі x'); spin = +1 — обертання
    // навколо осі тяги проти годинникової (дивлячись з боку, куди тягне), −1 — за. vi_guess — попередній індукований потік (теплий старт).
    Out eval(double n, double v_ax, double v_ip, double rho, int spin, double vi_guess, int iters = 4) const {
        Out o;
        if (!(n > 1.0) || !valid()) return o;
        const double om = 2 * 3.14159265358979323846 * n, area = 3.14159265358979323846 * r_ * r_;
        double vi = std::isfinite(vi_guess) && vi_guess > 0 ? vi_guess : 0.3 * om * r_ * 0.05;
        Out cur;
        for (int it = 0; it < iters; ++it) {
            const double vt = std::sqrt(v_ip * v_ip + (v_ax + vi) * (v_ax + vi));
            const double chi = std::atan2(std::fabs(v_ip), std::fmax(v_ax + vi, 1e-3));
            const double mu = v_ip / (om * r_);
            const double kx = std::clamp(4.0 / 3.0 * (1 - 1.8 * mu * mu) * std::tan(chi / 2), 0.0, 1.5);
            cur = sum(om, v_ax, v_ip, vi, kx, rho);
            const double target = cur.thrust / (2 * rho * area * std::fmax(vt, 0.5));
            vi += 0.6 * (std::clamp(target, -0.5 * std::fabs(v_ax) - 1.0, 40.0) - vi);
        }
        o = cur; o.vi = vi;
        if (spin < 0) { o.fy = -o.fy; o.mx = -o.mx; }
        return o;
    }

private:
    static constexpr int kNr = 8, kNpsi = 12;
    static constexpr double kRoot = 0.15, kAlphaZero = -0.11, kClA = 5.7, kClMax = 1.4, kClMin = -0.7, kCd0 = 0.022, kCdK = 0.035;
    double r_ = 0, pitch_ = 0.11;
    int blades_ = 2;
    double x_[kNr] = {}, chord_[kNr] = {}, beta_[kNr] = {}, sn_[kNpsi] = {}, cs_[kNpsi] = {};

    static void coeffs(double a, double& cl, double& cd) {
        const double lin = kClA * a;
        const double att = std::clamp(lin, kClMin, kClMax);
        const double s2 = std::sin(2 * a), sa = std::sin(a);
        const double w = std::clamp((std::fabs(a) - 0.28) / 0.15, 0.0, 1.0);   // від ≈ 16° — до пластини (на малих Re і з обертанням зрив затягнутий)
        cl = (1 - w) * att + w * s2;
        cd = (1 - w) * (kCd0 + kCdK * att * att) + w * (kCd0 + 1.25 * sa * sa);
    }

    // Сума по лопаті й азимуту; у точці ψ: позиція r(cosψ, sinψ), швидкість лопаті Ωr(−sinψ, cosψ); повітря набігає з боку +x'
    Out sum(double om, double v_ax, double v_ip, double vi0, double kx, double rho) const {
        Out o;
        const double dr = r_ * (1 - kRoot) / kNr, w = double(blades_) / kNpsi;
        for (int j = 0; j < kNpsi; ++j) {
            const double sp = sn_[j], cp = cs_[j];
            for (int k = 0; k < kNr; ++k) {
                const double r = x_[k] * r_;
                const double ut = om * r - v_ip * sp;
                const double up = v_ax + vi0 * (1 - kx * x_[k] * cp);
                const double phi = std::atan2(up, ut);
                double a = beta_[k] - phi;
                a = std::remainder(a, 2 * 3.14159265358979323846);
                double cl, cd;
                coeffs(a, cl, cd);
                const double q = 0.5 * rho * (ut * ut + up * up) * chord_[k] * dr;
                const double dT = q * (cl * std::cos(phi) - cd * std::sin(phi));
                const double dF = q * (cl * std::sin(phi) + cd * std::cos(phi));
                o.thrust += w * dT;
                o.torque += w * r * dF;
                o.fx += w * dF * sp; o.fy += -w * dF * cp;
                o.mx += w * r * sp * dT; o.my += -w * r * cp * dT;
            }
        }
        return o;
    }
};

// Мотор постійного струму + ESC. Керування — коефіцієнт заповнення duty ∈ [0, 1].
// esc_damped = 1: активне гальмування (complementary PWM) — при duty·V < ke·ω струм фази від'ємний (рекуперація), мотор гальмується;
// esc_damped = 0 або мертвий ESC (alive = false) — вибіг: струм фази не буває від'ємним.
class MotorEsc {
public:
    MotorEsc() = default;
    MotorEsc(const AircraftFile& a, const std::string& section, double rotor_inertia) : inertia_(rotor_inertia) {
        kv_ = a.num(section + ".kv"); rm_ = a.num(section + ".rm_ohm"); i0_ = a.num(section + ".i0_a");
        i_max_ = a.num(section + ".max_cont_a") * 1.35;   // піковий струм ESC ≈ 1.35 × тривалий (для 2810 F100: 61.65 / 45.6)
        esc_eff_ = a.num(section + ".esc_eff");
        damped_ = a.has(section + ".esc_damped") ? a.num(section + ".esc_damped") > 0.5 : false;
        ke_ = 60.0 / (2 * 3.14159265358979323846 * kv_);
        rm0_ = rm_;
        if (a.has(section + ".heat_capacity_j_k")) { heat_c_ = a.num(section + ".heat_capacity_j_k"); cool_ = a.num(section + ".cooling_w_k"); }
        if (a.has(section + ".ambient_c")) temp_ = amb_ = a.num(section + ".ambient_c");
    }
    double omega() const { return omega_; }
    double rps() const { return omega_ / (2 * 3.14159265358979323846); }
    void setOmega(double w) { omega_ = std::fmax(0.0, w); }
    double ke() const { return ke_; }
    double temperature() const { return temp_; }   // °C обмотки
    double rm() const { return rm_; }
    double i0() const { return i0_; }
    double escEff() const { return esc_eff_; }

    // Прогноз струму з батареї при заданих duty й напрузі шини (без зміни стану) — для неявного розв'язання «мотори ↔ батарея»
    double predictBusCurrent(double duty, double v_bus, bool alive = true) const {
        duty = std::clamp(duty, 0.0, 1.0);
        const double i_ph = std::clamp((duty * v_bus - ke_ * omega_) / rm_, (damped_ && alive) ? -i_max_ : 0.0, i_max_);
        return duty * i_ph * (i_ph < 0 ? esc_eff_ : 1.0 / esc_eff_) + 0.02;   // рекуперація — з втратами ESC, а не з «підсиленням»
    }

    struct Step { double motor_torque = 0, phase_current = 0, bus_current = 0, shaft_power = 0; };
    // q_load — момент гвинта на валу (Н·м); v_bus — напруга шини
    Step step(double dt, double duty, double v_bus, double q_load, bool alive = true) {
        Step s;
        duty = std::clamp(duty, 0.0, 1.0);
        double i_ph = (duty * v_bus - ke_ * omega_) / rm_;
        i_ph = std::clamp(i_ph, (damped_ && alive && omega_ > 1.0) ? -i_max_ : 0.0, i_max_);
        s.phase_current = i_ph;
        s.motor_torque = ke_ * (i_ph - (omega_ > 1.0 ? i0_ : 0.0));
        omega_ = std::fmax(0.0, omega_ + dt * (s.motor_torque - q_load) / inertia_);
        s.shaft_power = s.motor_torque * omega_;
        s.bus_current = duty * i_ph * (i_ph < 0 ? esc_eff_ : 1.0 / esc_eff_) + 0.02;   // втрати ESC + власний струм
        if (heat_c_ > 0) {   // нагрів обмотки I²R, охолодження (сильніше на обертах — обдув); опір міді +0.39 %/°C
            const double cool = cool_ * (1 + omega_ / 400.0);
            temp_ += dt * (i_ph * i_ph * rm_ - cool * (temp_ - amb_)) / heat_c_;
            rm_ = rm0_ * (1 + 0.0039 * (temp_ - 25));
        }
        return s;
    }

private:
    double kv_ = 1000, rm_ = 0.06, i0_ = 1.0, i_max_ = 60, esc_eff_ = 0.96, ke_ = 0.01, inertia_ = 2e-5, omega_ = 0;
    double rm0_ = 0.06, heat_c_ = 0, cool_ = 0.3, temp_ = 25, amb_ = 25;
    bool damped_ = false;
};

}  // namespace sim
