// qp_analysis.h — інструменти аналізу QuadPlane: тримування (крейсер і режим коптера), лінеаризація й власні значення мод,
// поляра потужності, оцінка місії. Усе на тій самій моделі (qp_sim.h), що й симуляція; double, чисельні похідні.
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <functional>
#include <vector>
#include "qp_sim.h"

namespace sim {

// ---------- Ньютон з чисельним якобіаном і зменшенням кроку ----------
inline bool newtonSolve(const std::function<std::vector<double>(const std::vector<double>&)>& f, std::vector<double>& x, double* residual = nullptr,
                        int iters = 80, double tol = 1e-7) {
    const size_t n = x.size();
    auto norm = [](const std::vector<double>& v) { double s = 0; for (double e : v) s += e * e; return std::sqrt(s); };
    std::vector<double> r = f(x);
    for (int it = 0; it < iters; ++it) {
        if (norm(r) < tol) { if (residual) *residual = norm(r); return true; }
        std::vector<std::vector<double>> J(n, std::vector<double>(n + 1));
        for (size_t j = 0; j < n; ++j) {
            std::vector<double> xp = x;
            const double h = 1e-6 * (1 + std::fabs(x[j]));
            xp[j] += h;
            const std::vector<double> rp = f(xp);
            for (size_t i = 0; i < n; ++i) J[i][j] = (rp[i] - r[i]) / h;
        }
        for (size_t i = 0; i < n; ++i) J[i][n] = -r[i];
        bool singular = false;
        for (size_t c = 0; c < n && !singular; ++c) {   // метод Гауса з вибором головного елемента
            size_t p = c;
            for (size_t i = c + 1; i < n; ++i) if (std::fabs(J[i][c]) > std::fabs(J[p][c])) p = i;
            if (std::fabs(J[p][c]) < 1e-14) { singular = true; break; }
            std::swap(J[p], J[c]);
            for (size_t i = c + 1; i < n; ++i) {
                const double m = J[i][c] / J[c][c];
                for (size_t k = c; k <= n; ++k) J[i][k] -= m * J[c][k];
            }
        }
        if (singular) return false;
        std::vector<double> dx(n);
        for (int i = int(n) - 1; i >= 0; --i) {
            double s = J[size_t(i)][n];
            for (size_t k = size_t(i) + 1; k < n; ++k) s -= J[size_t(i)][k] * dx[k];
            dx[size_t(i)] = s / J[size_t(i)][size_t(i)];
        }
        double lambda = 1.0;
        std::vector<double> xn, rn;
        for (int ls = 0; ls < 12; ++ls) {
            xn = x;
            for (size_t i = 0; i < n; ++i) xn[i] += lambda * dx[i];
            rn = f(xn);
            if (std::isfinite(norm(rn)) && norm(rn) < norm(r)) break;
            lambda *= 0.5;
        }
        x = xn; r = rn;
    }
    if (residual) *residual = norm(r);
    return norm(r) < 1e-4;
}

struct TrimResult {
    bool ok = false;
    double V = 0, alpha = 0, pitch_down = 0, elevator = 0, residual = 0;   // рад
    double cruise_rps = 0, cruise_thrust = 0, duty = 0;
    double vtol_rps_front = 0, vtol_rps_rear = 0, vtol_thrust = 0;
    double tilt = 0;                       // рад: нахил гондол (trimTransition)
    double shaft_w = 0, elec_w = 0, ld = 0;
};

namespace detail {

inline QpState levelState(double V, double alpha, const QuadPlaneSim& s) {
    QpState st;
    st.pos = Vec3d{0, 0, 100};
    st.vel_b = Vec3d{V * std::cos(alpha), 0, -V * std::sin(alpha)};
    st.omega = Vec3d{};
    st.q = Quatd::fromEuler(0, -alpha, 0);   // рівний політ: кут тангажа (ніс вниз +) = −α
    (void)s;
    return st;
}

// електрична потужність мотора для заданих обертів ω і моменту Q (усталений режим)
inline double motorBusPower(const MotorEsc& m, double rps, double q, double v_bus) {
    const double omega = rps * 2 * 3.14159265358979323846;
    const double i_ph = q / m.ke() + m.i0();
    const double duty = (m.ke() * omega + i_ph * m.rm()) / v_bus;
    return v_bus * (duty * i_ph / m.escEff() + 0.02);
}
inline double motorDuty(const MotorEsc& m, double rps, double q, double v_bus) {
    const double omega = rps * 2 * 3.14159265358979323846;
    return (m.ke() * omega + (q / m.ke() + m.i0()) * m.rm()) / v_bus;
}

}  // namespace detail

// Рівний політ V (м/с), γ = 0: невідомі — α, кермо висоти, оберти тягнучих гвинтів.
// Конвертоплан: гондоли на 90°, тягнуть усі 4 гвинти (або лише передні, tilt.cruise_rear = 0; задні тоді стоять, гвинти вітряком);
// апарат з маршовим гвинтом — маршовий, VTOL-гвинти стоять.
inline TrimResult trimCruise(QuadPlaneSim& s, double V, double v_bus = 0.0, double rho = 1.225) {
    if (v_bus <= 0) v_bus = s.battery().nominalBusV();
    TrimResult t; t.V = V;
    s.setDensity(rho);
    const bool tilt = s.tiltEnabled();
    if (tilt) s.presetTilt(1.5707963267948966);
    // задні гвинти інші, ніж передні (rearDiffers): автопілот ділить тягу вперед порівну — оберти задніх такі, щоб тяга дорівнювала передній
    const bool rd = tilt && s.rearDiffers();
    auto rearRps = [&](double n, double vax_) {
        const double tf = s.vtolProp(0).eval(n, vax_, 0.0, rho).thrust * s.thrustFactor(0);
        double lo = 1.0, hi = 400.0;
        for (int k = 0; k < 40; ++k) { const double m = 0.5 * (lo + hi); (s.vtolProp(1).eval(m, vax_, 0.0, rho).thrust * s.thrustFactor(1) < tf ? lo : hi) = m; }
        return 0.5 * (lo + hi);
    };
    double vax_now = V;
    auto rpsOf = [&](double n, double r[4]) {
        const double nr = rd ? rearRps(n, vax_now) : n;
        for (int i = 0; i < 4; ++i) r[i] = (i == 0 || i == 2) ? n : (s.cruiseRear() ? nr : 0.0);
    };
    auto resid = [&](const std::vector<double>& x) {
        const double alpha = x[0], de = x[1], n = std::fmax(x[2], 1.0);
        const QpState st = detail::levelState(V, alpha, s);
        if (tilt) {
            vax_now = V * std::cos(alpha);
            double r[4]; rpsOf(n, r);
            s.presetDrive(0, de, 0, 0, r, 0, 0);
        } else {
            const double zero4[4] = {0, 0, 0, 0};
            const Propeller::Out po = s.cruiseProp().eval(n, st.vel_b.x, 0.0, rho);
            s.presetDrive(0, de, 0, n, zero4, po.thrust, po.torque);
        }
        Vec3d F, M;
        s.forcesAndMoments(st, F, M);
        return std::vector<double>{F.x, F.z, M.y};
    };
    std::vector<double> x{0.03, -0.05, tilt ? 90.0 : 100.0};
    t.ok = newtonSolve(resid, x, &t.residual);
    t.alpha = x[0]; t.elevator = x[1]; t.cruise_rps = x[2]; t.pitch_down = -x[0];
    const double vax = V * std::cos(t.alpha);
    if (tilt) {
        vax_now = vax;
        double r[4]; rpsOf(t.cruise_rps, r);
        t.vtol_rps_front = r[0]; t.vtol_rps_rear = r[1];
        t.elec_w = s.avionicsPower();
        for (int i = 0; i < 4; ++i) {
            const Propeller::Out po = s.vtolProp(i).eval(r[i], vax, 0.0, rho);
            if (r[i] <= 0) continue;
            t.cruise_thrust += po.thrust * s.thrustFactor(i); t.shaft_w += po.power;
            t.elec_w += detail::motorBusPower(s.vtolMotor(i), r[i], po.torque, v_bus);
            t.duty = std::fmax(t.duty, detail::motorDuty(s.vtolMotor(i), r[i], po.torque, v_bus));
        }
    } else {
        const Propeller::Out po = s.cruiseProp().eval(t.cruise_rps, vax, 0.0, rho);
        t.cruise_thrust = po.thrust; t.shaft_w = po.power;
        t.duty = detail::motorDuty(s.cruiseMotor(), t.cruise_rps, po.torque, v_bus);
        t.elec_w = detail::motorBusPower(s.cruiseMotor(), t.cruise_rps, po.torque, v_bus) + s.avionicsPower();
    }
    t.ok = t.ok && t.duty < 1.0 && t.duty > 0.0 && t.cruise_rps > 5;
    t.ld = s.mass() * QuadPlaneSim::kG / std::fmax(t.cruise_thrust, 1e-6);
    return t;
}

// Перехід конвертоплана на швидкості V при заданому куті атаки (ніс трохи вгору): невідомі — спільний нахил гондол,
// оберти передніх і задніх гвинтів; рівняння Fx = 0, Fz = 0, My = 0 (кермо висоти 0). tilt_deg — знайдений нахил гондол.
// ok == false, якщо для цього α вага вже повністю на крилі (нахил ≥ 90°) — тоді літак у крейсері (trimCruise).
inline TrimResult trimTransition(QuadPlaneSim& s, double V, double alpha_deg, double v_bus = 0.0, double rho = 1.225) {
    if (v_bus <= 0) v_bus = s.battery().nominalBusV();
    TrimResult t; t.V = V; t.alpha = alpha_deg * 3.14159265358979323846 / 180; t.pitch_down = -t.alpha;
    s.setDensity(rho);
    const Propeller& vp = s.vtolProp();
    auto resid = [&](const std::vector<double>& x) {
        const double th = x[0], nf = std::fmax(x[1], 0.0), nr = std::fmax(x[2], 0.0);
        const QpState st = detail::levelState(V, t.alpha, s);
        s.presetTilt(th);
        const double rps[4] = {nf, nr, nf, nr};
        s.presetDrive(0, 0, 0, 0, rps, 0, 0);
        Vec3d F, M;
        s.forcesAndMoments(st, F, M);
        return std::vector<double>{F.x, F.z, M.y};
    };
    std::vector<double> x{0.6, 100.0, 100.0};
    t.ok = newtonSolve(resid, x, &t.residual);
    t.tilt = x[0]; t.vtol_rps_front = x[1]; t.vtol_rps_rear = x[2];
    const QpState st = detail::levelState(V, t.alpha, s);
    s.presetTilt(t.tilt);
    const double rps[4] = {t.vtol_rps_front, t.vtol_rps_rear, t.vtol_rps_front, t.vtol_rps_rear};
    t.elec_w = s.avionicsPower();
    for (int i = 0; i < 4; ++i) {
        const Vec3d ax = s.rotorAxis(i), va = st.vel_b;
        const double v_ax = va.dot(ax);
        const Propeller::Out po = vp.eval(rps[i], v_ax, (va - ax * v_ax).norm(), rho);
        t.vtol_thrust += po.thrust * s.thrustFactor(i); t.shaft_w += po.power;
        t.elec_w += detail::motorBusPower(s.vtolMotor(), rps[i], po.torque, v_bus);
    }
    t.ok = t.ok && x[0] > -0.2 && x[0] < 1.58 && x[1] > 5 && x[2] > 5;
    return t;
}
// Висіння: гондоли вертикально, тяга кожного гвинта m·g/4 (штовхальним заднім потрібні більші оберти — затінення), без вітру
inline TrimResult hoverPower(const QuadPlaneSim& s, double v_bus = 0.0, double rho = 1.225) {
    if (v_bus <= 0) v_bus = s.battery().nominalBusV();
    TrimResult t;
    const Propeller& vp = s.vtolProp();
    const double w4 = s.mass() * QuadPlaneSim::kG / 4;
    t.elec_w = s.avionicsPower(); t.ok = true;
    for (int i = 0; i < 4; ++i) {
        const double need = w4 / s.thrustFactor(i);
        double n = 0;
        for (double x = 20; x < 300; x += 0.01) if (vp.eval(x, 0, 0, rho).thrust < need) n = x;
        const Propeller::Out po = vp.eval(n, 0, 0, rho);
        (i == 0 || i == 2 ? t.vtol_rps_front : t.vtol_rps_rear) = n;
        t.vtol_thrust += po.thrust * s.thrustFactor(i); t.shaft_w += po.power;
        t.elec_w += detail::motorBusPower(s.vtolMotor(), n, po.torque, v_bus);
        t.duty = std::fmax(t.duty, detail::motorDuty(s.vtolMotor(), n, po.torque, v_bus));
        t.ok = t.ok && n > 5;
    }
    t.ok = t.ok && t.duty < 1;
    return t;
}


// ---------- лінеаризація й моди ----------
struct Mat4 { double a[4][4] = {}; };

inline std::vector<std::complex<double>> eigenvalues4(const Mat4& A) {
    // Фаддєєв–Леверьє для характеристичного многочлена, далі Дюран–Кернер
    double M[4][4] = {}, c[5] = {1, 0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) M[i][i] = 1;
    for (int k = 1; k <= 4; ++k) {
        double AM[4][4] = {};
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) for (int l = 0; l < 4; ++l) AM[i][j] += A.a[i][l] * M[l][j];
        double tr = 0;
        for (int i = 0; i < 4; ++i) tr += AM[i][i];
        c[k] = -tr / k;
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) M[i][j] = AM[i][j] + (i == j ? c[k] : 0.0);
    }
    using C = std::complex<double>;
    std::vector<C> r(4);
    for (int i = 0; i < 4; ++i) r[size_t(i)] = std::pow(C(0.4, 0.9), i);
    auto p = [&](C z) { return (((z + c[1]) * z + c[2]) * z + c[3]) * z + c[4]; };
    for (int it = 0; it < 500; ++it) {
        double change = 0;
        for (int i = 0; i < 4; ++i) {
            C d(1, 0);
            for (int j = 0; j < 4; ++j) if (j != i) d *= (r[size_t(i)] - r[size_t(j)]);
            if (std::abs(d) < 1e-300) d = C(1e-300, 0);
            const C dz = p(r[size_t(i)]) / d;
            r[size_t(i)] -= dz; change = std::fmax(change, std::abs(dz));
        }
        if (change < 1e-13) break;
    }
    return r;
}

struct LinearModes { Mat4 longitudinal, lateral; std::vector<std::complex<double>> long_eig, lat_eig; };

// Лінеаризація навколо рівного польоту (trim від trimCruise): поздовжня [u, w, q, θ] і бокова [v, p, r, φ]
inline LinearModes linearizeCruise(QuadPlaneSim& s, const TrimResult& t, double v_bus = 0.0, double rho = 1.225) {
    if (v_bus <= 0) v_bus = s.battery().nominalBusV();
    (void)v_bus;
    LinearModes lm;
    s.setDensity(rho);
    const double zero4[4] = {0, 0, 0, 0};
    if (s.tiltEnabled()) {
        s.presetTilt(1.5707963267948966);
        double r[4]; for (int i = 0; i < 4; ++i) r[i] = (i == 0 || i == 2 || s.cruiseRear()) ? t.cruise_rps : 0.0;
        s.presetDrive(0, t.elevator, 0, 0, r, 0, 0);
    } else {
        const Propeller::Out po = s.cruiseProp().eval(t.cruise_rps, t.V * std::cos(t.alpha), 0.0, rho);
        s.presetDrive(0, t.elevator, 0, t.cruise_rps, zero4, po.thrust, po.torque);
    }
    const double u0 = t.V * std::cos(t.alpha), w0 = -t.V * std::sin(t.alpha), th0 = t.pitch_down;
    auto accel = [&](double u, double v, double w, double p, double q, double r, double phi, double th, double out[3], double outw[3]) {
        QpState st;
        st.pos = Vec3d{0, 0, 100}; st.vel_b = Vec3d{u, v, w}; st.omega = Vec3d{p, q, r}; st.q = Quatd::fromEuler(phi, th, 0);
        Vec3d dv, dw;
        s.bodyAccel(st, dv, dw);
        out[0] = dv.x; out[1] = dv.y; out[2] = dv.z; outw[0] = dw.x; outw[1] = dw.y; outw[2] = dw.z;
    };
    auto longF = [&](const double x[4], double f[4]) {   // x = u, w, q, θ
        double a[3], aw[3];
        accel(x[0], 0, x[1], 0, x[2], 0, 0, x[3], a, aw);
        f[0] = a[0]; f[1] = a[2]; f[2] = aw[1]; f[3] = x[2];
    };
    auto latF = [&](const double x[4], double f[4]) {   // x = v, p, r, φ
        double a[3], aw[3];
        accel(u0, x[0], w0, x[1], 0, x[2], x[3], th0, a, aw);
        f[0] = a[1]; f[1] = aw[0]; f[2] = aw[2]; f[3] = x[1] + x[2] * std::tan(th0);
    };
    const double x_long[4] = {u0, w0, 0, th0}, x_lat[4] = {0, 0, 0, 0};
    const double steps[4] = {1e-3, 1e-3, 1e-4, 1e-4};
    for (int j = 0; j < 4; ++j) {
        for (int pass = 0; pass < 2; ++pass) {
            double xp[4], xm[4], fp[4], fm[4];
            const double* x0 = pass == 0 ? x_long : x_lat;
            for (int i = 0; i < 4; ++i) xp[i] = xm[i] = x0[i];
            xp[j] += steps[j]; xm[j] -= steps[j];
            if (pass == 0) { longF(xp, fp); longF(xm, fm); } else { latF(xp, fp); latF(xm, fm); }
            for (int i = 0; i < 4; ++i) (pass == 0 ? lm.longitudinal : lm.lateral).a[i][j] = (fp[i] - fm[i]) / (2 * steps[j]);
        }
    }
    lm.long_eig = eigenvalues4(lm.longitudinal);
    lm.lat_eig = eigenvalues4(lm.lateral);
    return lm;
}

// ---------- місія ----------
// Енергія з елементів за потужності навантаження p_load (Вт) протягом seconds: розв'язок P = (OCV − I·R)·I з внутрішнім опором R0 + R1 при SoC 50%
inline double batteryEnergyWh(const AircraftFile& f, double p_load, double seconds) {
    PackBattery b(f); b.setSoc(0.5);
    const double r = b.r0() * (1 + f.num("battery.r1_over_r0")), ocv = b.ocv();
    const double disc = std::fmax(ocv * ocv - 4 * r * p_load, 0.0);
    const double i = (ocv - std::sqrt(disc)) / (2 * r);
    return ocv * i * seconds / 3600.0;
}

struct MissionEstimate {
    bool ok = false;
    double hover_w = 0, cruise_w = 0, transition_w = 0, transition_tilt_deg = 0;
    double total_wh = 0, total_min = 0, usable_wh = 0, endurance_min = 0, range_km = 0;
    std::vector<std::pair<std::string, double>> legs_wh;
};

// Типова місія: зліт, перехід, політ leg_km, висіння 60 с над точкою, політ назад, перехід, посадка
inline MissionEstimate estimateMission(const AircraftFile& file, const AeroTables& tables, double cruise_v = 15.0, double leg_km = 5.0) {
    MissionEstimate m;
    double e = 0, tmin = 0;
    auto leg = [&](const std::string& name, double power_w, double seconds) {
        const double wh = batteryEnergyWh(file, power_w, seconds);   // з урахуванням внутрішнього опору (у висінні ≈9%, у крейсері ≈3%)
        m.legs_wh.emplace_back(name, wh); e += wh; tmin += seconds / 60.0;
    };
    QuadPlaneSim s(file, tables);
    m.hover_w = hoverPower(s).elec_w;
    const TrimResult tq = trimTransition(s, 8.0, 4.0);
    m.transition_w = tq.elec_w; m.transition_tilt_deg = tq.tilt * 180 / 3.14159265358979323846;
    QuadPlaneSim c(file, tables);
    const TrimResult tc = trimCruise(c, cruise_v);
    m.cruise_w = tc.elec_w;
    m.ok = tq.ok && tc.ok;
    leg("зліт і набір висоти (висіння, 40 с)", m.hover_w, 40);
    leg("перехід у літак (~8 м/с, 15 с)", tq.elec_w, 15);
    leg("політ " + std::to_string(int(leg_km)) + " км", tc.elec_w, leg_km * 1000 / cruise_v);
    leg("перехід у коптер (15 с)", tq.elec_w, 15);
    leg("висіння над точкою (60 с)", m.hover_w, 60);
    leg("перехід у літак (15 с)", tq.elec_w, 15);
    leg("політ назад " + std::to_string(int(leg_km)) + " км", tc.elec_w, leg_km * 1000 / cruise_v);
    leg("перехід у коптер (15 с)", tq.elec_w, 15);
    leg("посадка (висіння, 40 с)", m.hover_w, 40);
    m.total_wh = e; m.total_min = tmin;
    m.usable_wh = file.num("battery.usable_wh_target");
    m.endurance_min = 60.0 * m.usable_wh / batteryEnergyWh(file, m.cruise_w, 3600.0);   // тільки крейсер
    m.range_km = m.endurance_min * 60.0 * cruise_v / 1000.0;
    return m;
}

}  // namespace sim
