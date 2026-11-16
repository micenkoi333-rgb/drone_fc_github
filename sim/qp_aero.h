// qp_aero.h — компонентна аеродинаміка QuadPlane (за схемою Селіга, J. Aircraft 2014, Eq.(1)–(5), прочитано в тексті):
// крило, стабілізатор і кілі — смуги з таблицями cl/cd/cm(α) на −180…180°; індукований кут крила береться з таблиць
// нелінійної підйомної лінії (будуються при завантаженні); скіс потоку на стабілізаторі — закон Біо–Савара від кожної
// смуги крила; корпус, балки, шасі й субдрони — квадратичний опір по осях.
// Керування: aileron > 0 — правий крен (+X), elevator > 0 — ніс ВНИЗ (+Y), rudder > 0 — ніс ліворуч (+Z); рад кута поверхні.
// Динамічний зрив (гістерезис): частка відриву w кожної смуги — стан із запізненням за Гоманом–Храбровим (τ1 ẇ = w_ст(α − τ2·α̇) − w,
// τ1 = 3c/V, τ2 = 1.5c/V [О]); оновлюється раз на крок фізики (advance), у межах кроку RK4 заморожена.
// Ефект землі для крила: індукований кут ×φ(h/b) = (16h/b)²/(1+(16h/b)²) (Візельсбергер/МакКормік) для кожної смуги за її висотою.
// Затінення кіля фюзеляжем: підвітряний кіль при ковзанні β бачить q·(1 − k·min(|β|/20°, 1)) [О, fin_shadow_k].
// Re-залежність таблиць не моделюється (Селіг теж: «no interpolation on Reynolds number»).
#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <mutex>
#include <array>
#include <string>
#include <vector>
#include "aero_tables.h"
#include "aircraft_params.h"
#include "vec3d.h"

namespace sim {

enum class Ctrl { None, Aileron, Elevator, Rudder, VTail };   // VTail: руль V-хвоста — Δα = ge·висота + gr·напрям

struct Strip {
    Vec3d r, fwd, nrm;   // чверть хорди відносно ЦМ (тіло); напрямок вперед по хорді; нормаль, куди діє додатна підйомна сила
    double chord = 0, width = 0, ctrl_gain = 0;   // ctrl_gain: Δα смуги (рад) на 1 рад команди (знак і ефективність враховано)
    double twist = 0;                              // закрутка смуги відносно кореня, рад (для підйомної лінії)
    double ge = 0, gr = 0;                         // лише Ctrl::VTail: частки команд висоти й напряму (ctrl_gain = 1)
    Ctrl ctrl = Ctrl::None;
};

struct Surface {
    std::string name;
    std::vector<Strip> strips;
    const AirfoilTable* table = nullptr;
    double ar = 1, oswald = 0.9;
    double re_ref = 0;                             // число Рейнольдса таблиці профілю (0 — без поправки на Re)
    std::vector<double> eps_alpha;                 // сітка α для таблиці індукованого кута, рад (лише крило)
    std::vector<std::vector<double>> eps;          // [смуга][точка]
};

struct AeroInputs {
    Vec3d v_body;                // швидкість ЦМ відносно повітря в осях тіла, м/с
    Vec3d omega;                 // кутова швидкість тіла, рад/с
    double rho = 1.225;
    double aileron = 0, elevator = 0, rudder = 0;
    std::function<Vec3d(const Vec3d&)> wash;   // додаткова швидкість ПОВІТРЯ в точці (тіло, відн. ЦМ), напр. слід гвинта
    double h_agl = 1e9;                      // висота ЦМ над землею, м (для ефекту землі; 1e9 — землі немає)
    Vec3d up_body{0, 0, 1};                  // вертикаль світу в осях тіла (висота кожної смуги)
};

struct AeroResult {
    Vec3d force, moment;                        // у тілі, момент відносно ЦМ
    Vec3d wing_force, stab_force, fin_force, body_force, rear_force;   // rear_force — заднє крило (схема «два крила»)
    double tail_downwash_rad = 0;               // середній кут скосу потоку на стабілізаторі
    double wing_cl = 0, wing_alpha = 0;         // діагностика: підйомна сила крила / (qS), α набігання
};

class QuadPlaneAero {
public:
    QuadPlaneAero(const AircraftFile& a, const AeroTables& t, const AircraftDerived& d) {
        wing_table_ = &t.get(a.str("aero_model.wing_table"));
        tail_table_ = &t.get(a.str("aero_model.tail_table"));
        derate_ = a.num("controls.effectiveness_derate");
        cm_per_cl_ = a.num("controls.flap_cm_per_cl");
        flap_drag_ = a.num("controls.flap_drag_coeff");
        downwash_min_v_ = a.num("aero_model.tail_downwash_min_mps");
        stab_q_ratio_ = a.num("aero.tail_efficiency");   // η_h: стабілізатор у сліді крила й корпусу
        fin_shadow_ = a.has("aero_model.fin_shadow_k") ? a.num("aero_model.fin_shadow_k") : 0.0;   // лише H-хвіст із кілями
        dyn_stall_ = a.has("aero_model.dynamic_stall") ? a.num("aero_model.dynamic_stall") > 0.5 : false;
        if (a.has("aero_model.wake_core_m")) wake_core_ = a.num("aero_model.wake_core_m");
        if (a.has("aero_model.apply_oswald") && a.num("aero_model.apply_oswald") > 0.5) oswald_extra_ = 1.0 / a.num("aero.oswald_e") - 1.0;
        span_ = a.num("geometry.wing_span_m");
        cda_ = {a.num("drag.cda_x_m2"), a.num("drag.cda_y_m2"), a.num("drag.cda_z_m2")};
        cp_ = {0, 0, a.num("drag.cp_z_m")};
        if (a.has("drag.munk_volume_m3")) { munk_vol_ = a.num("drag.munk_volume_m3"); munk_k_ = a.has("drag.munk_k") ? a.num("drag.munk_k") : 0.9; }
        const Vec3d cg{d.cg_x, d.cg_y, d.cg_z};
        buildWing(a, d, cg);
        buildTail(a, d, cg);
        buildRearWing(a, cg);
        buildLiftingLine();
        if (a.has("aero_model.wing_table_re")) { wing_.re_ref = a.num("aero_model.wing_table_re"); rw_.re_ref = wing_.re_ref; }
        if (a.has("aero_model.tail_table_re")) { stab_.re_ref = a.num("aero_model.tail_table_re"); fin_.re_ref = stab_.re_ref; if (rw_sym_) rw_.re_ref = stab_.re_ref; }
        for (int k = 0; k < 4; ++k) dyn_[size_t(k)].assign(surf(k).strips.size(), DynState{});
    }

    // Динамічний зрив: оновити стан відриву смуг за кроком dt при вхідних умовах in (викликати раз на крок фізики)
    void advance(double dt, const AeroInputs& in) {
        if (!dyn_stall_ || dt <= 0) return;
        recordBegin(); (void)eval(in); advanceRecorded(dt);
    }
    void recordBegin() {
        if (!dyn_stall_) return;
        for (int k = 0; k < 4; ++k) { rec_alpha_[size_t(k)].assign(surf(k).strips.size(), 0.0); rec_vr_[size_t(k)].assign(surf(k).strips.size(), 0.0); }
        rec_ = true;
    }
    void advanceRecorded(double dt) {
        const bool had = rec_; rec_ = false;
        if (!dyn_stall_ || dt <= 0 || !had) return;
        for (int k = 0; k < 4; ++k) {
            const Surface* sf = &surf(k);
            std::vector<DynState>& st = dyn_[size_t(k)];
            for (size_t j = 0; j < sf->strips.size(); ++j) {
                const Strip& s = sf->strips[j];
                const double vr = rec_vr_[size_t(k)][j], a = rec_alpha_[size_t(k)][j];
                DynState& d = st[j];
                if (!d.init || vr < 2.0) { d = DynState{true, sf->table->eval(a).w_plate, a}; continue; }
                const double adot = std::remainder(a - d.alpha_prev, 2 * 3.14159265358979323846) / dt;
                d.alpha_prev = a;
                const double t1 = 3.0 * s.chord / vr, t2 = 1.5 * s.chord / vr;
                const double w_st = sf->table->eval(a - t2 * std::clamp(adot, -20.0, 20.0)).w_plate;
                d.w += (w_st - d.w) * (1 - std::exp(-dt / t1));
            }
        }
    }
    void setDynamicStall(bool on) { dyn_stall_ = on; for (auto& v : dyn_) for (DynState& d : v) d.init = false; }
    bool dynamicStall() const { return dyn_stall_; }
    double stripSeparation(size_t j) const { return dyn_[0][j].w; }   // частка відриву смуги крила (діагностика)

    AeroResult eval(const AeroInputs& in) const {
        AeroResult out;
        if (!in.v_body.finite() || !in.omega.finite() || !std::isfinite(in.rho) || in.rho <= 0) return out;
        const double V = in.v_body.norm();
        // --- крило
        std::vector<double> gamma(wing_.strips.size(), 0.0);
        double lift_sum = 0, area_sum = 0;
        for (size_t j = 0; j < wing_.strips.size(); ++j) {
            const Strip& s = wing_.strips[j];
            const Vec3d vs = stripVelocity(s, in, in.wash ? in.wash(s.r) : Vec3d{});
            const double dg = ctrlAngle(s, in);
            StripOut o = stripForce(s, vs, dg, wing_, true, in.rho, j, groundFactor(s, in));
            out.wing_force += o.force; out.moment += s.r.cross(o.force) + o.moment;
            gamma[j] = o.gamma;
            lift_sum += o.cl * s.chord * s.width; area_sum += s.chord * s.width;
        }
        out.wing_cl = area_sum > 0 ? lift_sum / area_sum : 0;
        if (oswald_extra_ > 0 && V > 1.0 && area_sum > 0) {   // індукований опір понад ідеальну підйомну лінію (корпус, пілони, стики): CL²/(π·AR)·(1/e − 1)
            const double cl = out.wing_cl, dq = 0.5 * in.rho * V * V * area_sum * cl * cl / (3.14159265358979323846 * wing_.ar) * oswald_extra_;
            const Vec3d df = in.v_body * (-dq / V);
            out.wing_force += df;
        }
        out.wing_alpha = V > 0.1 ? std::atan2(-in.v_body.z, in.v_body.x) : 0;
        // --- скіс потоку від крила на стабілізатор
        const bool wake_on = V > downwash_min_v_;
        const Vec3d wake_dir = wake_on ? (in.v_body * -1.0).normalized() : Vec3d{-1, 0, 0};
        double eps_sum = 0;
        for (size_t j = 0; j < stab_.strips.size(); ++j) {
            const Strip& s = stab_.strips[j];
            Vec3d w_air = wake_on ? wingInduced(s.r, gamma, wake_dir) : Vec3d{};
            if (in.wash) w_air += in.wash(s.r);
            eps_sum += wake_on ? std::atan2(-w_air.dot(s.nrm), std::fmax(V, 1e-3)) : 0;
            const Vec3d vs = stripVelocity(s, in, w_air);
            StripOut o = stripForce(s, vs, ctrlAngle(s, in), stab_, false, in.rho * stab_q_ratio_, j);
            out.stab_force += o.force; out.moment += s.r.cross(o.force) + o.moment;
        }
        out.tail_downwash_rad = stab_.strips.empty() ? 0 : eps_sum / double(stab_.strips.size());
        // --- заднє крило: у скосі потоку й сліді основного крила (η) та в струменях гвинтів; руль висоти — разом із V-хвостом
        for (size_t j = 0; j < rw_.strips.size(); ++j) {
            const Strip& s = rw_.strips[j];
            Vec3d w_air = wake_on ? wingInduced(s.r, gamma, wake_dir) : Vec3d{};
            if (in.wash) w_air += in.wash(s.r);
            const Vec3d vs = stripVelocity(s, in, w_air);
            StripOut o = stripForce(s, vs, ctrlAngle(s, in), rw_, false, in.rho * stab_q_ratio_, j);
            out.rear_force += o.force; out.moment += s.r.cross(o.force) + o.moment;
        }
        // --- кілі
        for (size_t j = 0; j < fin_.strips.size(); ++j) {
            const Strip& s = fin_.strips[j];
            const Vec3d w_air = in.wash ? in.wash(s.r) : Vec3d{};
            const Vec3d vs = stripVelocity(s, in, w_air);
            double rho_f = in.rho;
            if (fin_shadow_ > 0 && s.r.y * in.v_body.y < 0) {   // підвітряний кіль (повітря набігає з боку протилежного кіля) — у сліді корпусу
                const double beta = std::atan2(std::fabs(in.v_body.y), std::fmax(std::fabs(in.v_body.x), 1e-3));
                rho_f *= 1 - fin_shadow_ * std::fmin(beta / 0.349, 1.0);
            }
            StripOut o = stripForce(s, vs, ctrlAngle(s, in), fin_, false, rho_f, j);
            out.fin_force += o.force; out.moment += s.r.cross(o.force) + o.moment;
        }
        // --- корпус, балки, шасі, субдрони: F_i = −½ρ|V|·CdA_i·V_i, прикладено нижче ЦМ
        out.body_force = Vec3d{-0.5 * in.rho * std::fabs(in.v_body.x) * cda_.x * in.v_body.x, -0.5 * in.rho * std::fabs(in.v_body.y) * cda_.y * in.v_body.y,
                               -0.5 * in.rho * std::fabs(in.v_body.z) * cda_.z * in.v_body.z};   // по осях за власною складовою (однаковий закон; без фіктивної «підйомної» лінійності)
        out.moment += cp_.cross(out.body_force);
        if (munk_vol_ > 0)   // момент Мунка корпусу й гондол (тіло обертання в потоці): дестабілізує тангаж і рискання, ∝ sin 2α
            out.moment += in.v_body.cross(Vec3d{1, 0, 0}) * (munk_k_ * in.rho * munk_vol_ * in.v_body.x);
        out.force = out.wing_force + out.stab_force + out.fin_force + out.rear_force + out.body_force;
        if (!out.force.finite() || !out.moment.finite()) return AeroResult{};
        return out;
    }

    void setBodyCda(const Vec3d& cda) { cda_ = cda; }   // сумарні CdA корпусу (зменшуються при скиданні субдронів)
    const Vec3d& bodyCda() const { return cda_; }
    // найбільша нев'язка γ підйомної лінії серед α ∈ [lo, hi] (рад); велика — ітерації не зійшлися (зазвичай за зривом)
    double liftingLineResidual(double lo, double hi) const {
        double m = 0;
        for (size_t i = 0; i < wing_.eps_alpha.size(); ++i)
            if (wing_.eps_alpha[i] >= lo && wing_.eps_alpha[i] <= hi) m = std::fmax(m, ll_resid_[i]);
        return m;
    }
    const Surface& wing() const { return wing_; }
    double llResidualMax() const { double m = 0; for (double r : ll_resid_) m = std::fmax(m, r); return m; }   // найбільша нев'язка підйомної лінії (м²/с·…): має бути ≈ 0
    const Surface& stabilizer() const { return stab_; }
    const Surface& fins() const { return fin_; }
    const Surface& rearWing() const { return rw_; }   // порожня, якщо заднього крила немає

private:
    struct StripOut { Vec3d force, moment; double cl = 0, gamma = 0; };
    struct DynState { bool init = false; double w = 0, alpha_prev = 0; };
    mutable std::array<std::vector<double>, 4> rec_alpha_, rec_vr_;   // запис ефективного кута для advance()
    mutable bool rec_ = false;
    std::array<std::vector<DynState>, 4> dyn_;   // крило, стабілізатор, кілі, заднє крило (за індексом — об'єкт можна копіювати)
    const Surface& surf(int k) const { return k == 0 ? wing_ : (k == 1 ? stab_ : (k == 2 ? fin_ : rw_)); }
    int surfIndex(const Surface& sf) const { return &sf == &wing_ ? 0 : (&sf == &stab_ ? 1 : (&sf == &fin_ ? 2 : 3)); }
    bool dyn_stall_ = false, rw_sym_ = false;
    double oswald_extra_ = 0;   // 1/e − 1, якщо aero_model.apply_oswald = 1
    double wake_core_ = 0.02;   // м: радіус ядра вихорів сліду крила (оперення в площині сліду потребує ядра ≈ кроку смуг)
    double fin_shadow_ = 0, span_ = 1.6, munk_vol_ = 0, munk_k_ = 0.9;

    Surface wing_, stab_, fin_, rw_;
    const AirfoilTable *wing_table_ = nullptr, *tail_table_ = nullptr;
    double derate_ = 0.8, cm_per_cl_ = 0.35, flap_drag_ = 1.0, downwash_min_v_ = 3.0, stab_q_ratio_ = 1.0;
    Vec3d cda_, cp_;
    std::vector<double> ll_resid_;

    static double flapTau(double cf) {   // тонкопрофільна ефективність керма τ(cf/c)
        const double th = std::acos(2 * cf - 1);
        return 1 - (th - std::sin(th)) / 3.14159265358979323846;
    }

    void buildWing(const AircraftFile& a, const AircraftDerived& d, const Vec3d& cg) {
        wing_.name = "wing"; wing_.table = wing_table_;
        const double b = a.num("geometry.wing_span_m"), c = a.num("geometry.wing_chord_m");
        const double c_tip = a.has("geometry.wing_tip_chord_m") ? a.num("geometry.wing_tip_chord_m") : c;
        const double twist = a.has("geometry.wing_tip_twist_deg") ? a.num("geometry.wing_tip_twist_deg") * 3.14159265358979323846 / 180 : 0.0;   // закрутка кінця (− — менший кут)
        const int n = 2 * int(a.num("aero_model.wing_strips_per_half"));
        const double inc0 = a.num("geometry.wing_incidence_deg") * 3.14159265358979323846 / 180;
        wing_.ar = d.aspect; wing_.oswald = a.num("aero.oswald_e");
        const double tau = flapTau(a.num("controls.aileron_chord_frac")) * derate_;
        const double in_f = a.num("controls.aileron_inner_frac"), out_f = a.num("controls.aileron_outer_frac");
        for (int j = 0; j < n; ++j) {
            Strip s;
            const double y = -b / 2 + (j + 0.5) * b / n;
            const double f = std::fabs(y) / (b / 2), inc = inc0 + twist * f;
            s.r = Vec3d{d.wing_ac_x, y, a.num("geometry.wing_z_m")} - cg;
            s.fwd = Vec3d{std::cos(inc), 0, std::sin(inc)};
            s.nrm = Vec3d{-std::sin(inc), 0, std::cos(inc)};
            s.chord = c + (c_tip - c) * f; s.width = b / n; s.twist = twist * f;
            if (f >= in_f && f <= out_f) { s.ctrl = Ctrl::Aileron; s.ctrl_gain = (y > 0 ? 1.0 : -1.0) * tau; }
            wing_.strips.push_back(s);
        }
    }

    void buildTail(const AircraftFile& a, const AircraftDerived& d, const Vec3d& cg) {
        if (a.has("tail.type") && a.str("tail.type") == "v") { buildVTail(a, d, cg); return; }
        const double c = a.num("tail.stab_chord_m"), span = a.num("tail.stab_span_m");
        const double ac_x = a.num("tail.stab_le_x_m") - 0.25 * c;
        stab_.name = "stabilizer"; stab_.table = tail_table_;
        stab_.ar = span * span / (span * c); stab_.oswald = 0.9;
        const int ns = int(a.num("aero_model.stab_strips"));
        const double tau_e = flapTau(a.num("controls.elevator_chord_frac")) * derate_;
        for (int j = 0; j < ns; ++j) {
            Strip s;
            const double y = -span / 2 + (j + 0.5) * span / ns;
            s.r = Vec3d{ac_x, y, a.num("tail.stab_z_m")} - cg;
            s.fwd = Vec3d{1, 0, 0}; s.nrm = Vec3d{0, 0, 1};
            s.chord = c; s.width = span / ns; s.ctrl = Ctrl::Elevator; s.ctrl_gain = tau_e;
            stab_.strips.push_back(s);
        }
        const double h = a.num("tail.fin_height_m"), cf = a.num("tail.fin_chord_m");
        const int nf = int(a.num("aero_model.fin_strips_per_fin")), fins = int(a.num("tail.fin_count"));
        fin_.name = "fins"; fin_.table = tail_table_;
        fin_.ar = h * h / (h * cf) * a.num("aero_model.fin_ar_effective_factor"); fin_.oswald = 0.7;
        const double tau_r = flapTau(a.num("controls.rudder_chord_frac")) * derate_;
        for (int k = 0; k < fins; ++k) {
            const double y = (k == 0 ? 1.0 : -1.0) * a.num("geometry.boom_spacing_m") / 2;
            for (int j = 0; j < nf; ++j) {
                Strip s;
                s.r = Vec3d{ac_x, y, a.num("aero_model.fin_base_z_m") + (j + 0.5) * h / nf} - cg;
                s.fwd = Vec3d{1, 0, 0}; s.nrm = Vec3d{0, 1, 0};
                s.chord = cf; s.width = h / nf; s.ctrl = Ctrl::Rudder; s.ctrl_gain = -tau_r;   // кермо вліво (+N) — сила вправо
                fin_.strips.push_back(s);
            }
        }
    }

    // V-хвіст: дві панелі під кутом Γ до горизонту (у «стабілізаторі»: скіс потоку від крила й слід гвинтів діють на них так само).
    // Нормаль лівої панелі (y > 0) — (0, −sinΓ, cosΓ), правої — (0, +sinΓ, cosΓ). Рулі (ruddervator): висота — обидві панелі однаково
    // (як руль висоти), напрям — різницево: лівій +τr, правій −τr дає бічну силу −Y, як кермо напряму з rudder > 0 (вертикальні складові взаємно гасяться).
    void buildVTail(const AircraftFile& a, const AircraftDerived&, const Vec3d& cg) {
        const double c = a.num("tail.vtail_chord_m"), len = a.num("tail.vtail_panel_m");
        const double gam = a.num("tail.vtail_dihedral_deg") * 3.14159265358979323846 / 180;
        const double ac_x = a.num("tail.vtail_le_x_m") - 0.25 * c, z0 = a.num("tail.vtail_root_z_m");
        stab_.name = "vtail"; stab_.table = tail_table_;
        stab_.ar = (2 * len) * (2 * len) / (2 * len * c); stab_.oswald = 0.85;   // пара панелей — як одна поверхня розмахом 2·len
        fin_.name = "fins"; fin_.table = tail_table_; fin_.ar = 1;   // окремих кілів немає
        const int n = int(a.num("aero_model.vtail_strips_per_panel"));
        const double tau_e = flapTau(a.num("controls.elevator_chord_frac")) * derate_, tau_r = flapTau(a.num("controls.rudder_chord_frac")) * derate_;
        for (int side : {1, -1}) {
            for (int j = 0; j < n; ++j) {
                Strip s;
                const double r = (j + 0.5) * len / n;
                s.r = Vec3d{ac_x, side * r * std::cos(gam), z0 + r * std::sin(gam)} - cg;
                s.fwd = Vec3d{1, 0, 0}; s.nrm = Vec3d{0, -side * std::sin(gam), std::cos(gam)};
                s.chord = c; s.width = len / n;
                s.ctrl = Ctrl::VTail; s.ctrl_gain = 1; s.ge = tau_e; s.gr = side * tau_r;
                stab_.strips.push_back(s);
            }
        }
    }

    // Заднє крило: прямокутне, профіль як в основного; руль висоти по всьому розмаху (ефективність τ(cf/c)·derate).
    // Індукований кут — як у короткої поверхні (ітерація cl/(π e AR)); скіс потоку від основного крила — Біо–Савар у eval().
    void buildRearWing(const AircraftFile& a, const Vec3d& cg) {
        rw_.name = "rear_wing"; rw_.table = wing_table_;
        if (!a.has("rear_wing.span_m")) return;
        if (a.has("rear_wing.symmetric") && a.num("rear_wing.symmetric") > 0.5) { rw_.table = tail_table_; rw_sym_ = true; }   // стабілізатор із симетричним профілем
        const double b = a.num("rear_wing.span_m"), c = a.num("rear_wing.chord_m");
        const double inc = a.num("rear_wing.incidence_deg") * 3.14159265358979323846 / 180, ac_x = a.num("rear_wing.le_x_m") - 0.25 * c;
        const int n = 2 * int(a.num("rear_wing.strips_per_half"));
        rw_.ar = b / c; rw_.oswald = 0.9;
        const double tau = flapTau(a.num("rear_wing.elevator_chord_frac")) * derate_;
        for (int j = 0; j < n; ++j) {
            Strip s;
            s.r = Vec3d{ac_x, -b / 2 + (j + 0.5) * b / n, a.num("rear_wing.z_m")} - cg;
            s.fwd = Vec3d{std::cos(inc), 0, std::sin(inc)};
            s.nrm = Vec3d{-std::sin(inc), 0, std::cos(inc)};
            s.chord = c; s.width = b / n; s.ctrl = Ctrl::Elevator; s.ctrl_gain = tau;
            rw_.strips.push_back(s);
        }
    }

    // Нелінійна підйомна лінія (Прандтль): для кожного α крила — розподіл індукованого кута ε_i (рад), як у Селіга
    void buildLiftingLine() {
        const double kPi = 3.14159265358979323846;
        const size_t N = wing_.strips.size();
        if (N == 0) return;
        const double dy = wing_.strips[0].width;
        for (double deg = -25; deg <= 30.0001; deg += 1.0) wing_.eps_alpha.push_back(deg * kPi / 180);
        wing_.eps.assign(N, std::vector<double>(wing_.eps_alpha.size(), 0.0));
        ll_resid_.assign(wing_.eps_alpha.size(), 0.0);
        struct LlSaved { std::vector<std::vector<double>> eps; std::vector<double> resid; };
        static std::mutex ll_mx;
        static std::map<std::string, LlSaved> ll_cache;
        std::string key;
        {
            std::vector<double> kv{double(N), dy};
            for (size_t i = 0; i < N; ++i) { kv.push_back(wing_.strips[i].chord); kv.push_back(wing_.strips[i].twist); kv.push_back(wing_.strips[i].r.y); }
            for (double deg = -30; deg <= 40; deg += 2.5) kv.push_back(wing_table_->eval(deg * kPi / 180).cl);
            key.assign(reinterpret_cast<const char*>(kv.data()), kv.size() * sizeof(double));
            std::lock_guard<std::mutex> l(ll_mx);
            const auto it = ll_cache.find(key);
            if (it != ll_cache.end()) { wing_.eps = it->second.eps; ll_resid_ = it->second.resid; return; }
        }
        std::vector<double> yk(N + 1);
        for (size_t k = 0; k <= N; ++k) yk[k] = wing_.strips.front().r.y - dy / 2 + double(k) * dy;
        std::vector<double> gam(N, 0.0), eps(N, 0.0);
        for (size_t ia = 0; ia < wing_.eps_alpha.size(); ++ia) {
            const double ag = wing_.eps_alpha[ia];
            std::fill(eps.begin(), eps.end(), 0.0);
            for (size_t i = 0; i < N; ++i) gam[i] = 0.5 * wing_.strips[i].chord * wing_table_->eval(ag + wing_.strips[i].twist).cl;
            // крок релаксації — з умови стійкості: найшвидша («пилкоподібна») мода росте як c/Δy, тож широка хорда на дрібних смугах
            // (c/Δy ≈ 8 у QP-5) з фіксованим кроком 0.2 розходилась у пилку ±7° і давала хибний індукований опір
            double c_max = 0;
            for (size_t i = 0; i < N; ++i) c_max = std::fmax(c_max, wing_.strips[i].chord);
            const double relax = std::fmin(0.2, 0.8 / (1.0 + 2.2 * c_max / dy));
            double diff = 0;
            for (int it = 0; it < 4000; ++it) {
                diff = 0;
                for (size_t i = 0; i < N; ++i) {
                    double w = 0;
                    for (size_t k = 0; k <= N; ++k) {
                        const double dG = (k < N ? gam[k] : 0.0) - (k > 0 ? gam[k - 1] : 0.0);
                        w += dG / (wing_.strips[i].r.y - yk[k]);
                    }
                    eps[i] = w / (4 * kPi);
                }
                for (size_t i = 0; i < N; ++i) {
                    const double target = 0.5 * wing_.strips[i].chord * wing_table_->eval(ag + wing_.strips[i].twist - eps[i]).cl;
                    diff = std::fmax(diff, std::fabs(target - gam[i]));
                    gam[i] += relax * (target - gam[i]);
                }
                if (diff < 1e-7) break;
            }
            for (size_t i = 0; i < N; ++i) wing_.eps[i][ia] = eps[i];
            ll_resid_[ia] = diff;
        }
        std::lock_guard<std::mutex> l(ll_mx);
        if (ll_cache.size() > 64) ll_cache.clear();
        ll_cache[key] = LlSaved{wing_.eps, ll_resid_};
    }

    double epsLookup(size_t strip, double alpha) const {   // індукований кут крила; за межами таблиці — спадає до 0 на 90° (Селіг)
        const std::vector<double>& g = wing_.eps_alpha;
        const double kHalfPi = 1.57079632679489661923;
        const std::vector<double>& e = wing_.eps[strip];
        if (alpha <= g.front()) return e.front() * std::fmax(0.0, 1 - (g.front() - alpha) / (kHalfPi + g.front()));
        if (alpha >= g.back()) return e.back() * std::fmax(0.0, 1 - (alpha - g.back()) / (kHalfPi - g.back()));
        const size_t hi = size_t(std::upper_bound(g.begin(), g.end(), alpha) - g.begin());
        const double f = (alpha - g[hi - 1]) / (g[hi] - g[hi - 1]);
        return e[hi - 1] + (e[hi] - e[hi - 1]) * f;
    }

    static Vec3d stripVelocity(const Strip& s, const AeroInputs& in, const Vec3d& air_vel) {
        return in.v_body + in.omega.cross(s.r) - air_vel;   // швидкість точки відносно повітря, що має власну швидкість air_vel
    }

    static double ctrlAngle(const Strip& s, const AeroInputs& in) {
        switch (s.ctrl) {
            case Ctrl::Aileron: return in.aileron;
            case Ctrl::Elevator: return in.elevator;
            case Ctrl::Rudder: return in.rudder;
            case Ctrl::VTail: return s.ge * in.elevator + s.gr * in.rudder;
            default: return 0;
        }
    }

    // ефект землі крила: множник індукованого кута за висотою смуги (Візельсбергер)
    double groundFactor(const Strip& s, const AeroInputs& in) const {
        if (in.h_agl > 50) return 1.0;
        const double h = std::fmax(in.h_agl + s.r.dot(in.up_body), 0.02), k = 16 * h / span_;
        return k * k / (1 + k * k);
    }
    StripOut stripForce(const Strip& s, const Vec3d& vs, double cmd, const Surface& surf, bool use_ll, double rho, size_t index, double ge = 1.0) const {
        StripOut o;
        const double vf = vs.dot(s.fwd), vn = vs.dot(s.nrm);
        const double vr2 = vf * vf + vn * vn;
        if (vr2 < 1e-4) return o;
        const double vr = std::sqrt(vr2);
        const double a_dir = std::atan2(-vn, vf);
        const double dalpha = s.ctrl_gain * cmd;
        const double a_c = a_dir + dalpha;
        double eps = 0;
        if (use_ll) {
            eps = epsLookup(index, a_c - s.twist) * ge;
        } else {   // індукований кут короткої поверхні: ε = cl(α − ε)/(π e AR); крок Ньютона з локальним нахилом (стійко й при малому AR кілів)
            const double k = 1.0 / (3.14159265358979323846 * surf.oswald * surf.ar), h = 0.002;
            for (int it = 0; it < 5; ++it) {
                const SectionCoeffs c0 = surf.table->eval(a_c - eps);
                const double slope = (surf.table->eval(a_c - eps + h).cl - surf.table->eval(a_c - eps - h).cl) / (2 * h);
                const double g = k * std::fmax(slope, 0.0) * (1 - c0.w_plate);
                const double resid = eps - k * c0.cl * (1 - c0.w_plate);
                eps = std::clamp(eps - resid / (1 + g), -0.6, 0.6);
            }
        }
        SectionCoeffs sc = surf.table->eval(a_c - eps);
        if (rec_) { const size_t si = size_t(surfIndex(surf)); if (index < rec_alpha_[si].size()) { rec_alpha_[si][index] = a_c - eps; rec_vr_[si][index] = vr; } }
        if (dyn_stall_) {   // динамічний зрив: замість статичної частки відриву — запізнена (стан смуги)
            const std::vector<DynState>& dv = dyn_[size_t(surfIndex(surf))];
            if (index < dv.size() && dv[index].init) {
                const double wd = dv[index].w, ws = sc.w_plate, ae = a_c - eps;
                if (std::fabs(wd - ws) > 1e-4) {
                    // приєднане обтікання — лінійна cl (обмежена 1.6·cl_max), відрив — пластина (cn = 2 sinα, cd_max sin²α)
                    const double cl_att = std::clamp(surf.table->cl_alpha * (ae - surf.table->alpha_zero), 1.6 * surf.table->cl_min, 1.6 * surf.table->cl_max);
                    const double cl_pl = 2 * std::sin(ae) * std::cos(ae), cd_pl = surf.table->cd_max * std::sin(ae) * std::sin(ae);
                    const double cd_att = surf.table->eval(std::clamp(ae, surf.table->alpha_stall_neg, surf.table->alpha_stall_pos)).cd;
                    sc.cl += (ws - wd) * (cl_att - cl_pl);
                    sc.cd = std::fmax(sc.cd + (ws - wd) * (cd_att - cd_pl), 0.005);
                    sc.w_plate = wd;
                }
            }
        }
        const double att = 1 - sc.w_plate;   // частка «приєднаного» обтікання
        const double dcl = surf.table->cl_alpha * dalpha * att;
        const double cl = sc.cl;             // Δα керма вже враховано зсувом кута a_c
        double re_k = 1.0;   // профільний опір ∝ Re^−0.2 відносно Re таблиці (менша швидкість — товстіший прикордонний шар)
        if (surf.re_ref > 0) re_k = std::pow(std::clamp(vr * s.chord / 1.5e-5, 2e4, 2e6) / surf.re_ref, -0.2);
        const double cd = sc.cd * (att * re_k + (1 - att)) + std::fmax(0.0, sc.cl * eps) * att + flap_drag_ * dalpha * dalpha * att;   // + індукований опір cl·ε
        const double cm = sc.cm - cm_per_cl_ * dcl;
        const double q = 0.5 * rho * vr2, area = s.chord * s.width;
        const Vec3d d_hat = s.fwd * (-vf / vr) + s.nrm * (-vn / vr);
        const Vec3d l_hat = s.fwd * std::sin(a_dir) + s.nrm * std::cos(a_dir);
        o.force = (l_hat * cl + d_hat * cd) * (q * area);
        o.moment = s.fwd.cross(s.nrm) * (cm * q * area * s.chord);   // ніс угору +: вісь fwd × nrm
        o.cl = cl;
        o.gamma = 0.5 * vr * s.chord * cl;
        return o;
    }

    // Швидкість, індукована в точці P підковоподібними вихорами крила (Біо–Савар, Γ_j — із смуг)
    Vec3d wingInduced(const Vec3d& P, const std::vector<double>& gamma, const Vec3d& wake_dir) const {
        const double kFar = 200.0, kPi4 = 4 * 3.14159265358979323846, core = wake_core_;
        auto seg = [&](const Vec3d& A, const Vec3d& B, double g) {
            const Vec3d r1 = P - A, r2 = P - B, r0 = B - A, cr = r1.cross(r2);
            const double n1 = r1.norm(), n2 = r2.norm(), d = cr.dot(cr) + core * core * r0.dot(r0);
            if (n1 < 1e-9 || n2 < 1e-9 || d < 1e-18) return Vec3d{};
            return cr * (g / kPi4 * r0.dot(r1 / n1 - r2 / n2) / d);
        };
        Vec3d v;
        for (size_t j = 0; j < gamma.size(); ++j) {
            const Strip& s = wing_.strips[j];
            const Vec3d A{s.r.x, s.r.y + s.width / 2, s.r.z}, B{s.r.x, s.r.y - s.width / 2, s.r.z};
            v += seg(A + wake_dir * kFar, A, gamma[j]) + seg(A, B, gamma[j]) + seg(B, B + wake_dir * kFar, gamma[j]);
        }
        return v;
    }
};

}  // namespace sim
