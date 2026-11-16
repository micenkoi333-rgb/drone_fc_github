// qp_sim.h — повна модель апарата: 6DOF (RK4 1 кГц), аеродинаміка (qp_aero.h), 4 гвинти на поворотних гондолах (конвертоплан;
// без розділу [tilt] — нерухомі, вісь +Z), необов'язковий маршовий гвинт ([cruise_motor]), мотори/ESC, батарея Тевенена, ISA,
// вітер Драйдена, шасі TPU й жорсткі точки землі. Тіло FLU, початок O — ЦМ (за кресленням).
// Керування — на рівні виконавчих органів: duty моторів, кути нахилу гондол, кути поверхонь (рад, знаки в qp_aero.h).
// Назви класів (QuadPlaneSim, Qp*) лишилися від попереднього апарата QuadPlane — модель та сама, змінилася схема гвинтів.
#pragma once
#include <array>
#include <memory>
#include <random>
#include "aero_tables.h"
#include "aircraft_params.h"
#include "qp_aero.h"
#include "qp_battery.h"
#include "qp_env.h"
#include "qp_prop.h"
#include "qp_servo.h"
#include "vec3d.h"
#include "portable_rng.h"

namespace sim {

struct QpControls {
    double duty_vtol[4] = {0, 0, 0, 0};   // 0 передній-правий, 1 задній-лівий, 2 передній-лівий, 3 задній-правий (як fc::kQuadX)
    double duty_cruise = 0;
    double aileron = 0, elevator = 0, rudder = 0;   // рад
    double tilt[4] = {0, 0, 0, 0};        // рад, нахил гондол: 0 — гвинт тягне вгору, π/2 — уперед (за тим самим порядком моторів)
};

struct QpState {
    Vec3d pos;        // ЦМ у світі (ENU, Z вгору, 0 — земля), м
    Vec3d vel_b;      // швидкість ЦМ у тілі, м/с
    Vec3d omega;      // рад/с
    Quatd q;          // тіло -> світ
};

// Відмови, що вводяться в модель (для випробувань): множник duty моторів (0 — мертвий), заклинені рулі, слабка батарея
struct QpFaults {
    double vtol_scale[4] = {1, 1, 1, 1};     // підйомні мотори (0.6 — частково, 0 — вимкнено)
    double cruise_scale = 1;                 // маршовий мотор
    bool servo_stuck[3] = {false, false, false};   // елерон, висота, напрям: тримаються на місці
    bool tilt_stuck[4] = {false, false, false, false};   // заклинений привід нахилу гондоли
    double tilt_strength[4] = {1, 1, 1, 1};   // момент приводу нахилу × (знос, просадка живлення сервоприводу)
    double servo_strength[3] = {1, 1, 1};     // момент рульових машинок ×
    double prop_scale[4] = {1, 1, 1, 1};      // пошкоджений гвинт (відламаний кінчик лопаті): тяга й момент опору ×
    double battery_r_scale = 1, battery_cap_scale = 1;   // внутрішній опір ×, ємність ×
};

class QuadPlaneSim {
public:
    QpFaults faults;
    static constexpr double kG = 9.80665;

    QuadPlaneSim(const AircraftFile& file, const AeroTables& tables, unsigned seed = 1)
        : file_(file), derived_(deriveAircraft(file)), aero_(file, tables, derived_), vtol_prop_(file, "vtol_prop"),
          rear_prop_(file, file.has("vtol_prop_rear.diameter_m") ? "vtol_prop_rear" : "vtol_prop"),
          cruise_prop_(file, file.has("cruise_prop.diameter_m") ? "cruise_prop" : "vtol_prop"), battery_(file), wind_(WindConfig{}, seed), rng_(seed * 2654435761u + 17u) {
        auto flag = [&](const char* k) { return file.has(k) && file.num(k) > 0.5; };
        rotor_ge_ = flag("aero_model.rotor_ground_effect"); rotor_vrs_ = flag("aero_model.rotor_vortex_ring"); rotor_wash_ = flag("aero_model.rotor_wash_on_wing");
        rotor_bem_ = flag("aero_model.rotor_bem") && file.has("vtol_prop.pitch_m");
        if (rotor_bem_) {   // калібрування лопаті під виміряну статику (CT, CP при J = 0)
            bem_ = BemRotor(file, "vtol_prop");
            BemRotor::Out o; double vi = 0;
            for (int k = 0; k < 12; ++k) { o = bem_.eval(100, 0, 0, 1.225, 1, vi, 4); vi = o.vi; }
            const double d = vtol_prop_.diameter(), d4 = d * d * d * d;
            if (o.thrust > 0.1) bem_kt_ = std::clamp(vtol_prop_.ct(0) * 1.225 * 1e4 * d4 / o.thrust, 0.5, 2.0);
            if (o.torque > 1e-4) bem_kq_ = std::clamp(vtol_prop_.cp(0) * 1.225 * 1e6 * d4 * d / (2 * 3.14159265358979323846 * 100) / o.torque, 0.5, 2.0);
        }
        tilt_gyro_ = flag("aero_model.tilt_gyroscopic");
        rotor_rotor_ = flag("aero_model.rotor_rotor_interference");
        if (file.has("structure.roll_mode_hz")) {
            const double pi = 3.14159265358979323846;
            flex_w_[0] = 2 * pi * file.num("structure.roll_mode_hz"); flex_w_[1] = 2 * pi * file.num("structure.pitch_mode_hz");
            flex_z_ = file.num("structure.mode_damping");
            flex_k_[0] = file.num("structure.roll_deg_per_nm") * pi / 180; flex_k_[1] = file.num("structure.pitch_deg_per_nm") * pi / 180;
        }
        if (file.has("tilt.backlash_deg")) tilt_backlash_ = file.num("tilt.backlash_deg") * 3.14159265358979323846 / 180;
        if (file.has("controls.servo_backlash_deg")) servo_backlash_ = file.num("controls.servo_backlash_deg") * 3.14159265358979323846 / 180;
        origin_ = Vec3d{derived_.cg_x, derived_.cg_y, derived_.cg_z};
        // осі повороту гондол: front_* / rear_* (конвертоплан) або x_m, y_m, z_m (прямокутник, як у QuadPlane)
        const bool split = file.has("vtol_motor.front_x_m");
        static const double kSx[4] = {+1, -1, +1, -1}, kSy[4] = {-1, +1, +1, -1}, kYaw[4] = {-1, -1, +1, +1};
        tilt_on_ = file.has("tilt.rate_deg_s");
        if (tilt_on_) {
            const double d2r = 3.14159265358979323846 / 180;
            rear_pusher_ = file.num("tilt.rear_pusher") > 0.5; push_k_ = file.num("tilt.pusher_thrust_factor"); prop_off_ = file.num("tilt.prop_offset_m");
            tilt_min_ = file.num("tilt.min_deg") * d2r; tilt_max_ = file.num("tilt.max_deg") * d2r;
            tilt_rate_ = file.num("tilt.rate_deg_s") * d2r; tilt_tau_ = file.num("tilt.tau_s");
            cruise_rear_ = !file.has("tilt.cruise_rear") || std::fabs(file.num("tilt.cruise_rear") - 1) < 0.5;   // 1 — у крейсері тягнуть усі 4 (0 і 2 — передні)
        }
        for (int i = 0; i < 4; ++i) {
            const bool front = kSx[i] > 0;
            const char* pre = front ? "vtol_motor.front_" : "vtol_motor.rear_";
            const Vec3d pv = split ? Vec3d{file.num(std::string(pre) + "x_m"), kSy[i] * file.num(std::string(pre) + "y_m"), file.num(std::string(pre) + "z_m")}
                                   : Vec3d{kSx[i] * file.num("vtol_motor.x_m"), kSy[i] * file.num("vtol_motor.y_m"), file.num("vtol_motor.z_m")};
            pivot_[i] = pv - origin_;
            // площина гвинта — на prop_offset уздовж осі тяги: тягнучий — «над/перед» віссю, штовхальний (задні з rear_pusher) — «під/за»
            disk_sgn_[i] = (!tilt_on_) ? 0.0 : (front || !rear_pusher_ ? 1.0 : -1.0);
            thrust_k_[i] = (tilt_on_ && !front && rear_pusher_) ? push_k_ : 1.0;
            yaw_sign_[i] = kYaw[i];
            vtol_motor_[i] = MotorEsc(file, (!front && file.has("vtol_motor_rear.kv")) ? "vtol_motor_rear" : "vtol_motor", vp(i).inertia());
        }
        has_cruise_ = file.has("cruise_motor.kv");
        if (has_cruise_) {
            cruise_pos_ = Vec3d{file.num("cruise_motor.x_m"), 0, 0} - origin_;
            cruise_motor_ = MotorEsc(file, "cruise_motor", cruise_prop_.inertia());
            reaction_sign_ = file.num("cruise_motor.reaction_sign");
            slip_gain_ = file.num("cruise_motor.slipstream_gain");
        } else {
            cruise_motor_ = MotorEsc(file, "vtol_motor", vtol_prop_.inertia());   // немає: duty завжди 0, у силах не бере участі
        }
        tau_servo_ = file.num("servo.tau_s"); rate_servo_ = file.num("servo.rate_limit_rad_s");
        max_defl_ = file.num("controls.max_deflection_deg") * 3.14159265358979323846 / 180;
        {
            const double d2r = 3.14159265358979323846 / 180;
            auto opt = [&](const char* k, double def) { return file.has(k) ? file.num(k) : def; };
            ServoSpec ss;
            ss.rate = rate_servo_; ss.tau = tau_servo_; ss.backlash = servo_backlash_;
            ss.stall_nm = opt("servo.stall_nm", 0.45); ss.deadband = opt("servo.deadband_deg", 0.1) * d2r;
            ss.compliance = opt("servo.compliance_deg_per_nm", 3.0) * d2r;
            for (ServoActuator& v : surf_srv_) v = ServoActuator(ss);
            ServoSpec ts;
            ts.rate = tilt_rate_; ts.tau = tilt_tau_; ts.backlash = tilt_backlash_;
            ts.stall_nm = opt("tilt.servo_stall_nm", 2.9); ts.deadband = opt("tilt.servo_deadband_deg", 0.15) * d2r;
            ts.compliance = opt("tilt.servo_compliance_deg_per_nm", 0.6) * d2r;
            ts.inertia = opt("tilt.nacelle_inertia_kgm2", 4e-4);
            for (ServoActuator& v : tilt_srv_) v = ServoActuator(ts);
            for (int i = 0; i < 4; ++i) tilt_srv_[size_t(i)].reset(tilt_[size_t(i)]);
            const double ch = opt("servo.hinge_ch_delta", 0.55);   // |∂Ch/∂δ| тонкого профілю з рулем 25–35% хорди
            const double span = file.num("geometry.wing_span_m"), chord = file.num("geometry.wing_chord_m");
            const double ca = chord * opt("controls.aileron_chord_frac", 0.25);
            const double sa = span / 2 * (opt("controls.aileron_outer_frac", 0.95) - opt("controls.aileron_inner_frac", 0.45)) * ca;
            hinge_k_[0] = ch * sa * ca;
            const double cv = opt("tail.vtail_chord_m", 0.11) * opt("controls.elevator_chord_frac", 0.35);
            hinge_k_[1] = hinge_k_[2] = ch * opt("tail.vtail_panel_m", 0.25) * cv * cv;
        }
        avionics_w_ = file.num("avionics.power_w");
        buildContacts();
        for (const MassItem& it : file.items) fixed_items_.push_back(it);
        updateMass();
    }

    // --- настройка
    void setWind(const WindConfig& w, unsigned seed = 1) { wind_ = Wind(w, seed); }
    void setAtmosphere(const Atmosphere& a) { atmo_ = a; }
    void place(const Vec3d& pos, const Vec3d& vel_world, double roll, double pitch, double yaw) {
        st_.pos = pos; st_.q = Quatd::fromEuler(roll, pitch, yaw); st_.vel_b = st_.q.rotateInv(vel_world); st_.omega = Vec3d{};
    }
    void setState(const QpState& s) { st_ = s; }
    void setRotorRps(double vtol, double cruise) {
        for (auto& m : vtol_motor_) m.setOmega(vtol * 2 * 3.14159265358979323846);
        cruise_motor_.setOmega(cruise * 2 * 3.14159265358979323846);
    }
    PackBattery& battery() { return battery_; }
    const PackBattery& battery() const { return battery_; }

    Vec3d windAt(double h) const { return wind_.mean(std::fmax(h, 0.0)) + (wind_now_ - wind_.mean(std::fmax(st_.pos.z, 0.0))); }

    // --- один крок dt (с); рекомендовано 0.001
    void step(double dt, const QpControls& u) {
        const double h = std::fmax(st_.pos.z, 0.0);
        rho_ = atmo_.density(h);
        Vec3d wind_world = wind_.step(dt, h, std::fmax(airspeedVec(Vec3d{}).norm(), 5.0));
        wind_world.z += wind_.thermalW(st_.pos.x, st_.pos.y, h, t_);   // термічні потоки
        wind_now_ = wind_world;
        gust_rates_ = wind_.gustRates();
        QpControls ue = u;   // команди з урахуванням відмов
        for (int i = 0; i < 4; ++i) ue.duty_vtol[i] *= faults.vtol_scale[i];
        ue.duty_cruise = has_cruise_ ? ue.duty_cruise * faults.cruise_scale : 0.0;
        battery_.setResistanceScale(faults.battery_r_scale); battery_.setCapacityScale(faults.battery_cap_scale);
        ctrl_ = ue;
        updateServos(dt, ue);
        // RK4 жорсткого тіла при заморожених обертах, поверхнях і вітрі
        const QpState s0 = st_;
        aero_cache_ok_ = false; in_rk_ = aero_freeze_;
        if (aero_freeze_) aero_.recordBegin();
        Deriv k1 = deriv(s0), k2 = deriv(add(s0, k1, dt / 2)), k3 = deriv(add(s0, k2, dt / 2)), k4 = deriv(add(s0, k3, dt));
        Deriv k = combine(k1, k2, k3, k4);
        aero_cache_ok_ = false; in_rk_ = false;
        st_ = add(s0, k, dt);
        st_.q.normalize();
        if (!st_.pos.finite() || !st_.vel_b.finite() || !st_.omega.finite()) { diverged_ = true; st_ = s0; }
        t_ += dt;
        stepPropulsion(dt, ue);
        if (aero_freeze_) aero_.advanceRecorded(dt); else aero_.advance(dt, aeroInputs(st_));   // динамічний зрив: стан відриву смуг
        detectCrash();
    }

    // --- спостереження
    const QpState& state() const { return st_; }
    double time() const { return t_; }
    // Вантаж у відсіку біля ЦМ (трохи нижче): маса, центр мас і інерція перераховуються; 0 — без вантажу
    void setPayload(double kg) {
        fixed_items_.erase(std::remove_if(fixed_items_.begin(), fixed_items_.end(), [](const MassItem& it) { return it.name == "payload_runtime"; }), fixed_items_.end());
        if (kg > 1e-6) {
            const MassProps mp = massProperties(fixed_items_);
            MassItem p; p.name = "payload_runtime"; p.m = kg; p.x = mp.cg_x; p.y = mp.cg_y; p.z = mp.cg_z - 0.05; p.dx = 0.12; p.dy = 0.10; p.dz = 0.06;
            fixed_items_.push_back(p);
        }
        payload_kg_ = kg;
        updateMass();
    }
    double payload() const { return payload_kg_; }
    double payload_kg_ = 0;

    double mass() const { return mass_; }
    Vec3d cgOffset() const { return cg_rel_; }
    const Inertia& inertia() const { return inertia_; }
    const AircraftDerived& derived() const { return derived_; }
    const QuadPlaneAero& aero() const { return aero_; }
    Vec3d windWorld() const { return wind_now_; }
    double density() const { return rho_; }
    bool crashed() const { return crashed_; }
    bool diverged() const { return diverged_; }
    Vec3d velWorld() const { return st_.q.rotate(st_.vel_b); }
    Vec3d eulerRad() const { return st_.q.euler(); }
    double vtolRps(int i) const { return vtol_motor_[size_t(i)].rps(); }
    double cruiseRps() const { return cruise_motor_.rps(); }
    double vtolThrust(int i) const { return vtol_thrust_[size_t(i)]; }
    Vec3d flexRate() const { return flex_v_; }   // кутова швидкість пружних коливань у місці IMU (рад/с)
    void setRotorInterference(bool on) { rotor_rotor_ = on; }
    void setFlexScale(double k) { flex_s_ = k; }
    double flexScale() const { return flex_s_; }
    void setFlexModes(double roll_hz, double pitch_hz) {   // інша рама: податливість ∝ 1/ω² (та сама модальна маса — м'якша рама має нижчу частоту)
        const double w[2] = {2 * 3.14159265358979323846 * roll_hz, 2 * 3.14159265358979323846 * pitch_hz};
        for (int k = 0; k < 2; ++k) { if (flex_w_[k] > 0) flex_k_[k] *= (flex_w_[k] / w[k]) * (flex_w_[k] / w[k]); flex_w_[k] = w[k]; }
    }
    double interferenceInflow(int i) const { return ifr_in_[size_t(i)]; }   // м/с: слід інших гвинтів крізь диск уздовж осі (> 0 — дме проти тяги)
    void setFlex(bool on) { if (!on) { flex_w_[0] = flex_w_[1] = 0; flex_v_ = flex_q_ = Vec3d{}; } }
    double tilt(int i) const { return tilt_[size_t(i)]; }                 // фактичний нахил гондоли, рад
    bool tiltEnabled() const { return tilt_on_; }
    bool rearPusher() const { return rear_pusher_; }
    bool hasCruiseMotor() const { return has_cruise_; }
    bool cruiseRear() const { return cruise_rear_; }                      // у польоті літаком тягнуть і задні гвинти
    double thrustFactor(int i) const { return thrust_k_[size_t(i)]; }    // штовхальний гвинт у затіненні гондоли < 1
    Vec3d rotorAxis(int i) const { const double t = tilt_[size_t(i)]; return Vec3d{std::sin(t), 0, std::cos(t)}; }   // напрям тяги в тілі
    const Vec3d& pivot(int i) const { return pivot_[size_t(i)]; }
    void presetTilt(double t) { for (int i = 0; i < 4; ++i) { tilt_[size_t(i)] = tilt_gear_[size_t(i)] = t; tilt_srv_[size_t(i)].reset(t); } }   // для тримування й лінеаризації
    void presetTilts(const double t[4]) { for (int i = 0; i < 4; ++i) { tilt_[size_t(i)] = t[i]; tilt_gear_[size_t(i)] = t[i]; tilt_srv_[size_t(i)].reset(t[i]); } }
    double cruiseThrust() const { return cruise_thrust_; }
    double busCurrent() const { return bus_current_; }
    double busPower() const { return bus_current_ * bus_voltage_; }
    double busVoltage() const { return bus_voltage_; }
    double servoAileron() const { return servo_[0]; }
    // перемикачі уточнень моделі (за замовчуванням — з quadplane.ini, [aero_model])
    void setRotorEffects(bool ground, bool vrs, bool wash) { rotor_ge_ = ground; rotor_vrs_ = vrs; rotor_wash_ = wash; if (!wash) rotor_rotor_ = false; }
    QuadPlaneAero& aeroModel() { return aero_; }
    double vtolFactor(int i) const { return vtol_factor_[size_t(i)]; }   // множник тяги гвинта (земля × вихрове кільце)
    double servoElevator() const { return servo_[1]; }
    double servoRudder() const { return servo_[2]; }
    const ServoActuator& tiltServo(int i) const { return tilt_srv_[size_t(i)]; }   // привід нахилу: момент, нагрів
    ServoActuator& tiltServo(int i) { return tilt_srv_[size_t(i)]; }
    ServoActuator& surfaceServo(int i) { return surf_srv_[i]; }
    const ServoActuator& surfaceServo(int i) const { return surf_srv_[i]; }      // 0 елерон, 1 висота, 2 напрям
    double groundContactForce() const { return contact_fn_; }
    // повітряна швидкість у тілі (відносно повітря, у точці ЦМ)
    Vec3d airspeedVec(const Vec3d&) const { return st_.vel_b - st_.q.rotateInv(wind_now_); }
    double airspeed() const { return airspeedVec(Vec3d{}).norm(); }
    // аеродинамічний звіт у поточному стані (для діагностики/тримування)
    AeroResult aeroNow() const { return aero_.eval(aeroInputs(st_)); }

    // --- для тримування й лінеаризації: без кроку, у заданому стані й при замороженому приводі
    void forcesAndMoments(const QpState& s, Vec3d& F, Vec3d& M) const { evalForces(s, F, M); }
    void bodyAccel(const QpState& s, Vec3d& dv, Vec3d& dw) const { const Deriv d = deriv(s); dv = d.dvel; dw = d.domega; }
    void setDensity(double rho) { rho_ = rho; }
    // заморозити привід: положення поверхонь, оберти моторів, довідкова тяга й момент маршового гвинта (для сліду й реакції)
    void presetDrive(double ail, double ele, double rud, double cruise_rps, const double vtol_rps[4], double cruise_thrust, double cruise_torque) {
        servo_[0] = ail; servo_[1] = ele; servo_[2] = rud; servo_gear_[0] = ail; servo_gear_[1] = ele; servo_gear_[2] = rud;
        for (int i = 0; i < 3; ++i) surf_srv_[i].reset(servo_[i]);
        cruise_motor_.setOmega(cruise_rps * 2 * 3.14159265358979323846);
        for (int i = 0; i < 4; ++i) { vtol_motor_[size_t(i)].setOmega(vtol_rps[i] * 2 * 3.14159265358979323846); motor_torque_[size_t(i)] = 0; bem_c_[size_t(i)].valid = false; }
        cruise_thrust_ = cruise_thrust; cruise_torque_ = cruise_torque;
    }
    const Propeller& vtolProp() const { return vtol_prop_; }
    const Propeller& vtolProp(int i) const { return vp(i); }
    const MotorEsc& vtolMotor(int i) const { return vtol_motor_[size_t(i)]; }
    bool rearDiffers() const { return file_.has("vtol_prop_rear.diameter_m") || file_.has("vtol_motor_rear.kv"); }
    const Propeller& cruiseProp() const { return cruise_prop_; }
    const MotorEsc& vtolMotor() const { return vtol_motor_[0]; }
    const MotorEsc& cruiseMotor() const { return cruise_motor_; }
    Vec3d rotorPos(int i) const { return diskPos(i); }   // центр диска гвинта (відн. O, тіло)
    double avionicsPower() const { return avionics_w_; }
    Vec3d rotorAngularMomentum() const {   // H_r в тілі (для лінеаризації)
        Vec3d hr{0, 0, 0};
        for (int i = 0; i < 4; ++i) hr += rotorAxis(i) * (vp(i).inertia() * vtol_motor_[size_t(i)].omega() * (-yaw_sign_[size_t(i)]));
        if (has_cruise_) hr.x += cruise_prop_.inertia() * cruise_motor_.omega() * (-reaction_sign_);
        return hr;
    }

private:
    struct Deriv { Vec3d dpos, dvel, domega; double dq[4] = {0, 0, 0, 0}; };
    struct Contact { Vec3d r; double k, c; bool hard; };

    const AircraftFile& file_;
    AircraftDerived derived_;
    QuadPlaneAero aero_;
    Propeller vtol_prop_, rear_prop_, cruise_prop_;
    const Propeller& vp(int i) const { return (i == 1 || i == 3) ? rear_prop_ : vtol_prop_; }   // 0, 2 — передні; 1, 3 — задні
    PackBattery battery_;
    Wind wind_;
    Atmosphere atmo_;
    QpState st_;
    QpControls ctrl_;
    Vec3d origin_, cg_rel_, cruise_pos_, wind_now_, gust_rates_;
    std::array<Vec3d, 4> pivot_;                     // осі повороту гондол (відн. O)
    std::array<double, 4> tilt_{}, disk_sgn_{}, thrust_k_{1, 1, 1, 1};
    bool tilt_on_ = false, rear_pusher_ = false, has_cruise_ = false, cruise_rear_ = true;
    double push_k_ = 1, prop_off_ = 0, tilt_min_ = 0, tilt_max_ = 0, tilt_rate_ = 1.5, tilt_tau_ = 0.05;
    struct Wake { Vec3d c, d; double vi = 0; };
    int rotorWakes(const QpState& s, std::array<Wake, 4>& wk) const {
        int nwk = 0;
        const double rr = vtol_prop_.diameter() / 2;
        for (int i = 0; i < 4; ++i) {
            const double T = vtol_thrust_[size_t(i)];
            if (T < 0.3) continue;
            const Vec3d c = diskPos(i), ax = rotorAxis(i);
            const Vec3d va = pointAirVel(s, c);
            const double v_ax = va.dot(ax);
            const double vh = std::sqrt(T / (2 * rho_ * 3.14159265358979323846 * rr * rr)), mu = (va - ax * v_ax).norm();
            double vi = vh;
            for (int k = 0; k < 8; ++k) vi = std::clamp(vh * vh / std::sqrt(mu * mu + (v_ax + vi) * (v_ax + vi) + 1e-6), 0.0, 2.0 * vh);
            const Vec3d dir = (va * -1.0 - ax * vi);
            if (dir.norm() < 1e-3) { wk[size_t(nwk)].vi = 0; continue; }
            wk[size_t(nwk)] = Wake{c, dir.normalized(), vi};
            wake_src_[size_t(nwk++)] = i;
        }
        return nwk;
    }
    static Vec3d wakeVel(const std::array<Wake, 4>& wk, int nwk, double rr, const Vec3d& r, int skip, const std::array<int, 4>* src = nullptr) {
        Vec3d a{};
        for (int i = 0; i < nwk; ++i) {
            if (src && (*src)[size_t(i)] == skip) continue;
            const Wake& w = wk[size_t(i)];
            const Vec3d rel = r - w.c;
            const double along = rel.dot(w.d);
            if (along < -rr || along > 12 * rr) continue;
            const double perp = (rel - w.d * along).norm();
            const double rw = along > 0 ? rr * (0.78 + 0.22 * std::exp(-along / rr)) : rr;
            const double edge = std::clamp((rw - perp) / (0.15 * rr) + 0.5, 0.0, 1.0);
            if (edge <= 0) continue;
            a += w.d * (w.vi * (1 + along / std::sqrt(along * along + rr * rr)) * edge);
        }
        return a;
    }
    mutable std::array<int, 4> wake_src_{};
    std::array<double, 4> ifr_in_{};
    bool rotor_rotor_ = false;
    double flex_w_[2] = {0, 0}, flex_z_ = 0.03, flex_k_[2] = {0, 0}, flex_s_ = 1;
    Vec3d flex_q_{}, flex_v_{};
    Vec3d diskPos(int i) const { return pivot_[size_t(i)] + rotorAxis(i) * (prop_off_ * disk_sgn_[size_t(i)]); }
    std::array<double, 4> yaw_sign_{}, vtol_thrust_{}, motor_torque_{};
    std::array<double, 4> vrs_noise_{}, vtol_factor_{1, 1, 1, 1};
    bool rotor_ge_ = false, rotor_vrs_ = false, rotor_wash_ = false, rotor_bem_ = false, tilt_gyro_ = false;
    BemRotor bem_;
    struct BemCache { bool valid = false; double dT = 0, dQ = 0, fx = 0, fy = 0, mx = 0, my = 0, vi = 0, vi0 = 0; };
    double bem_kt_ = 1, bem_kq_ = 1;
    mutable std::array<BemCache, 4> bem_c_{};
    std::array<double, 4> tilt_rate_now_{}, tilt_gear_{};
    double tilt_backlash_ = 0, servo_backlash_ = 0;
    double servo_gear_[3] = {0, 0, 0};
    std::array<ServoActuator, 4> tilt_srv_{};
    ServoActuator surf_srv_[3];
    std::array<double, 4> tilt_load_{};
    double hinge_k_[3] = {0, 0, 0};   // шарнірний момент поверхні = q·hinge_k·δ (S·c·|Ch_δ|)
    long prop_step_ = 0;

    BemCache bemFactors(int i, double n, double v_ax, double v_ip, double t_tab, double q_tab, double vi_g, double vi0_g, int iters) const {
        BemCache c; c.valid = true;
        (void)t_tab; (void)q_tab;
        if (!rotor_bem_ || n < 5 || v_ip < 0.3) return c;
        const int spin = yaw_sign_[size_t(i)] > 0 ? -1 : 1;
        const BemRotor::Out o = bem_.eval(n, v_ax, v_ip, rho_, spin, vi_g, iters), o0 = bem_.eval(n, v_ax, 0, rho_, spin, vi0_g, iters);
        c.vi = o.vi; c.vi0 = o0.vi;
        c.dT = (o.thrust - o0.thrust) * bem_kt_;   // приріст від косого потоку (до виміряної осьової тяги з таблиць)
        c.dQ = (o.torque - o0.torque) * bem_kq_;
        c.fx = o.fx * bem_kt_; c.fy = o.fy * bem_kt_; c.mx = o.mx * bem_kt_; c.my = o.my * bem_kt_;
        return c;
    }
    std::mt19937 rng_;
    prng::normal<double> nd_{0.0, 1.0};
    std::array<MotorEsc, 4> vtol_motor_;
    MotorEsc cruise_motor_;
    double cruise_thrust_ = 0, cruise_torque_ = 0, reaction_sign_ = -1, slip_gain_ = 1.8;
    double servo_[3] = {0, 0, 0}, tau_servo_ = 0.05, rate_servo_ = 8, max_defl_ = 0.43;
    double mass_ = 3.5, rho_ = 1.225, t_ = 0, avionics_w_ = 12;
    double bus_current_ = 0, bus_voltage_ = 25, contact_fn_ = 0;
    bool crashed_ = false, diverged_ = false;
    std::vector<MassItem> fixed_items_;
    Inertia inertia_;
    Mat3d inertia_mat_, inertia_inv_;
    std::vector<Contact> contacts_;
    double mu_ = 0.6, slip_reg_ = 0.05, crash_vz_ = 3.5, gear_crash_vz_ = 4.5;

    void buildContacts() {
        if (file_.has("ground.gear_points")) {   // м'які опори з файлу (x y z низу ніжки)
            const std::vector<double> gp = file_.nums("ground.gear_points");
            if (gp.size() % 3 != 0) throw std::runtime_error("ground.gear_points: кількість чисел не кратна 3");
            for (size_t i = 0; i + 2 < gp.size(); i += 3)
                contacts_.push_back({Vec3d{gp[i], gp[i + 1], gp[i + 2]} - origin_, file_.num("ground.gear_stiffness_n_m"), file_.num("ground.gear_damping_ns_m"), false});
        } else {   // ніжки під моторами (QuadPlane)
            const double mx = file_.num("vtol_motor.x_m"), my = file_.num("vtol_motor.y_m"), tip = file_.num("vtol_motor.z_m") - file_.num("geometry.gear_height_m");
            for (double sx : {1.0, -1.0})
                for (double sy : {1.0, -1.0})
                    contacts_.push_back({Vec3d{sx * mx, sy * my, tip} - origin_, file_.num("ground.gear_stiffness_n_m"), file_.num("ground.gear_damping_ns_m"), false});
        }
        const std::vector<double> hp = file_.nums("ground.hard_points");
        if (hp.size() % 3 != 0) throw std::runtime_error("ground.hard_points: кількість чисел не кратна 3");
        for (size_t i = 0; i + 2 < hp.size(); i += 3)
            contacts_.push_back({Vec3d{hp[i], hp[i + 1], hp[i + 2]} - origin_, file_.num("ground.hard_stiffness_n_m"), file_.num("ground.hard_damping_ns_m"), true});
        mu_ = file_.num("ground.friction_mu"); slip_reg_ = file_.num("ground.slip_reg_mps");
        crash_vz_ = file_.num("ground.crash_vz_mps"); gear_crash_vz_ = file_.num("ground.gear_crash_vz_mps");
    }

    void updateMass() {
        const MassProps mp = massProperties(fixed_items_);
        aero_.setBodyCda(Vec3d{file_.num("drag.cda_x_m2"), file_.num("drag.cda_y_m2"), file_.num("drag.cda_z_m2")});
        mass_ = mp.mass;
        cg_rel_ = Vec3d{mp.cg_x, mp.cg_y, mp.cg_z} - origin_;
        inertia_ = mp.inertia;
        Mat3d m;
        m.m[0][0] = mp.inertia.xx; m.m[1][1] = mp.inertia.yy; m.m[2][2] = mp.inertia.zz;
        m.m[0][1] = m.m[1][0] = mp.inertia.xy; m.m[0][2] = m.m[2][0] = mp.inertia.xz; m.m[1][2] = m.m[2][1] = mp.inertia.yz;
        inertia_mat_ = m; inertia_inv_ = m.inverse();
    }

    void updateServos(double dt, const QpControls& u) {
        const double cmd[3] = {u.aileron, u.elevator, u.rudder};
        const double vx = st_.vel_b.x - st_.q.rotateInv(wind_now_).x;
        const double qd = 0.5 * rho_ * vx * std::fabs(vx);   // напір уздовж хорди (заднім ходом момент міняє знак)
        for (int i = 0; i < 3; ++i) {
            if (faults.servo_stuck[i]) { surf_srv_[i].hold(); continue; }   // заклинена машинка тримає положення
            const double target = std::clamp(cmd[i], -max_defl_, max_defl_);
            surf_srv_[i].step(dt, target, -qd * hinge_k_[i] * servo_[i], faults.servo_strength[i]);
            servo_[i] = surf_srv_[i].pos(); servo_gear_[i] = servo_[i];
        }
        if (tilt_on_)   // приводи нахилу гондол
            for (int i = 0; i < 4; ++i) {
                ServoActuator& sv = tilt_srv_[size_t(i)];
                if (faults.tilt_stuck[i]) { sv.hold(); tilt_rate_now_[size_t(i)] = 0; continue; }
                const double target = std::clamp(u.tilt[i], tilt_min_, tilt_max_);
                sv.step(dt, target, tilt_load_[size_t(i)], faults.tilt_strength[i]);
                tilt_[size_t(i)] = std::clamp(sv.pos(), tilt_min_ - 0.02, tilt_max_ + 0.02);
                tilt_gear_[size_t(i)] = tilt_[size_t(i)];
                tilt_rate_now_[size_t(i)] = sv.rate();
            }
    }

    static QpState add(const QpState& s, const Deriv& d, double h) {
        QpState r = s;
        r.pos = s.pos + d.dpos * h; r.vel_b = s.vel_b + d.dvel * h; r.omega = s.omega + d.domega * h;
        r.q = Quatd(s.q.w + d.dq[0] * h, s.q.x + d.dq[1] * h, s.q.y + d.dq[2] * h, s.q.z + d.dq[3] * h);
        r.q.normalize();
        return r;
    }
    static Deriv combine(const Deriv& a, const Deriv& b, const Deriv& c, const Deriv& d) {
        Deriv r;
        r.dpos = (a.dpos + b.dpos * 2 + c.dpos * 2 + d.dpos) / 6; r.dvel = (a.dvel + b.dvel * 2 + c.dvel * 2 + d.dvel) / 6;
        r.domega = (a.domega + b.domega * 2 + c.domega * 2 + d.domega) / 6;
        for (int i = 0; i < 4; ++i) r.dq[i] = (a.dq[i] + 2 * b.dq[i] + 2 * c.dq[i] + d.dq[i]) / 6;
        return r;
    }

    // Швидкий режим: аеродинаміка планера рахується раз на крок (1 мс) і однакова для всіх чотирьох стадій RK4 —
    // за 1 мс швидкість і кути майже не змінюються, а це найдорожча частина моделі (смуги крила, слід гвинтів).
    bool aero_freeze_ = true;
    mutable bool aero_cache_ok_ = false;
    bool in_rk_ = false;
    mutable AeroResult aero_cache_;
    Deriv deriv(const QpState& s) const {
        Vec3d F, M;
        evalForces(s, F, M);
        Deriv d;
        d.dpos = s.q.rotate(s.vel_b);
        d.dvel = F / mass_ - s.omega.cross(s.vel_b);
        // gyro: ротор обертається навколо осей тіла, H_r = Σ I_r ω_i
        const Vec3d hr = rotorAngularMomentum();
        const Vec3d Iw = inertia_mat_ * s.omega;
        d.domega = inertia_inv_ * (M - s.omega.cross(Iw + hr));
        const Quatd wq(0, s.omega.x, s.omega.y, s.omega.z), dq = s.q * wq;
        d.dq[0] = 0.5 * dq.w; d.dq[1] = 0.5 * dq.x; d.dq[2] = 0.5 * dq.y; d.dq[3] = 0.5 * dq.z;
        return d;
    }

    // Швидкість повітря в точці r (відн. O) у тілі: v_O + ω×r − вітер
    Vec3d pointAirVel(const QpState& s, const Vec3d& r) const {
        const Vec3d vO = s.vel_b - s.omega.cross(cg_rel_);
        return vO + s.omega.cross(r) - s.q.rotateInv(wind_now_);
    }

    AeroInputs aeroInputs(const QpState& s) const {
        AeroInputs in;
        in.v_body = s.vel_b - s.omega.cross(cg_rel_) - s.q.rotateInv(wind_now_);
        in.omega = s.omega; in.rho = rho_;
        in.aileron = servo_[0]; in.elevator = servo_[1]; in.rudder = servo_[2];
        const double rp = cruise_prop_.diameter() / 2, vax = in.v_body.x;
        const double t = has_cruise_ ? cruise_thrust_ : 0.0;
        const double a_disc = 3.14159265358979323846 * rp * rp;
        const double wi = t > 0.05 ? 0.5 * (-vax + std::sqrt(vax * vax + 2 * t / (rho_ * a_disc))) : 0.0;
        const Vec3d pos = cruise_pos_, gr = gust_rates_;
        const double gain = slip_gain_;
        in.h_agl = s.pos.z; in.up_body = s.q.rotateInv(Vec3d{0, 0, 1});
        // слід підйомних гвинтів (скошена трубка, осьова швидкість напівнескінченного вихрового циліндра w = v_i·(1 + s/√(s²+R²)),
        // стискання до 0.78R, згладжений край): впливає на крило в переходах і на H-хвіст за задніми гвинтами
        std::array<Wake, 4> wk{};
        const int nwk = rotor_wash_ ? rotorWakes(s, wk) : 0;
        const double rr = vtol_prop_.diameter() / 2;
        // повітря в точці тіла: слід маршового гвинта (циліндр 0.85·R позаду диска, до 1.5 м) + просторовий градієнт поривів:
        // δw = p_g·y + q_g·x (крен і тангаж від поривів), δv = r_g·x — турбулентність «хитає» крило нерівномірно по розмаху
        in.wash = [=](const Vec3d& r) {
            Vec3d a{0, gr.z * r.x, gr.x * r.y + gr.y * r.x};
            const double dx = pos.x - r.x;
            if (wi > 0 && dx >= 0 && dx <= 1.5 && std::hypot(r.y - pos.y, r.z - pos.z) <= 0.85 * rp) a.x -= gain * wi;
            a += wakeVel(wk, nwk, rr, r, -1);
            return a;
        };
        return in;
    }

    void evalForces(const QpState& s, Vec3d& F, Vec3d& M) const {
        F = Vec3d{}; M = Vec3d{};
        // аеродинаміка (моменти відносно O → ЦМ)
        if (!in_rk_ || !aero_cache_ok_) { aero_cache_ = aero_.eval(aeroInputs(s)); aero_cache_ok_ = in_rk_; }
        const AeroResult& ar = aero_cache_;
        F += ar.force; M += ar.moment - cg_rel_.cross(ar.force);
        // гвинти на гондолах: тяга вздовж осі гондоли (0 — +Z, 90° — +X), H-сила — проти поперечної швидкості
        for (int i = 0; i < 4; ++i) {
            const Vec3d r = diskPos(i), ax = rotorAxis(i);
            const Vec3d va = pointAirVel(s, r);
            const double v_ax = va.dot(ax);
            const Vec3d perp = va - ax * v_ax;
            const Propeller::Out po = vp(i).eval(vtol_motor_[size_t(i)].rps(), v_ax, perp.norm(), rho_);
            if (rotor_bem_) {   // косий потік — теорія елемента лопаті (множник тяги, сили в площині диска, моменти асиметрії)
                BemCache& bc = bem_c_[size_t(i)];
                if (!bc.valid) bc = bemFactors(i, vtol_motor_[size_t(i)].rps(), v_ax, perp.norm(), po.thrust, po.torque, 0, 0, 8);
                Vec3d f = ax * ((po.thrust + bc.dT) * thrust_k_[size_t(i)] * faults.prop_scale[i] * rotorFactor(s, i, va));
                if (perp.norm() > 0.3) {
                    const Vec3d xh = perp.normalized(), yh = ax.cross(xh);
                    const double kk = thrust_k_[size_t(i)] * faults.prop_scale[i];
                    f += (xh * bc.fx + yh * bc.fy) * kk;
                    M += (xh * bc.mx + yh * bc.my) * kk;
                }
                F += f; M += (r - cg_rel_).cross(f);
            } else {
                Vec3d f = ax * (po.thrust * thrust_k_[size_t(i)] * faults.prop_scale[i] * rotorFactor(s, i, va));
                if (perp.norm() > 1e-6) f -= perp.normalized() * po.h_force;
                F += f; M += (r - cg_rel_).cross(f);
            }
            M += ax * (yaw_sign_[size_t(i)] * motor_torque_[size_t(i)]);   // реакція моменту мотора вздовж осі гвинта
            if (tilt_gyro_ && tilt_on_) {   // гіроскопічний момент від повороту гондоли з ротором, що крутиться: −dH/dt
                const double t = tilt_[size_t(i)];
                M -= Vec3d{std::cos(t), 0, -std::sin(t)} * (vp(i).inertia() * vtol_motor_[size_t(i)].omega() * (-yaw_sign_[size_t(i)]) * tilt_rate_now_[size_t(i)]);
            }
        }
        // маршовий гвинт (вісь +X)
        if (has_cruise_) {
            const Vec3d va = pointAirVel(s, cruise_pos_);
            const Vec3d perp{0, va.y, va.z};
            const Propeller::Out po = cruise_prop_.eval(cruise_motor_.rps(), va.x, perp.norm(), rho_);
            Vec3d f{po.thrust, 0, 0};
            if (perp.norm() > 1e-6) f -= perp.normalized() * po.h_force;
            F += f; M += (cruise_pos_ - cg_rel_).cross(f);
            M.x += reaction_sign_ * cruise_torque_;
        }
        // земля
        for (const Contact& c : contacts_) {
            const Vec3d rc = c.r - cg_rel_;
            const Vec3d pw = s.pos + s.q.rotate(rc);
            const double depth = -pw.z;
            if (depth <= 0) continue;
            const Vec3d vw = s.q.rotate(s.vel_b + s.omega.cross(rc));
            const double fn = std::fmax(0.0, c.k * depth - c.c * vw.z);
            const double vt = std::hypot(vw.x, vw.y);
            const double fr = mu_ * fn / std::sqrt(vt * vt + slip_reg_ * slip_reg_);
            const Vec3d fw{-vw.x * fr, -vw.y * fr, fn};
            const Vec3d fb = s.q.rotateInv(fw);
            F += fb; M += rc.cross(fb);
        }
        // вага
        F += s.q.rotateInv(Vec3d{0, 0, -kG * mass_});
    }

    void stepPropulsion(double dt, const QpControls& u) {
        const QpState& s = st_;
        const bool alive = !battery_.depleted();
        // Напруга шини залежить від струму, а струм моторів — від напруги: розв'язуємо це неявно (бісекція за струмом).
        // Явне «напруга з попереднього кроку» дає коливання з періодом 2 мс при великому опорі батареї й піках 250–300 А.
        auto demand = [&](double v) {
            double i = avionics_w_ / std::fmax(v, 5.0);
            for (int k = 0; k < 4; ++k) i += vtol_motor_[size_t(k)].predictBusCurrent(alive ? u.duty_vtol[k] : 0.0, v, alive && faults.vtol_scale[k] > 1e-6);
            return has_cruise_ ? i + cruise_motor_.predictBusCurrent(alive ? u.duty_cruise : 0.0, v, alive && faults.cruise_scale > 1e-6) : i;
        };
        double lo = 0, hi = 600;
        for (int k = 0; k < 40; ++k) { const double mid = 0.5 * (lo + hi); (demand(std::fmax(battery_.voltage(mid), 0.0)) > mid ? lo : hi) = mid; }
        const double v_bus = std::fmax(battery_.voltage(0.5 * (lo + hi)), 0.0);
        bus_voltage_ = v_bus;
        double i_total = avionics_w_ / std::fmax(v_bus, 5.0);
        std::array<Wake, 4> wk{};
        const int nwk = rotor_rotor_ ? rotorWakes(s, wk) : 0;
        const double rr_d = vtol_prop_.diameter() / 2;
        for (int i = 0; i < 4; ++i) {
            const Vec3d ax = rotorAxis(i);
            Vec3d va = pointAirVel(s, diskPos(i));
            if (nwk > 0) {   // слід інших гвинтів на цьому диску: середнє по центру й чотирьох точках на 0.7R
                const Vec3d c = diskPos(i), e1 = Vec3d{0, 1, 0}, e2 = ax.cross(e1);
                Vec3d w = wakeVel(wk, nwk, rr_d, c, i, &wake_src_);
                for (int k = 0; k < 4; ++k) {
                    const double ang = k * 1.5707963267948966;
                    w += wakeVel(wk, nwk, rr_d, c + (e1 * std::cos(ang) + e2 * std::sin(ang)) * (0.7 * rr_d), i, &wake_src_);
                }
                va = va - w * 0.2;
                ifr_in_[size_t(i)] = -(w * 0.2).dot(ax);
            }
            const double v_ax = va.dot(ax);
            const double v_ip = (va - ax * v_ax).norm();
            const Propeller::Out po = vp(i).eval(vtol_motor_[size_t(i)].rps(), v_ax, v_ip, rho_);
            BemCache& bc = bem_c_[size_t(i)];
            if (rotor_bem_ && (!bc.valid || prop_step_ % 5 == 0))   // 200 Гц, теплий старт індукованого потоку
                bc = bemFactors(i, vtol_motor_[size_t(i)].rps(), v_ax, v_ip, po.thrust, po.torque, bc.vi, bc.vi0, bc.valid ? 3 : 8);
            const double q_load = (po.torque + (rotor_bem_ ? bc.dQ : 0.0)) * faults.prop_scale[i];
            const MotorEsc::Step ms = vtol_motor_[size_t(i)].step(dt, alive ? u.duty_vtol[i] : 0.0, v_bus, q_load, alive && faults.vtol_scale[i] > 1e-6);   // мертвий ESC не гальмує
            motor_torque_[size_t(i)] = ms.motor_torque;
            {   // пульсації тяги у вихровому кільці: низькочастотний шум (τ ≈ 0.3 с) на кожен гвинт
                const double a = std::exp(-dt / 0.3);
                vrs_noise_[size_t(i)] = a * vrs_noise_[size_t(i)] + 0.15 * std::sqrt(1 - a * a) * nd_(rng_);
            }
            vtol_factor_[size_t(i)] = rotorFactor(s, i, va);
            if (tilt_on_) {   // момент на осі нахилу (вісь Y тіла): H-сила гвинта на плечі prop_offset + гіроскопічний момент ротора
                const Vec3d perp = va - ax * v_ax;
                const Vec3d fh = perp.norm() > 1e-6 ? perp.normalized() * (-po.h_force) : Vec3d{};
                const Vec3d arm = ax * (prop_off_ * disk_sgn_[size_t(i)]);
                const Vec3d H = ax * (vp(i).inertia() * vtol_motor_[size_t(i)].omega() * (-yaw_sign_[size_t(i)]));
                tilt_load_[size_t(i)] = arm.cross(fh).y + s.omega.cross(H).y;
            }
            vtol_thrust_[size_t(i)] = (po.thrust + (rotor_bem_ ? bc.dT : 0.0)) * thrust_k_[size_t(i)] * faults.prop_scale[i] * vtol_factor_[size_t(i)];
            i_total += ms.bus_current;
        }
        if (has_cruise_) {
            const Vec3d va = pointAirVel(s, cruise_pos_);
            const Propeller::Out po = cruise_prop_.eval(cruise_motor_.rps(), va.x, std::hypot(va.y, va.z), rho_);
            const MotorEsc::Step ms = cruise_motor_.step(dt, alive ? u.duty_cruise : 0.0, v_bus, po.torque, alive && faults.cruise_scale > 1e-6);
            cruise_torque_ = ms.motor_torque; cruise_thrust_ = po.thrust;
            i_total += ms.bus_current;
        }
        bus_current_ = i_total;
        if (flex_w_[0] > 0) {   // пружність конструкції: перша форма згину крила (крен) і балок гондол (тангаж); бачить лише IMU
            double mx = 0, my = 0;
            for (int i = 0; i < 4; ++i) { const Vec3d f = rotorAxis(i) * vtol_thrust_[size_t(i)], r = diskPos(i); mx += r.y * f.z - r.z * f.y; my += r.z * f.x - r.x * f.z; }
            const double m[2] = {mx, my};
            double* q[2] = {&flex_q_.x, &flex_q_.y}; double* v[2] = {&flex_v_.x, &flex_v_.y};
            for (int k = 0; k < 2; ++k) {
                const double w = flex_w_[k], acc = w * w * (flex_s_ * flex_k_[k] * m[k] - *q[k]) - 2 * flex_z_ * w * *v[k];
                *v[k] += acc * dt; *q[k] += *v[k] * dt;
            }
        }
        battery_.step(dt, i_total);
        ++prop_step_;
    }

    // Множник тяги підйомного гвинта:
    //  ефект землі (Cheeseman–Bennett, 1955): T/T∞ = 1 / (1 − (R/4z)²/(1 + (V/v_h)²)), z ≥ R/2 — висота диска над землею;
    //  вихрове кільце (VRS): у зоні 0.3 < −V_ос/v_h < 1.6 і малій горизонтальній швидкості (μ = V_гор/v_h < 0.8) тяга падає до −25%
    //  і пульсує (±15%·інтенсивність) — межі за Вольковичем/Johnson (NASA TP-2005-213477), амплітуди [О].
    double rotorFactor(const QpState& s, int i, const Vec3d& va) const {
        double k = 1.0;
        const double rr = vtol_prop_.diameter() / 2;
        const double T = std::fmax(vtol_thrust_[size_t(i)], 0.5);
        const double vh = std::sqrt(T / (2 * rho_ * 3.14159265358979323846 * rr * rr));
        const Vec3d ax = rotorAxis(i);
        const double v_ax = va.dot(ax);
        const double mu = (va - ax * v_ax).norm() / vh;
        if (rotor_ge_) {   // лише коли диск дивиться вниз на землю (у польоті літаком гондола горизонтальна — ефекту немає)
            const double z = s.pos.z + s.q.rotate(diskPos(i) - cg_rel_).z;
            const double down = std::clamp(s.q.rotate(ax).z, 0.0, 1.0);
            if (z > 0 && z < 5 * rr && down > 0.3) {
                const double x = rr / (4 * std::fmax(z, 0.5 * rr));
                k /= 1 - down * x * x / (1 + mu * mu);
            }
        }
        if (rotor_vrs_) {
            const double zeta = -v_ax / vh;
            if (zeta > 0.3 && zeta < 1.6) {
                const double b = std::pow(std::sin(3.14159265358979323846 * (zeta - 0.3) / 1.3), 2);
                const double e = std::pow(std::clamp(1 - mu / 0.8, 0.0, 1.0), 2);
                k *= 1 - 0.25 * b * e + vrs_noise_[size_t(i)] * b * e;
            }
        }
        return k;
    }

    void detectCrash() {
        contact_fn_ = 0;
        for (const Contact& c : contacts_) {
            const Vec3d rc = c.r - cg_rel_;
            const Vec3d pw = st_.pos + st_.q.rotate(rc);
            if (pw.z >= 0) continue;
            const Vec3d vw = st_.q.rotate(st_.vel_b + st_.omega.cross(rc));
            contact_fn_ += c.k * -pw.z;
            if (-vw.z > (c.hard ? crash_vz_ : gear_crash_vz_)) crashed_ = true;
        }
        if (std::fabs(st_.q.rotate(Vec3d{0, 0, 1}).z) < 0.1 && contact_fn_ > 0 && st_.pos.z < 0.5) crashed_ = true;   // лежить на боці
    }
};

}  // namespace sim
