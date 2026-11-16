// quad_sim.h — фізика квадрокоптера 6DOF + вітер + дисбаланс
#pragma once
#include <cmath>
#include <random>
#include "../fc/airframe.h"
#include "../fc/battery.h"
#include "../fc/fc_math.h"
#include "portable_rng.h"

namespace sim {
using fc::Vec3;
using fc::Quat;

// Розрахункова конфігурація (див. tools/power_budget.py):
//   рама 700 мм + мотори 4114 + пропелери 15" + електроніка + комп'ютер 500 г + камери 350 г = 2.77 кг
//   + батарея Li-ion 6S4P 18 А·год (~1.8 кг)  =  4.57 кг злітної ваги без вантажу
struct QuadParams {  // «справжній» дрон (може відрізнятися від того, що знає контролер)
    float mass = 4.57f;                         // кг (разом з батареєю і вантажем, якщо є)
    Vec3 inertia{0.12f, 0.12f, 0.22f};          // кг·м²
    float arm_length = 0.35f;                   // м
    float arm_x = 0, arm_y = 0;                 // м — мотори прямокутником (VTOL на балках); 0 — квадрат з arm_length
    float dx() const { return arm_x > 0 ? arm_x : arm_length * 0.70710678f; }
    float dy() const { return arm_y > 0 ? arm_y : arm_length * 0.70710678f; }
    Vec3 motorPos(int i) const { return Vec3{fc::kQuadX[i].sx * dx(), fc::kQuadX[i].sy * dy(), 0}; }
    float k_yaw = 0.025f;                       // м
    float max_thrust = 28.0f;                   // Н на мотор при номінальній напрузі батареї (21.6 В)
    float motor_tau = 0.06f;                    // с, інерція мотора (великі пропелери повільніші)
    float thrust_expo = 0.0f;                   // 0 = лінійна тяга, 1 = квадратична від cmd
    float motor_eff[4] = {1, 1, 1, 1};          // ефективність кожного мотора (знос, пошкоджений пропелер)
    Vec3 com_offset{0, 0, 0};                   // зміщення центру мас (дисбаланс), м
    Vec3 cop_offset{0, 0, 0.08f};               // центр тиску вітру відносно геом. центру, м
    float drag_xy = 0.10f, drag_z = 0.25f;      // Н/(м/с)², опір корпусу
    float rotor_drag = 0.8f;                    // Н/(м/с), лінійний опір гвинтів (горизонтальний, у тілі) при тязі висіння
    float rotor_drag_ref_thrust = 44.8f;        // Н — загальна тяга, при якій заміряно rotor_drag (висіння 4.57 кг)
    float ang_damping = 0.02f;                  // Н·м/(рад/с)
    // Реалізм біля землі й у спуску
    bool ground_effect = true;                  // біля землі гвинти дають більше тяги (Cheeseman–Bennett)
    bool vortex_ring = true;                    // швидкий вертикальний спуск у власному обдуві: втрата й «тремтіння» тяги
    float ground_mu = 0.5f;                     // тертя ніг об землю при ковзанні після приземлення
    float max_touch_hspeed = 6.0f;              // м/с: швидше горизонтально — перекинувся при приземленні
    float rotor_height = 0.15f;                 // м: диски гвинтів над землею, коли центр мас на землі
    fc::AirframeConfig air;                     // крило / кіль / тяговий гвинт (корпус — drag_xy)
};

// Батарея Li-ion 6S4P (21700, напр. Molicel P45B) + модель енергоспоживання
struct BatteryParams {
    int cells = 6;
    float capacity_ah = 18.0f;
    float r_internal = 0.035f;     // Ом — пакет + проводка (просідання під навантаженням)
    float v_nominal_cell = 3.6f;   // при цій напрузі мотор дає max_thrust
    float avionics_w = 30.0f;      // комп'ютер, камери, контролер, GPS, радіо (у симуляторі задає контролер живлення)
    // Інерція ротора: кожен «смик» мотора — розкрутити пропелер і знову загальмувати. Енергія розкручування
    // E = J·ω²/2; при гальмуванні регулятор (з активним гальмуванням) повертає в батарею лише частину.
    // Через це «нервовий» регулятор, шум D-складової і боротьба з кожним поривом реально гріють мотори.
    float rotor_inertia = 4.5e-4f; // кг·м² — пропелер 15" (~30 г) + дзвін мотора 4114
    float omega_max = 733.0f;      // рад/с (~7000 об/хв) при max_thrust
    float regen = 0.3f;            // частка енергії гальмування, що повертається в батарею
    float prop_diameter = 0.381f;  // м (15")
    float figure_of_merit = 0.70f; // якість гвинта
    float drive_eff = 0.80f;       // мотор × регулятор
    float soc0 = 1.0f;             // заряд на старті (1 = 100%)
    float temp_c = 25.0f;          // °C: на морозі опір пакета росте (~2.5%/°C нижче 25) і віддається менше ємності
};

using fc::liionOcv;

struct WindParams {
    Vec3 mean{0, 0, 0};       // м/с, постійний вітер (світ); з shear_exp > 0 — це вітер на висоті ref_height
    float shear_exp = 0;      // 0.14 — типовий профіль над відкритою місцевістю: вище — сильніше
    float ref_height = 10;    // м
    float gust_sigma = 0;     // м/с, інтенсивність поривів (турбулентність)
    float gust_tau = 0.8f;    // с, «довжина» пориву (проста модель)
    // Модель Драйдена (MIL-F-8785C, мала висота): масштаб вихорів росте з висотою (L_u ≈ 180 м на 40 м),
    // тож пориви повільніші, ніж у простій моделі; вертикальні — ~0.6 від горизонтальних. Пориви НЕ
    // множаться на профіль вітру (з висотою σ майже не росте).
    bool dryden = false;
};

struct State {
    Vec3 pos{0, 0, 0}, vel{0, 0, 0};  // світ, Z вгору
    Quat q;                            // body -> world
    Vec3 omega{0, 0, 0};               // рад/с, тіло
};

class QuadSim {
public:
    QuadParams p;
    BatteryParams bat;
    WindParams wind;
    // Стан батареї
    float soc = 1.0f, volts = 25.0f, amps = 0, watts = 0, wh_used = 0;
    float motor_w = 0, rotor_w = 0;   // потужність моторів; з неї — втрати на «смикання» роторів (середнє)
    // Аеродинаміка планера
    float pusher_cmd = 0;             // 0..1 — команда тягового гвинта (пише контролер)
    Vec3 surf_cmd, surf;              // рулі: команда контролера / реальне положення (з інерцією машинок), −1..1
    float pusher_thrust = 0, pusher_w = 0;
    fc::AeroResult aero;
    bool soc_init = false;
    State s;
    float t = 0;
    float thrust[4] = {0, 0, 0, 0};
    Vec3 specific_force_body{0, 0, fc::kGravity};  // те, що «відчуває» акселерометр
    Vec3 wind_now{0, 0, 0};
    Vec3 extra_force_world{0, 0, 0};    // для тестів: поштовх
    Vec3 extra_torque_body{0, 0, 0};
    bool on_ground = true, crashed = false;
    float impact_speed = 0;

    explicit QuadSim(unsigned seed = 1) : rng_(seed) {}

    void step(const float cmd[4], float dt) {
        // --- Вітер: середній + пориви (фільтрований шум) ---
        prng::normal<float> n01(0, 1);
        float k = 1.0f;
        if (wind.shear_exp > 0) k = std::pow(std::fmax(s.pos.z, 0.5f) / wind.ref_height, wind.shear_exp);
        if (wind.gust_sigma > 0 && wind.dryden) {
            const float h_ft = std::fmax(s.pos.z, 3.0f) * 3.281f, m = 0.177f + 0.000823f * h_ft;
            const float Lu = h_ft / std::pow(m, 1.2f) / 3.281f, Lw = h_ft / 3.281f;              // м
            const float U = std::fmax((s.vel - wind.mean * k).norm(), std::fmax(wind.mean.norm() * k, 1.0f));
            const float au = std::fmin(dt * U / Lu, 1.0f), aw = std::fmin(dt * U / Lw, 1.0f);
            const float su = wind.gust_sigma, sw = wind.gust_sigma * std::pow(m, 0.4f);
            gust_.x = gust_.x * (1 - au) + n01(rng_) * su * std::sqrt(2 * au);
            gust_.y = gust_.y * (1 - au) + n01(rng_) * su * std::sqrt(2 * au);
            gust_.z = gust_.z * (1 - aw) + n01(rng_) * sw * std::sqrt(2 * aw);
            wind_now = wind.mean * k + gust_;
        } else if (wind.gust_sigma > 0) {
            float a = dt / wind.gust_tau;
            float kg = wind.gust_sigma * std::sqrt(2 * a);
            gust_ = gust_ * (1 - a) + Vec3{n01(rng_), n01(rng_), 0.3f * n01(rng_)} * kg;
            wind_now = (wind.mean + gust_) * k;
        } else {
            gust_ = Vec3{};
            wind_now = wind.mean * k;
        }

        // --- Мотори: перший порядок ---
        // --- Батарея: напруга під навантаженням і її вплив на тягу ---
        if (!soc_init) { soc = bat.soc0; soc_init = true; volts = liionOcv(soc) * bat.cells; }
        const float v_nom = bat.v_nominal_cell * bat.cells;
        const float vscale = (volts / v_nom) * (volts / v_nom);  // тяга ~ оберти² ~ напруга²
        float T = 0;
        Vec3 torque{0, 0, 0};
        // Ефект землі й вихровий режим: множники до тяги (при тих самих обертах і потужності)
        float ge = 1.0f;
        if (p.ground_effect) {
            const float R = 0.5f * bat.prop_diameter, zr = std::fmax(s.pos.z + p.rotor_height, 0.5f * R);
            const float x = R / (4 * zr);
            ge = fc::clampf(1.0f / (1.0f - x * x), 1.0f, 1.25f);
        }
        float vrs_loss = 0, vrs_unsteady = 0;
        if (p.vortex_ring) {
            const Vec3 ab = s.q.rotateInv(s.vel - wind_now);
            const float Tm = std::fmax((thrust[0] + thrust[1] + thrust[2] + thrust[3]) * 0.25f, 0.5f);
            const float A = 3.14159265f * bat.prop_diameter * bat.prop_diameter / 4;
            const float vh = std::sqrt(Tm / (2 * fc::kRho * A));
            const float vd = -ab.z, Vh = std::sqrt(ab.x * ab.x + ab.y * ab.y);   // швидкість спуску вздовж осі, бічний обдув
            const float r = vd / vh;
            // глибина режиму: максимум при спуску ≈ 0.8 від індукованої швидкості, зникає з бічним потоком
            const float depth = std::exp(-((r - 0.8f) / 0.35f) * ((r - 0.8f) / 0.35f)) * fc::clampf(1.0f - Vh / (0.7f * vh), 0, 1);
            vrs_loss = 0.30f * depth;
            vrs_unsteady = 0.10f * depth;
        }
        for (int i = 0; i < 4; ++i) {
            float c = fc::clampf(cmd[i], 0, 1);
            float target = p.motor_eff[i] * p.max_thrust * ((1 - p.thrust_expo) * c + p.thrust_expo * c * c);
            target *= (volts > 0.5f * v_nom ? vscale : 0.0f);
            thrust[i] += (target - thrust[i]) * (dt / (p.motor_tau + dt));
            {   // реакція на розкручування ротора (J·dω/dt) — у той самий бік, що й реактивний момент опору
                const float kT = p.max_thrust / (bat.omega_max * bat.omega_max);
                const float w = std::sqrt(std::fmax(thrust[i], 0.0f) / kT);
                if (omega_dyn_init_) torque.z += fc::kQuadX[i].yaw_sign * bat.rotor_inertia * (w - omega_dyn_[i]) / dt;
                omega_dyn_[i] = w;
            }
            float eff = ge;
            if (vrs_loss > 0) {   // нестабільний обдув: втрата тяги й повільне випадкове «хитання» кожного гвинта
                vrs_n_[i] += (n01(rng_) * 1.5f - vrs_n_[i]) * (dt / (0.15f + dt));
                eff *= fc::clampf(1.0f - vrs_loss + vrs_unsteady * vrs_n_[i], 0.4f, 1.1f);
            }
            const float ti = thrust[i] * eff;
            Vec3 r = p.motorPos(i);
            r -= p.com_offset;  // момент відносно реального центру мас
            torque += r.cross(Vec3{0, 0, ti});
            torque.z += fc::kQuadX[i].yaw_sign * p.k_yaw * ti;
            T += ti;
        }
        omega_dyn_init_ = true;

        // --- Тяговий (штовхальний) гвинт ---
        {
            const fc::PusherConfig& pu = p.air.pusher;
            const float u = s.q.rotateInv(s.vel - wind_now).x;   // зустрічний потік уздовж осі гвинта
            float target = 0;
            if (pu.enabled) target = fc::clampf(pusher_cmd, 0, 1) * pu.max_thrust * (volts > 0.5f * v_nom ? vscale : 0.0f) *
                                     pu.thrustFactor(u);
            pusher_thrust += (target - pusher_thrust) * (dt / (0.08f + dt));
            pusher_w = pu.enabled ? fc::pusherPower(pu, pusher_thrust, u) : 0.0f;
        }

        updateBattery(dt);

        // --- Аеродинаміка ---
        Vec3 v_rel = s.vel - wind_now;
        const float vh = std::sqrt(v_rel.x * v_rel.x + v_rel.y * v_rel.y);  // опір ~ |v|·v (не залежить від напрямку)
        Vec3 drag{-p.drag_xy * v_rel.x * vh, -p.drag_xy * v_rel.y * vh, -p.drag_z * v_rel.z * std::fabs(v_rel.z)};
        Vec3 vrb = s.q.rotateInv(v_rel);
        // опір гвинтів ~ обертам (∝ √тяги): коли крило тримає вагу і гвинти майже стоять — опір значно менший
        // (зупинені гвинти вже враховано в опорі корпусу — тут лише ті, що крутяться)
        const float tr = std::fmax(T, 0.0f) / p.rotor_drag_ref_thrust;
        const float rd = p.rotor_drag * (0.15f * fc::clampf(tr / 0.05f, 0, 1) + 0.85f * std::sqrt(tr));
        drag += s.q.rotate(Vec3{-rd * vrb.x, -rd * vrb.y, 0});
        Vec3 drag_body = s.q.rotateInv(drag);
        torque += (p.cop_offset - p.com_offset).cross(drag_body);
        // крило, кіль, тяговий гвинт
        if (p.air.hasAero() || p.air.pusher.enabled) {
            const float disk = 3.14159265f * bat.prop_diameter * bat.prop_diameter / 4;
            aero = fc::airframeForces(p.air, vrb, s.omega, p.com_offset, T, disk);
            Vec3 fb = aero.force + Vec3{pusher_thrust, 0, 0};
            drag += s.q.rotate(fb);
            torque += aero.torque + (Vec3{p.air.pusher.x, 0, 0} - p.com_offset).cross(Vec3{pusher_thrust, 0, 0});
            // рулі: машинки повертаються з запізненням; момент — від потоку вздовж носа
            if (p.air.surfaces.enabled) {
                surf += (surf_cmd - surf) * (dt / (p.air.surfaces.tau + dt));
                const float u = std::fmax(vrb.x, 0.0f);
                torque += fc::surfaceMoments(p.air.surfaces, p.air.wing, 0.5f * fc::kRho * u * u, surf.x, surf.y, surf.z);
            }
        } else {
            aero = fc::AeroResult{};
        }
        torque -= s.omega * p.ang_damping;
        torque += extra_torque_body;

        // --- Поступальний рух ---
        Vec3 f_thrust = s.q.rotate(Vec3{0, 0, T});
        Vec3 f_nongrav = f_thrust + drag + extra_force_world;
        Vec3 acc = f_nongrav / p.mass - Vec3{0, 0, fc::kGravity};

        // --- Земля ---
        if (on_ground) {
            if (acc.z <= 0) {  // ще не відірвались: земля тримає (нормальна реакція + тертя)
                f_nongrav = Vec3{0, 0, p.mass * fc::kGravity};
                acc = Vec3{0, 0, 0};
                // горизонтальна швидкість після приземлення гаситься тертям ніг (ковзання), а не зникає миттєво
                {
                    float hs = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y);
                    if (hs > 1e-4f) {
                        const float k = std::fmax(hs - p.ground_mu * fc::kGravity * dt, 0.0f) / hs;
                        s.vel.x *= k; s.vel.y *= k;
                        s.pos.x += s.vel.x * dt; s.pos.y += s.vel.y * dt;
                    }
                    s.vel.z = 0;
                }
                // стоїть на ногах на рівній землі: якщо нахилений — плавно «падає» на ноги
                // (через кутову швидкість, щоб гіроскоп це бачив), обертання гасить тертя
                Vec3 u = s.q.rotateInv(Vec3{0, 0, 1});
                Vec3 axis{-u.y, u.x, 0};
                float ang = std::acos(fc::clampf(u.z, -1, 1));
                s.omega = axis.norm() > 1e-6f ? axis.normalized() * (ang * 15.0f) : Vec3{0, 0, 0};
                s.q.integrate(s.omega, dt);
                torque = Vec3{0, 0, 0};
                ground_hold_ = true;
            } else {
                on_ground = false;
            }
        }
        if (ground_hold_) {  // на землі: решту динаміки обертання пропускаємо
            ground_hold_ = false;
            specific_force_body = s.q.rotateInv(f_nongrav / p.mass);
            t += dt;
            return;
        }
        specific_force_body = s.q.rotateInv(f_nongrav / p.mass);

        // --- Обертальний рух: I·dω = τ − ω×(Iω) ---
        Vec3 Iw = p.inertia.mul(s.omega);
        Vec3 dw = torque - s.omega.cross(Iw);
        dw = Vec3{dw.x / p.inertia.x, dw.y / p.inertia.y, dw.z / p.inertia.z};
        s.omega += dw * dt;
        s.q.integrate(s.omega, dt);

        s.vel += acc * dt;
        s.pos += s.vel * dt;
        if (s.pos.z < 0) {
            impact_speed = -s.vel.z;
            float tilt = std::acos(fc::clampf(s.q.cosTilt(), -1, 1)) * fc::kRad2Deg;
            const float hs = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y);
            if (impact_speed > 3.0f || tilt > 60 || hs > p.max_touch_hspeed) crashed = true;
            s.pos.z = 0;
            s.vel = Vec3{crashed ? 0.0f : s.vel.x, crashed ? 0.0f : s.vel.y, 0};
            on_ground = true;
        }
        t += dt;
    }

private:
    // Потужність з теорії імпульсу гвинта: P = T^1.5 / sqrt(2·ρ·A) / (FM·η) + набір висоти + бортова електроніка
    // Потужність гвинтів з теорії імпульсу (Глауерт): індукована швидкість залежить від
    // обдуву гвинта збоку (поступальний рух/вітер, «ефект косого обдуву» — у русі гвинт ефективніший)
    // і від осьового потоку (набір висоти — дорожче, спуск — дешевше).
    //   v_i = v_h² / sqrt(V² + (v_c + v_i)²),  v_h = sqrt(T / (2ρA)),  P = T·(v_c + v_i) / (FM·η)
    void updateBattery(float dt) {
        const float A = 3.14159265f * bat.prop_diameter * bat.prop_diameter / 4;
        Vec3 air_b = s.q.rotateInv(s.vel - wind_now);           // потік відносно дрона, у системі тіла
        const float V = std::sqrt(air_b.x * air_b.x + air_b.y * air_b.y);  // обдув у площині гвинтів
        const float vc = air_b.z;                                 // >0 — дрон іде вгору відносно повітря
        float p_mech = 0;
        const float kT = p.max_thrust / (bat.omega_max * bat.omega_max);
        // профільна потужність лопатей (не зникає в русі, ∝ Ω³, росте з μ²) — підібрана так, щоб на висінні
        // (тяга ~11 Н на гвинт) сума була такою ж, як T·v_h/FM
        const float Th = 0.25f * p.rotor_drag_ref_thrust, vhh = std::sqrt(Th / (2 * 1.225f * A));
        const float P0h = Th * vhh / bat.figure_of_merit - 1.15f * Th * vhh, Omh = std::sqrt(Th / kT);
        float p_shaft[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            float ti = std::fmax(thrust[i], 0.0f);
            if (ti < 1e-3f) continue;
            float vh2 = ti / (2 * 1.225f * A);
            float vi = std::sqrt(vh2);
            for (int it = 0; it < 6; ++it) {  // кілька ітерацій вистачає
                float d = std::sqrt(V * V + (vc + vi) * (vc + vi));
                vi = 0.5f * vi + 0.5f * vh2 / std::fmax(d, 0.05f);
            }
            float pw = 1.15f * ti * std::fmax(vc + vi, 0.25f * std::sqrt(vh2));  // на швидкому спуску — не нижче профільних втрат
            const float Om = std::sqrt(ti / kT), mu = V / std::fmax(Om * bat.prop_diameter / 2, 1.0f);
            pw += P0h * std::pow(Om / Omh, 3.0f) * (1 + 4.65f * mu * mu);
            p_shaft[i] = pw;
            p_mech += pw / bat.drive_eff;
        }
        // Розкручування/гальмування роторів (ω = sqrt(T/kT)): енергія обертання разом із потужністю на валу.
        // Коли ротор гальмує, його енергія йде в тягу (вал отримує менше від мотора) — втрата лише тоді, коли
        // ротор гальмують швидше, ніж повітря забирає енергію (тоді — рекуперація з втратами).
        float p_rot = 0;
        for (int i = 0; i < 4; ++i) {
            float w = std::sqrt(std::fmax(thrust[i], 0.0f) / kT);
            if (omega_init_) {
                const float pd = 0.5f * bat.rotor_inertia * (w * w - omega_[i] * omega_[i]) / dt, tot = p_shaft[i] + pd;
                const float elec = tot > 0 ? tot / bat.drive_eff : tot * bat.regen * bat.drive_eff;
                p_rot += elec - p_shaft[i] / bat.drive_eff;
            }
            omega_[i] = w;
        }
        omega_init_ = true;
        // струм регулятора обмежений і згладжений: енергія та сама, але без нереальних піків (запуск моторів)
        p_rot_f_ += (p_rot - p_rot_f_) * (dt / (0.05f + dt));
        p_rot = p_rot_f_;
        p_mech += p_rot;
        rotor_w += (p_rot - rotor_w) * (dt / 2.0f);  // середнє за ~2 с — для показу
        p_mech += pusher_w;
        p_mech_ = p_mech;
        motor_w = p_mech;
        watts = p_mech + bat.avionics_w;
        const float ocv = liionOcv(soc) * bat.cells;
        // V = OCV − I·R,  I = P / V  ->  V² − OCV·V + P·R = 0
        // опір пакета росте на морозі й наприкінці розряду (діапазон реалістичний для Li-ion 21700)
        const float cold = std::fmax(0.0f, 25.0f - bat.temp_c);
        const float r_eff = bat.r_internal * (1 + 0.025f * cold) * (1 + 0.6f * fc::clampf((0.25f - soc) / 0.25f, 0, 1));
        float disc = ocv * ocv - 4 * watts * r_eff;
        volts = disc > 0 ? 0.5f * (ocv + std::sqrt(disc)) : 0.5f * ocv;
        amps = watts / std::fmax(volts, 1.0f);
        const float cap_eff = bat.capacity_ah * (1 - 0.006f * std::fmin(cold, 40.0f));   // на морозі віддається менше
        soc = std::fmax(0.0f, soc - amps * dt / (cap_eff * 3600.0f));
        wh_used += watts * dt / 3600.0f;
    }
    std::mt19937 rng_;
    float p_mech_ = 0;
    float omega_[4] = {0, 0, 0, 0}, omega_dyn_[4] = {0, 0, 0, 0}, vrs_n_[4] = {0, 0, 0, 0};
    bool omega_init_ = false, omega_dyn_init_ = false;
    float p_rot_f_ = 0;
    Vec3 gust_{0, 0, 0};
    bool ground_hold_ = false;
};

}  // namespace sim
