#pragma once
// qp_types.h — типи автопілота конвертоплана: таблиці тяги, фази польоту, кроки місії, команди приводам,
// конфігурація (що контролер знає про апарат) і ручне керування. Окремо від логіки режимів (qp_mission.h).
#include <algorithm>
#include <cmath>
#include <vector>
#include "qp_control.h"

namespace fc {

// duty(T, V): 2D-таблиця для маршового гвинта (тяга падає з V за фіксованого duty)
class ThrustMap2D {
public:
    void add(float v, const ThrustMap& m) { v_.push_back(v); maps_.push_back(m); }
    float maxThrust(float v) const {
        if (maps_.empty()) return 0;
        size_t i = 0; while (i + 1 < v_.size() && v_[i + 1] <= v) ++i;
        return maps_[i].maxThrust();
    }
    float fromDuty(float duty, float v, float v_bus) const {
        if (maps_.empty()) return 0;
        if (v <= v_.front()) return maps_.front().fromDuty(duty, v_bus);
        if (v >= v_.back()) return maps_.back().fromDuty(duty, v_bus);
        size_t hi = size_t(std::upper_bound(v_.begin(), v_.end(), v) - v_.begin());
        const float f = (v - v_[hi - 1]) / (v_[hi] - v_[hi - 1]);
        return maps_[hi - 1].fromDuty(duty, v_bus) * (1 - f) + maps_[hi].fromDuty(duty, v_bus) * f;
    }
    bool valid() const { return !maps_.empty(); }
    float toDuty(float thrust, float v, float v_bus) const {
        if (maps_.empty()) return 0;
        if (v <= v_.front()) return maps_.front().toDuty(thrust, v_bus);
        if (v >= v_.back()) return maps_.back().toDuty(thrust, v_bus);
        size_t hi = size_t(std::upper_bound(v_.begin(), v_.end(), v) - v_.begin());
        const float f = (v - v_[hi - 1]) / (v_[hi] - v_[hi - 1]);
        return maps_[hi - 1].toDuty(thrust, v_bus) * (1 - f) + maps_[hi].toDuty(thrust, v_bus) * f;
    }
private:
    std::vector<float> v_;
    std::vector<ThrustMap> maps_;
};

enum class QpPhase { OnGround, Takeoff, VtolHold, TransFwd, Cruise, TransBack, Landing, Landed, FwLand, Manual };
inline const char* phaseName(QpPhase p) {
    static const char* n[] = {"на землі", "зліт", "висіння", "перехід у літак", "літак", "перехід у коптер", "посадка", "сів", "посадка літаком", "ручне керування"};
    return n[int(p)];
}

struct QpStep {
    enum Type { Takeoff, FlyTo, HoverAt, Land } type = Takeoff;
    Vec3 p;               // x, y — ціль; z — висота
    float hold_s = 0;     // HoverAt: скільки висіти над точкою, с
};

struct QpActuators {
    float duty_vtol[4] = {0, 0, 0, 0};
    float duty_cruise = 0;
    float aileron = 0, elevator = 0, rudder = 0;
    float tilt[4] = {0, 0, 0, 0};   // рад: кут гондол (конвертоплан), 0 — вгору, π/2 — уперед
};

struct QpFlightConfig {
    float wind_kp_boost = 0, wind_ki_boost = 0;   // у висінні в сильному вітрі (повітряна швидкість 5…9 м/с → 0…повністю): швидкісний контур жорсткіший
    float dist_ff_xy = 0.0f, dist_ff_z = 0.0f, dist_ff_tau = 0.3f;
    float hold_gps_tau = 0.0f;    // с: у висінні положення — інтеграл швидкості, GPS лише повільно підтягує (0 — звичайне положення GPS)
    float gust_ff = 0.0f;          // поправка тяги на порив: зміну підйомної сили крила за зміною V² з ПВТ, коефіцієнт вивчається в польоті (0 — вимкнено)
    float wind_wing_up = 6 * kDeg2Rad;   // рад: у висінні в зустрічному вітрі ≥ 10 м/с ніс угору на стільки (крило несе частину ваги); 6…10 м/с — плавно
    QpAirframe af;
    ThrustMap vtol_map;
    // задні мотори й гвинти інші, ніж передні (rear_differs): свої таблиці; індекси 1 і 3 — задні
    bool rear_differs = false;
    ThrustMap vtol_map_r, vtol_rps_map_r;
    ThrustMap2D vtol_map2_r;
    float prop_d_r = 0;
    std::vector<float> prop_ct_r;
    static bool isRear(int i) { return i == 1 || i == 3; }
    const ThrustMap& vmap(int i) const { return rear_differs && isRear(i) ? vtol_map_r : vtol_map; }
    const ThrustMap& vrps(int i) const { return rear_differs && isRear(i) ? vtol_rps_map_r : vtol_rps_map; }
    const ThrustMap2D& vmap2(int i) const { return rear_differs && isRear(i) ? vtol_map2_r : vtol_map2; }
    float yaw_dt_gain = 4.0f;        // Н·м на радіан команди керма напряму
    bool yaw_diff_thrust = false;    // у польоті літаком рискання різницею тяги лівих і правих моторів (керма напряму немає)
    ThrustMap vtol_rps_map;          // duty → оберти (об/с) підйомного мотора при vref — очікування для контролю відмов за телеметрією ESC
    ThrustMap2D cruise_rps_map;      // те саме для маршового: duty → оберти за повітряною швидкістю
    ThrustMap2D cruise_map;
    float v_cruise = 17.0f;
    float cruise_thrust_max = 14.0f; // Н: найбільша тяга маршового гвинта (з моделі, див. qp_flight.h)
    float v_max = 22.0f;             // м/с: у сильному вітрі крейсерська швидкість зростає до max(v_cruise, вітер + 8), але не вище
    float v_trans = 12.0f;           // м/с: з цієї повітряної швидкості починається blend
    float v_back = 10.5f;            // м/с: з цієї швидкості гвинти повертаються при переході у коптер
    float blend_time = 4.0f;         // с
    float cruise_alt = 60.0f;
    float assist_speed = 11.0f;      // Q_ASSIST_SPEED: нижче — коптерні мотори підхоплюють (вище звалювання 9.5–10 м/с, як радить ArduPilot)
    float assist_angle = 35 * kDeg2Rad;
    float assist_alt = 8.0f;
    float accept_radius = 45.0f;
    float brake_distance = 110.0f;
    float takeoff_climb = 2.0f;
    float landing_rate = 0.8f;
    float i_max = 90.0f;             // А: ліміт струму батареї 6S2P 21700 (≈ 45 А на комірку, P42A); вище — обмежувач знижує всі duty
    float v_low = 20.9f;             // В: напруга спокою (з компенсацією просідання) нижче — аварійне повернення й посадка (RTL); 3.48 В/комірка ≈ 20% заряду
    float batt_r_est = 0.06f;        // Ом: оцінка внутрішнього опору для компенсації просідання (як BATT_FS_VOLTSRC=1 в ArduPilot)
    float fwd_pitch_lim = 3 * kDeg2Rad;   // Q_FWD_PIT_LIM: у висінні ніс униз не більше, решту — маршовим гвинтом (< 0 — вимкнено)
    bool weathervane = true;         // Q_WVANE: у висінні розвертатись носом проти вітру
    float wvane_gain = 1.0f;         // 1/с: швидкість розвороту на радіан крену
    float fw_land_speed = 13.0f;     // м/с: посадка літаком (≈ 1.3·Vs)
    float fw_glide_deg = 5.0f;       // кут глісади посадки літаком
    // конвертоплан: поворотні гондоли замість маршового мотора
    bool tilt = false;
    float tilt_min = -0.21f, tilt_max = 1.6f, tilt_rate = 1.57f, tilt_tau = 0.05f;   // рад, рад/с, с — як привід у моделі
    float motor_y[4] = {0, 0, 0, 0};       // м: бічне положення гондол (тіло, відн. ЦМ) — для рискання різницевим нахилом
    bool tilt_yaw_vector = true;           // рискання у висінні нахилом гондол
    int cruise_rear = 1;                   // у польоті літаком: 0 — тягнуть лише передні, 1 — усі 4 порівну, 2 — задні допомагають, коли переднім не вистачає
    float front_cruise_cap = 3.0f;         // Н на передній гвинт для cruise_rear = 2
    float tilt_brake_n = 3.0f;             // Н: найбільша сила назад нахилом гондол у висінні (Q_FWD_THR_USE «назад»)
    float back_decel = 1.4f;               // м/с²: темп зниження цільової швидкості на гальмуванні коптером (фаза B переходу назад)
    ThrustMap2D vtol_map2;                 // тяга гвинта гондоли за duty й осьовою швидкістю (у переході й польоті потік набігає вздовж осі)
    // модель тяги за обертами (телеметрія ESC): T = k·CT(J)·ρ·n²·D⁴ — для оцінки зовнішньої сили на борту
    float prop_d = 0.33f;
    float prop_k[4] = {1, 1, 1, 1};
    std::vector<float> prop_j, prop_ct;
    float rotorThrust(int i, float rps, float v_axial, float rho = 1.225f) const {
        if (rps <= 0.5f || prop_j.size() < 2) return 0.0f;
        const bool rr = rear_differs && isRear(i) && prop_ct_r.size() == prop_j.size();
        const float pd = rr ? prop_d_r : prop_d;
        const std::vector<float>& pct = rr ? prop_ct_r : prop_ct;
        const float J = v_axial / (rps * pd);
        size_t k = 1;
        while (k + 1 < prop_j.size() && prop_j[k] < J) ++k;
        const float f = (J - prop_j[k - 1]) / (prop_j[k] - prop_j[k - 1]);
        const float ct = std::fmax(-0.05f, pct[k - 1] + f * (pct[k] - pct[k - 1]));
        const float d2 = pd * pd;
        return prop_k[i] * ct * rho * rps * rps * d2 * d2;
    }
};

// Ручне керування коптером (клавіатура/геймпад): швидкості в осях курсу −1…1, швидкість підйому −1…1, кутова швидкість рискання −1…1
struct QpManual {
    // режими (як в ArduPilot QuadPlane): Assisted ≈ QLOITER — швидкості й висота, точку тримає автопілот;
    // QStabilize — кути й тяга напряму; QAcro — кутові швидкості й тяга; Fbwa — літак, кути крену/тангажу + газ маршового;
    // Manual — літак, рулі й газ напряму, без жодної стабілізації. У Fbwa/Manual підйомні мотори вимкнено.
    enum Mode { Assisted, QStabilize, QAcro, Fbwa, Manual };
    Mode mode = Assisted;
    float roll = 0, pitch = 0, yaw = 0;   // ручки −1…1: + правий крен, + ніс униз (ручка від себе), + курс праворуч
    float throttle = 0;                   // газ 0…1: коптер — колективна тяга, літак — маршовий мотор
    float pusher = 0;                     // газ маршового в коптерних режимах 0…1 (для ручного переходу в літак); конвертоплан: −1…0 — гондоли назад (гальмування)
    // чутливість (як на RC-пульті): експо — м'якше біля центру ручки; найбільші кути й кутова швидкість
    float expo = 0.35f, max_angle_deg = 25.0f, acro_rate_dps = 180.0f, fbwa_bank_deg = 40.0f, fbwa_pitch_deg = 20.0f;
    float shaped(float x) const { x = clampf(x, -1.0f, 1.0f); return (1 - expo) * x + expo * x * x * x; }
    bool on = false;
    float fwd = 0, left = 0, climb = 0, yaw_rate = 0;
    float speed_max = 8.0f, climb_max = 3.0f;
    bool direct() const { return on && mode != Assisted; }
};

}  // namespace fc
