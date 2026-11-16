#pragma once
// Параметри автопілота у файлі (як .param в ArduPilot): «ІМ'Я = значення», # — коментар.
// На залізі ті самі імена читаються з SD-карти / EEPROM, тож налаштування переносяться без перекомпіляції.
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>
#include "qp_mission.h"

namespace fc {

struct QpParam { const char* name; float* f = nullptr; bool* b = nullptr; int* i = nullptr; const char* about = ""; };

inline std::vector<QpParam> qpParamList(QpFlight& fl) {
    QpFlightConfig& c = fl.cfg;
    QpSafetyConfig& s = fl.safety;
    auto F = [](const char* n, float& v, const char* a) { QpParam p; p.name = n; p.f = &v; p.about = a; return p; };
    auto B = [](const char* n, bool& v, const char* a) { QpParam p; p.name = n; p.b = &v; p.about = a; return p; };
    auto I = [](const char* n, int& v, const char* a) { QpParam p; p.name = n; p.i = &v; p.about = a; return p; };
    std::vector<QpParam> list = {
        F("MASS_KG", c.af.mass, "маса без вантажу, кг"),
        F("V_CRUISE", c.v_cruise, "крейсерська повітряна швидкість, м/с"), F("V_MAX", c.v_max, "найбільша швидкість, м/с"),
        F("V_TRANS", c.v_trans, "швидкість завершення переходу, м/с"), F("V_BACK", c.v_back, "швидкість початку переходу назад, м/с"),
        F("BLEND_TIME", c.blend_time, "змішування коптер/літак, с"), F("CRUISE_ALT", c.cruise_alt, "висота польоту літаком, м"),
        F("ASSIST_SPEED", c.assist_speed, "Q_ASSIST_SPEED, м/с"), F("ASSIST_ANGLE", c.assist_angle, "Q_ASSIST_ANGLE, рад"),
        F("ASSIST_ALT", c.assist_alt, "Q_ASSIST_ALT, м"), F("ACCEPT_RADIUS", c.accept_radius, "радіус досягнення точки, м"),
        F("BRAKE_DIST", c.brake_distance, "гальмівна відстань переходу назад, м"), F("TKOFF_CLIMB", c.takeoff_climb, "швидкість зльоту, м/с"),
        F("LAND_RATE", c.landing_rate, "швидкість посадки, м/с"), F("I_MAX", c.i_max, "ліміт струму батареї, А"),
        F("BATT_V_LOW", c.v_low, "аварійна напруга спокою, В"), F("BATT_R", c.batt_r_est, "внутрішній опір батареї, Ом"),
        F("FWD_PIT_LIM", c.fwd_pitch_lim, "Q_FWD_PIT_LIM, рад"), F("WIND_WING", c.wind_wing_up, "висіння на крилі у вітрі ≥ 10 м/с: ніс угору, рад (0 — вимкнено)"), B("WVANE", c.weathervane, "носом проти вітру"),
        F("WVANE_GAIN", c.wvane_gain, "швидкість розвороту, 1/с"), F("FW_LAND_SPD", c.fw_land_speed, "посадка літаком, м/с"),
        F("FW_GLIDE_DEG", c.fw_glide_deg, "глісада, °"), F("TILT_RATE", c.tilt_rate, "швидкість гондол, рад/с"),
        B("TILT_YAW_VEC", c.tilt_yaw_vector, "рискання нахилом гондол"), I("CRUISE_REAR", c.cruise_rear, "0/1/2 — задні гвинти в крейсері"),
        F("FRONT_CAP_N", c.front_cruise_cap, "тяга переднього гвинта для CRUISE_REAR=2, Н"), F("TILT_BRAKE_N", c.tilt_brake_n, "гальмування гондолами, Н"),
        F("BACK_DECEL", c.back_decel, "гальмування коптером, м/с²"),
        B("ARMING_CHECK", s.preflight, "передпольотні перевірки"), F("ARM_GYRO_MAX", s.arm_gyro_max, "рад/с"),
        F("ARM_LEVEL_MAX", s.arm_level_max, "рад"), F("ARM_TILT_ERR", s.arm_tilt_err, "рад"), F("ARM_PITOT_MAX", s.arm_pitot_max, "м/с"),
        F("ARM_V_MARGIN", s.arm_v_margin, "В понад BATT_V_LOW"), F("FENCE_RADIUS", s.fence_radius, "м, 0 — вимкнено"),
        F("FENCE_ALT", s.fence_alt, "м над домом, 0 — вимкнено"), F("FENCE_MARGIN", s.fence_margin, "далі — посадка на місці"),
        F("FS_RC_TIMEOUT", s.rc_timeout_s, "с без пульта в ручному режимі"),
        B("MASS_ADAPT", fl.mass_adapt, "оцінка маси в польоті"),
        I("FS_RC_AUTO", s.rc_auto_action, "втрата пульта в автоматичному польоті: 0 — летіти далі, 1 — додому, 2 — сісти на місці"),
    };
    auto pid = [&](const char* pre, PidGains& g, const char* about) {
        static std::deque<std::string> names;   // імена живуть увесь час роботи (рядки в QpParam — const char*; deque не переносить рядки)
        const char* suf[5] = {"_P", "_I", "_D", "_IMAX", "_FLTD"};
        float* v[5] = {&g.kp, &g.ki, &g.kd, &g.i_limit, &g.d_lpf_hz};
        for (int k = 0; k < 5; ++k) {
            const std::string n = std::string(pre) + suf[k];
            size_t idx = names.size();
            for (size_t j = 0; j < names.size(); ++j) if (names[j] == n) idx = j;
            if (idx == names.size()) names.push_back(n);
            QpParam p; p.name = names[idx].c_str(); p.f = v[k]; p.about = about; list.push_back(p);
        }
    };
    QpGains& g = c.af.gains;
    pid("PSC_VELZ", g.vz, "регулятор вертикальної швидкості (коптер)");
    pid("PSC_VELXY", g.vxy, "регулятор горизонтальної швидкості (коптер)");
    pid("Q_RAT_RP", g.rate_rp, "кутова швидкість крену й тангажу (коптер)");
    pid("Q_RAT_YAW", g.rate_yaw, "кутова швидкість рискання (коптер)");
    pid("FW_RLL", g.fw_roll, "кутова швидкість крену (літак)");
    pid("FW_PTCH", g.fw_pitch, "кутова швидкість тангажу (літак)");
    return list;
}

inline bool saveQpParams(QpFlight& fl, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "# параметри автопілота конвертоплана\n");
    for (const QpParam& p : qpParamList(fl)) {
        if (p.f) std::fprintf(f, "%-14s = %.6g    # %s\n", p.name, double(*p.f), p.about);
        else if (p.b) std::fprintf(f, "%-14s = %d    # %s\n", p.name, *p.b ? 1 : 0, p.about);
        else if (p.i) std::fprintf(f, "%-14s = %d    # %s\n", p.name, *p.i, p.about);
    }
    std::fclose(f);
    return true;
}

// Повертає кількість застосованих; невідомі імена — у unknown
inline int loadQpParams(QpFlight& fl, const std::string& path, std::vector<std::string>* unknown = nullptr) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return -1;
    std::vector<QpParam> list = qpParamList(fl);
    const float mass0 = fl.cfg.af.mass;
    char line[512];
    int n = 0;
    while (std::fgets(line, sizeof line, f)) {
        std::string s(line);
        const size_t h = s.find('#'); if (h != std::string::npos) s.resize(h);
        const size_t eq = s.find('='); if (eq == std::string::npos) continue;
        auto trim = [](std::string t) { const size_t a = t.find_first_not_of(" \t\r\n"), b = t.find_last_not_of(" \t\r\n"); return a == std::string::npos ? std::string() : t.substr(a, b - a + 1); };
        const std::string k = trim(s.substr(0, eq)), v = trim(s.substr(eq + 1));
        if (k.empty() || v.empty()) continue;
        bool hit = false;
        for (const QpParam& p : list) {
            if (k != p.name) continue;
            const double x = std::strtod(v.c_str(), nullptr);
            if (p.f) *p.f = float(x); else if (p.b) *p.b = x != 0; else if (p.i) *p.i = int(x);
            hit = true; ++n; break;
        }
        if (!hit && unknown) unknown->push_back(k);
    }
    std::fclose(f);
    fl.configure(fl.cfg);   // нові коефіцієнти регуляторів
    if (fl.cfg.af.mass != mass0) { const float m = fl.cfg.af.mass; fl.cfg.af.mass = mass0; fl.setMass(m, fl.cfg.af.inertia); }
    return n;
}

}  // namespace fc
