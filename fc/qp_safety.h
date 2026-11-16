#pragma once
// Безпека польоту: передпольотні перевірки перед армуванням, геозона, втрата зв'язку з пультом.
// Логіка — як у ArduPilot (ARMING_CHECK, FENCE_*, FS_*), спрощена під цей апарат.
#include <cmath>
#include <cstdint>
#include <string>
#include "qp_control.h"

namespace fc {

struct QpSafetyConfig {
    bool preflight = true;
    float arm_ok_s = 1.0f;           // перевірки мають проходити стільки поспіль
    float arm_gyro_max = 0.15f;      // рад/с — апарат нерухомий
    float arm_level_max = 0.35f;     // рад — стоїть рівно (≈ 20°)
    float arm_tilt_err = 0.09f;      // рад — гондоли вертикально за датчиками (≈ 5°)
    float arm_pitot_max = 15.0f;     // м/с — на землі Піто показує лише вітер; більше — датчик несправний або вітер нельотний
    float arm_v_margin = 0.6f;       // В понад v_low
    float fence_radius = 0.0f;       // м від дому; 0 — вимкнено (типово вимкнено; увімкнути — параметр FENCE_RADIUS)
    float fence_alt = 400.0f;        // м над домом; 0 — вимкнено
    float fence_margin = 1.3f;       // за межею × margin — сідати на місці, не повертатись
    float rc_timeout_s = 1.0f;       // ручне керування без пульта довше — повернення
    int rc_auto_action = 0;          // пульт пропав в автоматичному польоті: 0 — виконувати місію далі (як FS_LONG_ACTN=0), 1 — додому, 2 — сісти на місці
};

enum PreflightBit : uint32_t {
    kPfEstimator = 1u << 0, kPfGps = 1u << 1, kPfBattery = 1u << 2, kPfMoving = 1u << 3, kPfLevel = 1u << 4,
    kPfTiltSensor = 1u << 5, kPfTiltPos = 1u << 6, kPfEsc = 1u << 7, kPfPitot = 1u << 8, kPfRc = 1u << 9,
};

inline std::string preflightText(uint32_t f) {
    static const char* n[] = {"оцінювач не готовий", "немає GPS", "низька напруга", "апарат рухається", "стоїть нерівно",
                              "немає датчика гондоли", "гондола не вертикально", "немає телеметрії ESC", "Піто показує забагато", "немає пульта"};
    std::string s;
    for (int i = 0; i < 10; ++i) if (f & (1u << i)) { if (!s.empty()) s += ", "; s += n[i]; }
    return s;
}

inline uint32_t preflightCheck(const QpEstimate& e, const QpSafetyConfig& c, float v_low, bool tilt, bool need_rc) {
    uint32_t f = 0;
    if (!e.est_ok) f |= kPfEstimator;
    if (!e.gps_ok) f |= kPfGps;
    if (e.v_bus < v_low + c.arm_v_margin) f |= kPfBattery;
    if (std::sqrt(e.omega.x * e.omega.x + e.omega.y * e.omega.y + e.omega.z * e.omega.z) > c.arm_gyro_max) f |= kPfMoving;
    const Vec3 up = e.q.rotate(Vec3{0, 0, 1});
    if (std::acos(std::fmin(1.0f, std::fmax(-1.0f, up.z))) > c.arm_level_max) f |= kPfLevel;
    if (tilt) for (int i = 0; i < 4; ++i) {
        if (e.tilt_meas[i] < -5) f |= kPfTiltSensor;
        else if (std::fabs(e.tilt_meas[i]) > c.arm_tilt_err) f |= kPfTiltPos;
    }
    for (int i = 0; i < 4; ++i) if (e.vtol_rps[i] < 0) f |= kPfEsc;
    if (e.airspeed > c.arm_pitot_max) f |= kPfPitot;
    if (need_rc && !e.rc_ok) f |= kPfRc;
    return f;
}

}  // namespace fc
